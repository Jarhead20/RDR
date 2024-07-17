import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
import numpy as np
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')

        self.multiplier = 1  # Adjust this to scale the acceleration
        self.wheel_radius = 0.5  # Wheel radius in meters

        self.current_velocity = np.zeros(2)  # 2D velocity (x, y)
        self.current_orientation = 0.0  # Yaw in radians
        self.current_position = np.zeros(2)  # 2D position (x, y)
        self.previous_time = None

        self.initial_orientation = None  # To store the initial orientation
        self.acc_bias = np.zeros(2)  # Bias for acceleration

        self.scaling_factor = 1.0  # Apply scaling if needed

        # Kalman filter state
        self.state = np.zeros(4)  # [pos_x, pos_y, vel_x, vel_y]
        self.covariance = np.eye(4)  # State covariance matrix

        # Process and measurement noise
        self.process_noise = np.eye(4) * 0.01
        self.measurement_noise = np.eye(2) * 0.1

        # Add clock subscription
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.left_encoder_subscription = self.create_subscription(
            Float32,
            '/left_wheel',
            self.left_encoder_callback,
            10
        )

        self.right_encoder_subscription = self.create_subscription(
            Float32,
            '/right_wheel',
            self.right_encoder_callback,
            10
        )

        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.prev_left_wheel_velocity = 0
        self.prev_right_wheel_velocity = 0

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_time = None  # Initialize current time

    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def imu_callback(self, msg):
        if self.current_time is None:
            return

        if self.previous_time is None:
            self.previous_time = self.current_time
            self.initial_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            return

        dt = self.current_time - self.previous_time

        if dt <= 0:
            return

        linear_acceleration = np.array([msg.linear_acceleration.x,
                                        msg.linear_acceleration.y]) * self.scaling_factor
        
        orientation_q = [msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z,
                         msg.orientation.w]
        _, _, current_yaw = euler_from_quaternion(orientation_q)

        initial_yaw = euler_from_quaternion(self.initial_orientation)[2]
        relative_yaw = current_yaw - initial_yaw
        self.current_orientation = relative_yaw

        corrected_acceleration = self.correct_for_gravity(linear_acceleration, relative_yaw)

        corrected_acceleration -= self.acc_bias

        self.kalman_predict(corrected_acceleration, dt)
        
        self.publish_odometry(self.current_time)
        self.publish_transform(self.current_time)

        self.previous_time = self.current_time

    def left_encoder_callback(self, msg):
        self.left_wheel_velocity = msg.data * self.multiplier * self.wheel_radius * 2 * np.pi
        self.update_wheel_velocity()

    def right_encoder_callback(self, msg):
        self.right_wheel_velocity = msg.data * self.multiplier * self.wheel_radius * 2 * np.pi
        self.update_wheel_velocity()

    def update_wheel_velocity(self):
        if self.left_wheel_velocity is not None and self.right_wheel_velocity is not None:
            # Apply threshold to detect if the wheel velocities are effectively zero
            threshold = 0.01
            if abs(self.left_wheel_velocity - self.prev_left_wheel_velocity) < threshold and abs(self.right_wheel_velocity - self.prev_right_wheel_velocity) < threshold:
                avg_wheel_velocity = 0.0
            else:
                avg_wheel_velocity = (self.left_wheel_velocity + self.right_wheel_velocity) / 2.0
            
            self.kalman_update(avg_wheel_velocity)
            
            self.prev_left_wheel_velocity = self.left_wheel_velocity
            self.prev_right_wheel_velocity = self.right_wheel_velocity

    def kalman_predict(self, acceleration, dt):
        F = np.eye(4) + np.array([
            [0, 0, dt, 0],
            [0, 0, 0, dt],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])  # State transition model

        B = np.array([
            [0.5 * dt**2, 0],
            [0, 0.5 * dt**2],
            [dt, 0],
            [0, dt]
        ])  # Control input model

        u = acceleration  # Control input

        self.state = F @ self.state + B @ u
        self.covariance = F @ self.covariance @ F.T + self.process_noise

    def kalman_update(self, wheel_velocity):
        H = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])  # Measurement model

        R = self.measurement_noise  # Measurement noise

        z = np.array([wheel_velocity, wheel_velocity])  # Measurement

        y = z - H @ self.state  # Measurement residual
        S = H @ self.covariance @ H.T + R  # Residual covariance
        K = self.covariance @ H.T @ np.linalg.inv(S)  # Kalman gain

        self.state = self.state + K @ y
        self.covariance = (np.eye(4) - K @ H) @ self.covariance

        self.current_position = self.state[:2]
        self.current_velocity = self.state[2:]

    def correct_for_gravity(self, acceleration, yaw):
        return acceleration

    def publish_odometry(self, current_time):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.create_time_msg(current_time)
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.current_position[0]
        odom_msg.pose.pose.position.y = self.current_position[1]
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.current_orientation)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.linear.z = 0.0

        self.odom_publisher.publish(odom_msg)

    def publish_transform(self, current_time):
        transform = TransformStamped()
        transform.header.stamp = self.create_time_msg(current_time)
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.current_position[0]
        transform.transform.translation.y = self.current_position[1]
        transform.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, self.current_orientation)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)

    def create_time_msg(self, current_time):
        time_msg = Time()
        time_msg.sec = int(current_time)
        time_msg.nanosec = int((current_time - int(current_time)) * 1e9)
        return time_msg

def main(args=None):
    rclpy.init(args=args)
    imu_odometry_node = ImuOdometryNode()
    rclpy.spin(imu_odometry_node)
    imu_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
