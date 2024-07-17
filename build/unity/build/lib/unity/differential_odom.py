#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from rosgraph_msgs.msg import Clock
import tf_transformations
import math
import tf2_ros
from builtin_interfaces.msg import Time

class DiffOdomNode(Node):
    def __init__(self):
        super().__init__('diff_odom_node')

        # Use simulation time
        # self.declare_parameter('use_sim_time', True)

        # Parameters for the robot
        self.declare_parameter('wheel_separation', 1.56)
        self.declare_parameter('wheel_radius', 0.5)
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.multiplier = 1  # Adjust this to scale the acceleration

        # Initialize pose and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Time variables for computing dt
        self.last_time = None
        self.current_time = None

        # Subscription for the clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        # Subscriptions for the wheel topics
        self.subscription_left_wheel = self.create_subscription(
            Float32,
            'left_wheel',
            self.left_wheel_callback,
            10
        )

        self.subscription_right_wheel = self.create_subscription(
            Float32,
            'right_wheel',
            self.right_wheel_callback,
            10
        )

        # Subscription for the IMU topic
        self.subscription_imu = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Variables to hold encoder readings
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        # Previous encoder readings for delta calculation
        self.prev_left_wheel_position = None
        self.prev_right_wheel_position = None

    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def left_wheel_callback(self, msg):
        new_left_position = msg.data * self.multiplier
        if self.prev_left_wheel_position is not None:
            delta_left_wheel = new_left_position - self.prev_left_wheel_position
            self.prev_left_wheel_position = new_left_position
            self.compute_odometry(delta_left_wheel, None)
        else:
            self.prev_left_wheel_position = new_left_position

    def right_wheel_callback(self, msg):
        new_right_position = msg.data * self.multiplier
        if self.prev_right_wheel_position is not None:
            delta_right_wheel = new_right_position - self.prev_right_wheel_position
            self.prev_right_wheel_position = new_right_position
            self.compute_odometry(None, delta_right_wheel)
        else:
            self.prev_right_wheel_position = new_right_position

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = tf_transformations.euler_from_quaternion(orientation_list)

    def compute_odometry(self, delta_left_wheel, delta_right_wheel):
        if self.current_time is None or self.last_time is None:
            self.last_time = self.current_time
            return

        dt = self.current_time - self.last_time

        if delta_left_wheel is not None:
            left_distance = delta_left_wheel * self.wheel_radius * 2 * math.pi
        else:
            left_distance = 0.0

        if delta_right_wheel is not None:
            right_distance = delta_right_wheel * self.wheel_radius * 2 * math.pi
        else:
            right_distance = 0.0

        delta_distance = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation

        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)

        self.vx = delta_distance / dt if dt > 0.0 else 0.0
        self.vth = delta_theta / dt if dt > 0.0 else 0.0

        self.publish_odometry(self.current_time)
        self.last_time = self.current_time

    def publish_odometry(self, current_time):
        odom = Odometry()
        odom.header.stamp = self.create_time_msg(current_time)
        odom.header.frame_id = 'odom'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Position covariance
        odom.pose.covariance[0] = 1 # x
        odom.pose.covariance[7] = 1 # y
        odom.pose.covariance[35] = 0.1 # yaw 

        # Twist covariance
        odom.twist.covariance[0] = 1 # x
        odom.twist.covariance[7] = 1 # y
        odom.twist.covariance[35] = 0.1 # yaw

        # Velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        # Publish the odometry message
        self.odom_publisher.publish(odom)

        # Publish the transform over TF
        transform = TransformStamped()
        transform.header.stamp = self.create_time_msg(current_time)
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform)

    def create_time_msg(self, current_time):
        time_msg = Time()
        time_msg.sec = int(current_time)
        time_msg.nanosec = int((current_time - int(current_time)) * 1e9)
        return time_msg

def main(args=None):
    rclpy.init(args=args)
    node = DiffOdomNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
