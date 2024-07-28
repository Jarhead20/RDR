import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformListener, Buffer
import tf_transformations
import numpy as np
import cv2

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        # Subscribers
        self.path_subscriber = self.create_subscription(
            Path,
            '/track_path',
            self.path_callback,
            10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            Twist,
            '/drive',
            10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.follow_path)
        
        # tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize position and heading
        self.position = None
        self.theta = None
        self.path = None

        self.invert_direction = True

        # Velocity
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.look_ahead_distance = 0.1
        
    def path_callback(self, msg):
        # Extract path from message
        self.path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])

    def odom_callback(self, msg):
        # Update velocity from odometry
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def update_tf_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.position = np.array([transform.transform.translation.x, transform.transform.translation.y])
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            euler = tf_transformations.euler_from_quaternion(quaternion)
            self.theta = euler[2]
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % str(e))


    def follow_path(self):
        self.update_tf_position()
        if self.position is None or self.path is None:
            return

        # Update position using the velocity
        dt = 0.1  # Time step (seconds)
        # self.position[0] += self.linear_velocity * np.cos(self.theta) * dt
        # self.position[1] += self.linear_velocity * np.sin(self.theta) * dt
        # self.theta += self.angular_velocity * dt
        
        self.pure_pursuit = PurePursuit(self.path, self.look_ahead_distance)
        steering_value = self.pure_pursuit.control(self.position, self.theta, self.invert_direction)
        
        # Create a Twist message to publish the steering and power values
        drive_msg = Twist()
        power_value = 0.5
        drive_msg.linear.x = power_value
        drive_msg.angular.z = steering_value

        print(steering_value)
        
        self.visualise_path()
        
        # Publish the drive message
        self.publisher.publish(drive_msg)

    def visualise_path(self):
        # Create a new image
        self.map2 = np.zeros((480, 800, 3), dtype=np.uint8) * 255
        # Show the points on the path and the current position, set 240, 320 as the origin
        for point in self.path:
            # Scale the points to fit the image
            point = point * 5
            cv2.circle(self.map2, (int(point[0] + 320), int(point[1] + 240)), 2, (0, 0, 255), -1)
            
        cv2.circle(self.map2, (int(self.position[0] * 5 + 320), int(self.position[1] * 5 + 240)), 5, (255, 0, 0), -1)
        # Show current steering vector
        target_point = self.path[self.pure_pursuit.current_index]
        cv2.line(self.map2, (int(self.position[0] * 5 + 320), int(self.position[1] * 5 + 240)), (int(target_point[0] * 5 + 320), int(target_point[1] * 5 + 240)), (0, 255, 0), 2)
        cv2.imshow('Path', self.map2)
        cv2.waitKey(1)

class PurePursuit:
    def __init__(self, path, look_ahead_distance):
        self.path = path
        self.look_ahead_distance = look_ahead_distance
        self.current_index = 0

    def find_closest_point(self, position):
        # Find the closest point on the path to the current position
        distances = np.linalg.norm(self.path - position, axis=1)
        closest_point_index = np.argmin(distances)
        return closest_point_index

    def get_target_point(self, position, invert_direction):
        # while True:
            # Calculate distance from the current position to the next waypoint
            # distance = np.linalg.norm(self.path[self.current_index] - position)
            # if distance >= self.look_ahead_distance or self.current_index == len(self.path) - 1:
            #     break
            # self.current_index += 1
        closest_index = self.find_closest_point(position)
        if invert_direction:
            self.current_index = closest_index - 1
        else:
            self.current_index = closest_index + 1
        return self.path[self.current_index]

    def control(self, position, heading, invert_direction):
        target_point = self.get_target_point(position, invert_direction)
        angle_to_target = np.arctan2(target_point[1] - position[1], target_point[0] - position[0])
        steering_angle = angle_to_target - heading

        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    path_follower_node = PathFollowerNode()
    rclpy.spin(path_follower_node)
    path_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
