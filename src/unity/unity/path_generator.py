import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import os
import tf2_ros
import tf_transformations

class TrackPathPlanner(Node):
    def __init__(self):
        super().__init__('track_path_planner')

        self.declare_parameter('map_resolution', 1.0)  # Resolution in meters per pixel
        self.declare_parameter('map_origin', [0.0, 0.0])  # Origin of the map

        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin = self.get_parameter('map_origin').get_parameter_value().double_array_value

        self.path_publisher = self.create_publisher(Path, 'track_path', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path = None

    def map_callback(self, msg):
        self.get_logger().info('Received new map data.')
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map = self.occupancy_grid_to_image(msg)
        self.path = self.generate_path()
        self.publish_path()        

    def occupancy_grid_to_image(self, occupancy_grid):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))
        map_image = np.zeros((height, width), dtype=np.uint8)
        map_image[data == -1] = 205  # Unknown
        map_image[data == 0] = 255  # Free space
        map_image[data > 0] = 0  # Obstacles
        return map_image

    def generate_path(self):
        if self.map is None:
            return None
        
        position = None
        theta = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            position = [transform.transform.translation.x, transform.transform.translation.y]
            theta = tf_transformations.euler_from_quaternion(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
            )[2]
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

        if position is None or theta is None:
            self.get_logger().error("Could not get position and heading.")
            return None

        # Set all values that aren't white to black
        self.map[self.map < 250] = 0

        # visualize the map
        # cv2.imshow('Map', self.map)
        # cv2.waitKey(0)

        # expand the white area to make sure the track is closed
        

        # Morphological operations to remove noise
        kernel = np.ones((7, 7), np.uint8)
        self.map = cv2.morphologyEx(self.map, cv2.MORPH_OPEN, kernel)

        # cv2.imshow('Map', self.map)
        # cv2.waitKey(0)

        kernel = np.ones((7, 7), np.uint8)
        self.map = cv2.dilate(self.map, kernel)

        # cv2.imshow('Map', self.map)
        # cv2.waitKey(0)
        
        # Skeletonize the map
        self.map = cv2.ximgproc.thinning(self.map)

        # cv2.imshow('Map', self.map)
        # cv2.waitKey(0)


        contours, _ = cv2.findContours(self.map, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        self.get_logger().info(f"Found {len(contours)} contours in the map.")

        if len(contours) == 0:
            self.get_logger().error("No contours found in the map.")
            return None

        # Get the largest contour
        track_contour = max(contours, key=cv2.contourArea)

        # Convert to 3 channel image for visualization
        self.map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

        # Visualize contour
        cv2.drawContours(self.map, [track_contour], -1, (0, 255, 0), 2)

        # Generate points along the contour
        epsilon = 0.001 * cv2.arcLength(track_contour, True)
        approx = cv2.approxPolyDP(track_contour, epsilon, True)

        path_points = [tuple(point[0]) for point in approx]

        # Convert to ROS Path message
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        poses = []

        for point in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0] * self.map_resolution + self.map_origin[0]
            pose.pose.position.y = point[1] * self.map_resolution + self.map_origin[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            # path.poses.append(pose)
            poses.append(pose)

        
        # reorder the points to start from the closest point to the current position, maintaining the order of the points
        if position is not None:
            # Find the closest point on the path to the initial position
            # closest_point_index = min(range(len(poses)), key=lambda i: np.linalg.norm(np.array(poses[i]) - np.array(position)))
            # path_points = path_points[closest_point_index:] + path_points[:closest_point_index]
            closest_point_index = min(range(len(poses)), key=lambda i: np.linalg.norm(np.array([poses[i].pose.position.x, poses[i].pose.position.y]) - np.array(position)))
            poses = poses[closest_point_index:] + poses[:closest_point_index]

        print(f"Closest point index: {closest_point_index}")
        # print(poses)
        path.poses = poses
        # Visualize path
        for point in path_points:
            cv2.circle(self.map, point, 2, (0, 0, 255), -1)

        # Increase map size for visualization
        self.map = cv2.resize(self.map, (0, 0), fx=1, fy=1)

        # cv2.imshow('Map', self.map)
        # cv2.waitKey(1) & 0XFF

        return path

    def publish_path(self):
        if self.path:
            self.path_publisher.publish(self.path)
        else:
            self.get_logger().warn("Path is not generated yet.")

def main(args=None):
    rclpy.init(args=args)
    node = TrackPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
