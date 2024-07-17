#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rosgraph_msgs.msg import Clock
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import math
import time

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
        
        # Declare parameters
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)
        self.declare_parameter('range_min', 1.0)
        self.declare_parameter('range_max', 50.0)
        self.declare_parameter('scan_time', 1.0 / 30.0)
        self.declare_parameter('frame_id', 'laser_frame')

        # Get parameter values
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        self.frame_id = self.get_parameter('frame_id').value


        # Subscribe to the PointCloud2 and Clock topics
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/lidar',
            self.pointcloud_callback,
            10
        )

        # Publisher for LaserScan
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)


    def pointcloud_callback(self, msg):
        # Process the point cloud data to create a laser scan
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ranges = np.full(int((self.angle_max - self.angle_min) / self.angle_increment), np.inf)
        # start timer
        start = time.time()

        for point in points:
            x, y, _ = point
            angle = math.atan2(y, x)
            if self.angle_min <= angle <= self.angle_max:
                range_index = int((angle - self.angle_min) / self.angle_increment)
                range_distance = math.sqrt(x ** 2 + y ** 2)
                if self.range_min <= range_distance <= self.range_max:
                    ranges[range_index] = min(ranges[range_index], range_distance)

        

        # end timer
        end = time.time()
        print(f"Time taken: {end - start}")

        # Create and populate LaserScan message
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        scan.scan_time = self.scan_time

        # Publish the LaserScan message
        self.scan_publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
