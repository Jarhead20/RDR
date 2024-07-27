import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
import math
import numpy as np
import cv2
import time


class CameraToPointsNode(Node):
    def __init__(self):
        super().__init__('camera_to_points_node')

        # create camera height paramerter
        self.declare_parameter('ground_height', 10)
        self.declare_parameter('ground_width', 10)
        self.declare_parameter('camera_height', 10)

        # get camera height parameter
        self.camera_height = self.get_parameter('camera_height').value
        self.ground_height = self.get_parameter('ground_height').value
        self.ground_width = self.get_parameter('ground_width').value

        self.subscription = self.create_subscription(
            CompressedImage,
            'mask',
            self.binary_mask_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/scan', 10)

    def binary_mask_callback(self, msg):
        # convert to image
        # self.mask = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        # get the rgb image and convert to grayscale
        self.mask = np.frombuffer(msg.data, np.uint8)
        self.mask = cv2.imdecode(self.mask, cv2.IMREAD_COLOR)
        self.mask = cv2.cvtColor(self.mask, cv2.COLOR_BGR2GRAY)

        
        # print(self.mask)

        # threshold the image
        _, self.mask = cv2.threshold(self.mask, 127, 255, cv2.THRESH_BINARY)
        # cv2.imshow('mask', self.mask)
        # cv2.waitKey(1)
        # return
        self.scan_degrees = 36

        # show mask
        # cv2.imshow('mask', self.mask)
        # cv2.waitKey(0)

        # Convert binary mask to laser scan message
        laser_scan_msg = LaserScan()
        laser_scan_msg.header.stamp = msg.header.stamp
        laser_scan_msg.header.frame_id = "laser_frame"
        laser_scan_msg.angle_increment = math.pi / self.scan_degrees
        laser_scan_msg.angle_min = -math.pi / 2
        laser_scan_msg.angle_max = math.pi / 2 - laser_scan_msg.angle_increment
        
        laser_scan_msg.range_min = 1.0
        laser_scan_msg.range_max = 50.0
        laser_scan_msg.scan_time = 1.0 / 30.0
        
        points = []

        middle = int(self.mask.shape[1] / 2)

        self.mask2 = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2BGR)

        # add the middle point
        points.append((middle, self.mask.shape[0]-1))

        angle = 0


        increament = 180/self.scan_degrees

        j = 0

        # sweep the image from left to right with the midpoint at the bottom middle of the image
        # for x in range(0, 180, increament):
        while j < 180:
            j += increament

            # concentrate more scans in the middle of the image sinusoidally keeping the angle between 0 and 180
            add = ((math.cos(math.radians(2*j)) + 1) * increament)
            

            tx = math.cos(math.radians(angle))
            ty = math.sin(math.radians(angle))

            angle += add
            for distance in range(0, self.mask.shape[1]):
                y = self.mask.shape[0] - int(ty*distance) -1
                x = middle + int(distance * tx)
                if x < 0 or x >= self.mask.shape[1] or y < 0 or y >= self.mask.shape[0]:
                    # if no point is found, set it to max range
                    x = middle
                    y = self.mask.shape[0] - 1
                    # x = middle + int(tx * 1000)
                    # y = self.mask.shape[0] - int(ty * 1000)
                    cv2.circle(self.mask2, (x,y), 3, (0, 255, 0), -1)
                    points.append((x, y))
                    break

                # visualise the point
                cv2.line(self.mask2, (middle, self.mask.shape[0]-1), (x, y), (255, 0, 0), 1)
                if self.mask[y, x] == 255:
                    # visualise on mask2
                    cv2.circle(self.mask2, (x,y), 3, (0, 0, 255), -1)
                    # # draw a line from the middle to the point
                    # cv2.imshow('mask5', self.mask2)
                    # cv2.waitKey(0)
                    points.append((x, y))
                    break
        

        # # show midpoint
        cv2.circle(self.mask2, (middle, self.mask.shape[0]-1), 1, (0, 255, 0), -1)
        



        

        # show mask

        # load the distortion matrix
        npzfile = np.load('calibration/CalibrationMatrix_college_cpt.npz')
        mtx = npzfile['Camera_matrix']

        # print(points)
        points = np.array([points], dtype=np.float32)

        # undistort the points
        # points = cv2.undistortPoints(points, mtx, None)

        # print(points)
        
        # load calibration file from calibration/BirdsEyeMatrix_college_cpt.npz
        npzfile = np.load('calibration/BirdsEyeMatrix_college_cpt.npz')
        matrix = npzfile['M']
        # print(matrix)
        print(points.shape)

        # apply perspective transform to the points
        points = cv2.perspectiveTransform(points, matrix)
        # print(points)

        # cv2.imshow('mask5', self.mask2)
        # cv2.waitKey(0)

        # copy the mask to mask3
        self.mask3 = np.copy(self.mask)

        # convert to 3 colour image
        # self.mask3 = cv2.cvtColor(self.mask3, cv2.COLOR_GRAY2BGR)
        self.mask3 = np.zeros((self.mask.shape[0]*2, self.mask.shape[1]*2, 3), dtype=np.uint8)

        last_angle = 0

        counter = 0

        # print(points)

        p = points.squeeze()

        midpoint = p[0]

        # remove the 1st point
        p = p[1:]

        # a = int(math.pi / laser_scan_msg.angle_increment)

        # iterate through the points and calculate the distance and angle
        ranges = []
        

        laser_angle = 0

        c1 = 0
        c2 = 0
        c3 = 0
        c4 = 0

        # determine the scale of the distance to be in meters from the calibration file
        corners = np.array([[0, 0], [0, 640], [480, 640], [480, 0]], dtype=np.float32)
        corners = cv2.perspectiveTransform(np.array([corners], dtype=np.float32), matrix)

        dist1 = corners[0][2][1] - corners[0][1][1]

        # calculate the width that the camera would see when it is 2m off the ground and with a 90 degree fov
        ground_width = 2 * math.tan(math.pi / 4) * self.camera_height

        scale = 480/dist1
        # print("scale ", scale)
        scale = ground_width * scale
        print("scale ", scale)


        for point in p:
            dist = math.sqrt((point[0] - midpoint[0])**2 + (point[1] - midpoint[1])**2)
            # if dist < laser_scan_msg.range_min or dist > laser_scan_msg.range_max:
            #     continue
            point = point / scale
            point[1] += self.mask.shape[1]
            point[0] += self.mask.shape[0]
            # point[1] *= -1
            print(point, midpoint, dist)
            cv2.circle(self.mask3, (int(point[0]), self.mask.shape[0]-int(point[1])), 3, (0, 0, 255), -1)
            cv2.line(self.mask3, (int(midpoint[0]), self.mask.shape[0]-int(midpoint[1])), (int(point[0]), self.mask.shape[0]-int(point[1])), (255, 0, 0), 1)
        # cv2.imshow('mask3', self.mask3)
        # cv2.waitKey(0)

        # self.get_logger().info(f'p = {p.shape}')

        while(laser_angle < math.pi and counter < len(p)):
            x = p[counter][1]
            y = p[counter][0]

            # calculate the distance from the midpoint
            distance = math.sqrt((x - midpoint[1])**2 + (y - midpoint[0])**2)

            # scale distance
            distance = distance / scale

            # calculate the angle
            image_angle = math.atan2(y - midpoint[0], x - midpoint[1])

            counter += 1

            print(laser_angle, image_angle, distance, counter, x, y)

            

            if laser_angle < image_angle:
                c1 += 1
                laser_angle -= laser_scan_msg.angle_increment
                ranges.append(distance)
                # cv2.circle(self.mask3, (int(x), self.mask.shape[0]-int(y)), 3, (0, 0, 255), -1)

                # cv2.line(self.mask3, (int(midpoint[1]), self.mask.shape[0]-int(midpoint[0])), (int(x), self.mask.shape[0]-int(y)), (255, 0, 0), 1)
                # cv2.imshow('mask', self.mask3)
                # cv2.waitKey(0)
                continue
            
            if distance < laser_scan_msg.range_min or distance > laser_scan_msg.range_max:
                c2 += 1
                ranges.append(laser_scan_msg.range_max)
                laser_angle += laser_scan_msg.angle_increment
                continue

            if image_angle > laser_angle:
                c3 += 1
                laser_angle += laser_scan_msg.angle_increment
                ranges.append(laser_scan_msg.range_max)
                continue

            c4 += 1
            ranges.append(distance)
            
            

        # print(c1, c2, c3)
        # log using ros2 logger
        # self.get_logger().info(f'c1 = {c1}, c2 = {c2}, c3 = {c3}, c4 = {c4}')

        # 

        # Set the ranges
        laser_scan_msg.ranges = ranges
        # print(ranges)
        # print("len" + len(ranges))
        print("len" + str(len(ranges)))

        # Publish the laser scan message
        self.publisher.publish(laser_scan_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_to_points_node = CameraToPointsNode()
    rclpy.spin(camera_to_points_node)
    camera_to_points_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()