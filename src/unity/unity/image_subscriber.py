import rclpy
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from rclpy.node import Node
import cv2
import numpy as np
import os

class ImageSubscriber(Node):
        
        

        def __init__(self):
            super().__init__('image_subscriber')


            # check if calibration file exists
            if not os.path.exists('calibration/CalibrationMatrix_college_cpt.npz'):
                print('calibration file not found')
                # initialise variables mtx and dist
                self.mtx = None
                self.dist = None
                return
            else:
                print('calibration file found')
                data = np.load('calibration/CalibrationMatrix_college_cpt.npz')
                self.mtx = data['Camera_matrix']
                self.dist = data['distCoeff']
                print('calibration file loaded')

            # check if warp file exists
            if not os.path.exists('calibration/BirdsEyeMatrix_college_cpt.npz'):
                print('warp file not found')
                # initialise variables mtx and dist
                self.M = None
                return
            else:
                print('warp file found')
                data = np.load('calibration/BirdsEyeMatrix_college_cpt.npz')
                self.M = data['M']
                print('warp file loaded')

            self.time = None

            self.subscription = self.create_subscription(Image, 'image', self.listener_callback, 10)

            # subscribe to /clock
            self.subscription = self.create_subscription(Clock, 'clock', self.clock_callback, 10)


            self.subscription  # prevent unused variable warning

            self.mask_publisher = self.create_publisher(Image, 'mask', 10)
            # self.yellow_mask_publisher = self.create_publisher(Image, 'yellow_mask', 10)
            self.blue_lower = np.array([114, 112, 153])
            self.blue_upper = np.array([153,232,255])
            self.yellow_lower = np.array([21,121,224])
            self.yellow_upper = np.array([44,240,255])

            # create a slider gui for the lower and upper hsv values
            cv2.namedWindow('hsv')
            # cv2.createTrackbar('blue_h_low', 'hsv', 0, 255, self.on_trackbar)
            # cv2.createTrackbar('blue_s_low', 'hsv', 0, 255, self.on_trackbar)
            # cv2.createTrackbar('blue_v_low', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('blue_h_high', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('blue_s_high', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('blue_v_high', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('yellow_h_low', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('yellow_s_low', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('yellow_v_low', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('yellow_h_high', 'hsv', 0, 255, self.on_trackbar)
            # # cv2.createTrackbar('yellow_s_high', 'hsv', 0, 255, self.on_trackbar)
            # cv2.createTrackbar('yellow_v_high', 'hsv', 0, 255, self.on_trackbar)
            # create a button to save the current image

            cv2.createTrackbar('save_image', 'hsv', 0, 1, self.save_image)

            cv2.imshow('hsv', np.zeros((1, 300, 3), np.uint8))

        def clock_callback(self, msg):
            self.time = msg.clock.sec + msg.clock.nanosec * 1e-9

        def on_trackbar(self, val):
            try:
                self.blue_lower[0] = cv2.getTrackbarPos('blue_h_low', 'hsv')
                self.blue_lower[1] = cv2.getTrackbarPos('blue_s_low', 'hsv')
                self.blue_lower[2] = cv2.getTrackbarPos('blue_v_low', 'hsv')
                self.blue_upper[0] = cv2.getTrackbarPos('blue_h_high', 'hsv')
                self.blue_upper[1] = cv2.getTrackbarPos('blue_s_high', 'hsv')
                self.blue_upper[2] = cv2.getTrackbarPos('blue_v_high', 'hsv')
                self.yellow_lower[0] = cv2.getTrackbarPos('yellow_h_low', 'hsv')
                self.yellow_lower[1] = cv2.getTrackbarPos('yellow_s_low', 'hsv')
                self.yellow_lower[2] = cv2.getTrackbarPos('yellow_v_low', 'hsv')
                self.yellow_upper[0] = cv2.getTrackbarPos('yellow_h_high', 'hsv')
                self.yellow_upper[1] = cv2.getTrackbarPos('yellow_s_high', 'hsv')
                self.yellow_upper[2] = cv2.getTrackbarPos('yellow_v_high', 'hsv')
            except:
                pass

        def save_image(self, state):
            if(state == 0):
                return
            # save file with unique name
            name = 'calibration/image' + str(np.random.randint(0, 1000)) + '.png'
            cv2.imwrite(name, self.image)
            # set the trackbar to 0
            cv2.setTrackbarPos('save_image', 'hsv', 0)
            print('image saved')
    
        def listener_callback(self, msg):
            mask = self.convert_image_message_to_cv(msg)
            self.publish_mask(mask)

        def filter_image(self, img, lower, upper):
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            # combine masks
            # mask = cv2.bitwise_not(mask)
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            return mask
            
    
        def convert_image_message_to_cv(self, msg):
            np_arr = np.frombuffer(msg.data, np.uint8)
            np_arr = np_arr.reshape(msg.height, msg.width, 3)
            # flip the image
            np_arr = cv2.flip(np_arr, 0)
            # convert to bgr
            np_arr = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)

            self.image = np_arr
            # warp image based on calibration
            # if self.mtx is not None:
            #     np_arr = cv2.undistort(np_arr, self.mtx, self.dist, None)
            # if self.M is not None:
            #     np_arr = self.birdseye(np_arr)
            # cv2.imshow('image', np_arr)
            
            # hsv filter image
            blue_mask = self.filter_image(np_arr, self.blue_lower, self.blue_upper)
            yellow_mask = self.filter_image(np_arr, self.yellow_lower, self.yellow_upper)
            # combine the mask
            mask = cv2.bitwise_or(blue_mask, yellow_mask)
            # cv2.imshow('mask', mask)
            # cv2.imshow('blue_mask', blue_mask)
            # cv2.imshow('yellow_mask', yellow_mask)
            # cv2.waitKey(1)
            return mask
        
        def publish_mask(self, yellow):
            if yellow is None:
                return
            if self.time is None:
                return
            msg = Image()

            msg.header.stamp = self.create_time_msg(self.time)
            msg.header.frame_id = 'mask'

            msg.height = yellow.shape[0]
            msg.width = yellow.shape[1]
            msg.encoding = 'bgr8'
            msg.data = yellow.tobytes()
            self.mask_publisher.publish(msg)
            
            # msg = Image()
            # msg.height = blue.shape[0]
            # msg.width = blue.shape[1]
            # msg.encoding = 'bgr8'
            # msg.data = blue.tobytes()
            # self.blue_mask_publisher.publish(msg)

        def birdseye(self, img):
            warped = cv2.warpPerspective(img, self.M, (img.shape[1], img.shape[0]))
            return warped
        
        def create_time_msg(self, current_time):
            time_msg = Time()
            time_msg.sec = int(current_time)
            time_msg.nanosec = int((current_time - int(current_time)) * 1e9)
            return time_msg


        
        
def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSubscriber()

    while rclpy.ok():
        rclpy.spin_once(image_sub)

    image_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()