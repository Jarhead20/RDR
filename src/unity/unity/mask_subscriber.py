import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
import cv2
import numpy as np
import time

class MaskSubscriber(Node):
        def __init__(self):
            super().__init__('mask_subscriber')
            self.subscription = self.create_subscription(CompressedImage, 'mask', self.listener_callback, 10)
            self.rgbsub = self.create_subscription(CompressedImage, 'rgb', self.listener_callback2, 10)
    
        def listener_callback(self, msg):
            img = self.convert_image_message_to_cv(msg)
            cv2.imshow('mask', img)
            cv2.waitKey(1)

        def listener_callback2(self, msg):
            img = self.convert_image_message_to_cv(msg)

            # detect keyboard input and record image if 's' is pressed
            key = cv2.waitKey(1)
            if key == ord('e'):
                # use the time as the image name
                name = 'calibration/'+str(time.time()) + '.png'
                cv2.imwrite(name, img)
                print('Image saved')

            cv2.imshow('rgb', img)
            cv2.waitKey(1)
    
        def convert_image_message_to_cv(self, msg):
            # convert compressed image to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # print(image_np)
            # convert to mask
            # image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            return image_np
        
def main(args=None):
    rclpy.init(args=args)

    mask_sub = MaskSubscriber()

    while rclpy.ok():
        rclpy.spin_once(mask_sub)

    mask_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()