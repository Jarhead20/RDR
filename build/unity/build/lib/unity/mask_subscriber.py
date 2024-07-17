import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
import cv2
import numpy as np

class MaskSubscriber(Node):
        def __init__(self):
            super().__init__('mask_subscriber')
            self.subscription = self.create_subscription(Image, 'mask', self.listener_callback, 10)
            self.subscription  # prevent unused variable warning
    
        def listener_callback(self, msg):
            img = self.convert_image_message_to_cv(msg)
            cv2.imshow('mask', img)
            cv2.waitKey(1)
    
        def convert_image_message_to_cv(self, msg):
            np_arr = np.frombuffer(msg.data, np.uint8)
            np_arr = np_arr.reshape(msg.height, msg.width, 3)
            # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return np_arr
        
def main(args=None):
    rclpy.init(args=args)

    mask_sub = MaskSubscriber()

    while rclpy.ok():
        rclpy.spin_once(mask_sub)

    mask_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()