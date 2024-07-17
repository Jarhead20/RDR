import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class HSVFilterNode(Node):
    def __init__(self):
        super().__init__('hsv_filter_node')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/mask',
            10
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Apply HSV filter to the image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_bound = (0, 0, 0)  # TODO: Set your lower HSV threshold values
        upper_bound = (255, 255, 255)  # TODO: Set your upper HSV threshold values
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Convert OpenCV image back to ROS Image message
        mask_msg = self.cv_bridge.cv2_to_imgmsg(mask, 'mono8')

        # view the image

        # Publish the mask
        self.publisher.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    hsv_filter_node = HSVFilterNode()
    rclpy.spin(hsv_filter_node)
    hsv_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()