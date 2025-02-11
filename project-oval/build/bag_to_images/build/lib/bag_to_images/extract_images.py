import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageExtractor(Node):
    def __init__(self):
        super().__init__('image_extractor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.output_directory = '/home/wolfwagen/video_feed/'

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting image message: %s' % str(e))
            return
        
        filename = os.path.join(self.output_directory, f"image_{msg.header.stamp}.jpg")
        cv2.imwrite(filename, cv_image)
        self.get_logger().info('Saved image: %s' % filename)

def main(args=None):
    rclpy.init(args=args)
    image_extractor = ImageExtractor()
    rclpy.spin(image_extractor)
    image_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
