#!/usr/bin/env python3
"""
ROS2 Node to visualize lane detection images
Subscribes to the visualization topic and displays it with OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class LaneVisualizer(Node):
    def __init__(self):
        super().__init__('lane_visualizer')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscriber to the visualization topic
        self.create_subscription(
            Image,
            '/lane_detection/visualization',
            self.image_callback,
            10
        )

        self.get_logger().info("Lane Visualizer Node started, listening on /lane_detection/visualization")

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Show the image
            cv2.imshow("Lane Detection Visualization", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LaneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()