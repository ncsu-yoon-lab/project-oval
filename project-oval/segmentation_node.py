#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
from std_msgs.msg import Int64
from sidewalk_segmentation.sidewalk_segmentation import SegmentationModel


#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"

seg_model = SegmentationModel()

br = CvBridge()
image = None
zed_image = None

def camera_callback(data):
    global zed_image
    zed_image = br.imgmsg_to_cv2(data)
    zed_image = zed_image[:,:,:3]

def process_image(image):
    cv2.imshow("Segmented image", image)
    cv2.waitKey(1)

def main():

    rclpy.init()
    node = rclpy.create_node('segmentation_node')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        image = seg_model.segment_image(zed_image)


        process_image(image)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
