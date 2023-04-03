import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
import cv2 as cv
import numpy as np
# from matplotlib.pyplot import hsv
from cv_bridge import CvBridge
import threading
import time
import tensorflow as tf
keras = tf.keras

DATA_SAVE_PATH = "data/img_"
DATA_FILE_EXTENSION = ".png"

file_idx = 0
img = None
bridge = CvBridge()

def zed_callback(msg: Image = None) -> None :
    global img
    img = msg

def main(args=None) -> None :
    global img

    rclpy.init(args=args)
    node = Node("lane_image_dataset_generate_node")

    node.create_subscription(
        Image,
        '/zed2i/zed_node/rgb_raw/image_raw_color',
        zed_callback,
        20
    )

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(1, node.get_clock())

    while rclpy.ok() :
        cv.imwrite(DATA_SAVE_PATH + str(file_idx) + DATA_FILE_EXTENSION, bridge.imgmsg_to_cv2(img))
        file_idx += 1
        rate.sleep()
    
    rclpy.shutdown()


if __name__ == "__main__" :
    main()