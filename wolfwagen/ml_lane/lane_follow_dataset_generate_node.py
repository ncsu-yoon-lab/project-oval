import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
import cv2 as cv
import numpy as np
import pandas as pd
from cv_bridge import CvBridge
import threading
import time
import tensorflow as tf
keras = tf.keras

DATA_SAVE_PATH = "lanefollow/data/"
DATA_NAME_PRE = "img_"
DATA_FILE_EXTENSION = ".png"

steering = None
file_idx = 0
img = None
bridge = CvBridge()

def save_dataset(dataset: pd.DataFrame):
    dataset.to_csv(DATA_SAVE_PATH + "meta.csv")

def zed_callback(msg: Image = None) -> None :
    global img
    img = msg

def steering_callback(msg: Int64 = None) -> None:
    global steering
    steering = msg

def main(args=None) -> None :
    global img, steering

    rclpy.init(args=args)
    node = Node("lane_image_dataset_generate_node")

    node.create_subscription(
        Image,
        '/zed2i/zed_node/rgb_raw/image_raw_color',
        zed_callback,
        1
    )

    node.create_subscription(
        Int64,
        'pid_steering',
        steering_callback,
        1
    )

    dataset: pd.DataFrame = pd.DataFrame(columns=["img", "steering"])

    try:
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        rate = node.create_rate(1, node.get_clock())

        while rclpy.ok() :
            fname = DATA_NAME_PRE + str(file_idx) + DATA_FILE_EXTENSION
            cv.imwrite(DATA_SAVE_PATH + fname, bridge.imgmsg_to_cv2(img))
            row = pd.Series({"img": fname, "steering": steering})
            dataset = pd.concat([dataset, row.to_frame().T], ignore_index=True)
            file_idx += 1
            rate.sleep()
    
    except KeyboardInterrupt:
        pass

    finally:
        save_dataset(dataset)
    
    rclpy.shutdown()


if __name__ == "__main__" :
    main()