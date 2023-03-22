#!/usr/bin/env python
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

SHOW_IMAGES = False

PI = 3.1415926

bridge = CvBridge()

glob_raw_img = None

def zed_callback(msg: Image):
    global glob_raw_img
    glob_raw_img = msg

def main(args=None) -> None:

    rclpy.init(args=args)
    node = Node("stop_sign_detect_node")
    
    node.create_subscription(
        Image,
        '/zed2i/zed_node/rgb_raw/image_raw_color',
        zed_callback,
        20
    )

    publisher = node.create_publisher(
        Int64,
        "stop_sign",
        10
    )

    model = keras.models.load_model("./stop_sign_model")
    print("model loaded")
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5, node.get_clock())

    m = Int64()
    print("starting...")
    while rclpy.ok():

        loop_start_time = time.time()

        if glob_raw_img == None:
            print("no image")
            rate.sleep()
            continue

        # cache and convert the raw image to openCV format
        raw_img = bridge.imgmsg_to_cv2(glob_raw_img)
        if SHOW_IMAGES:
            cv.imshow("initial", raw_img)
            cv.waitKey(1)

        # apply transformations to make it easier to preprocess
        median_blur_img = cv.medianBlur(raw_img, 7)
        grayscale_img = cv.cvtColor(median_blur_img, cv.COLOR_BGR2GRAY)
        hsv_img = cv.cvtColor(median_blur_img, cv.COLOR_BGR2HSV)

        # obtain red color mask
        red_mask = cv.bitwise_or(
            cv.inRange(hsv_img, (0, 50, 100), (10, 255, 255)),
            cv.inRange(hsv_img, (160, 100, 100), (180, 255, 255))
        )

        # mask the raw image with the red mask to set all non-red values to zero
        masked_img = cv.cvtColor(cv.bitwise_and(median_blur_img, median_blur_img, mask=red_mask), cv.COLOR_BGR2GRAY)

        # grab circles from grayscale image
        circles = cv.HoughCircles(
            image=masked_img,
            method=cv.HOUGH_GRADIENT,
            dp=1,
            minDist=len(grayscale_img) // 5,
            param1=200,
            param2=20,
            minRadius=len(grayscale_img) // 20,
            maxRadius=len(grayscale_img) // 4
        )

        # if there are no circles, publish a value of zero
        if circles is None:
            print("none")
            m.data = 0
        else:

            if SHOW_IMAGES:
                cv.imshow("red", masked_img)
                cv.waitKey(1)

            # find the circle with the largest percent area of non-zero values
            # based on the red-masked image
            best_num = 0
            best = [1, 1, 1]
            empty_mask = np.zeros_like(masked_img)
            for circle in circles[0,:]:
                cv.circle(
                    img=empty_mask,
                    center=(int(circle[0]), int(circle[1])),
                    radius=int(circle[2]),
                    color=(255, 255, 255),
                    thickness=-1
                )
                double_masked_img = cv.bitwise_and(masked_img, empty_mask)
                
                percent_area = cv.countNonZero(double_masked_img) / (PI * (circle[2] ** 2))
                if percent_area > best_num:
                    best = circle
                    best_num = percent_area

            if SHOW_IMAGES:
                cv.imshow("circles", empty_mask)
                cv.waitKey(1)

            # crop the image to the detected potential stop sign
            img_y, img_x, chan = raw_img.shape
            buffer = best[2] * 1.5

            crop_x1 = max(int(best[0] - buffer), 0)
            crop_x2 = min(int(best[0] + buffer), img_x)
            crop_y1 = max(int(best[1] - buffer), 0)
            crop_y2 = min(int(best[1] + buffer), img_y)

            cropped_img = raw_img[crop_y1:crop_y2, crop_x1:crop_x2]

            if SHOW_IMAGES:
                cv.imshow("cropped", cropped_img)
                cv.waitKey(1)

            # resize the image to the size needed by the model
            resized_img = cv.resize(cropped_img, (128, 128))

            if SHOW_IMAGES:
                cv.imshow("resized", resized_img)
                cv.waitKey(1)

            # format with batch_size dimension
            input_img = np.array([cv.cvtColor(resized_img, cv.COLOR_BGR2RGB)])

            # get the model prediction
            classification = model(input_img).numpy().argmax()
        
            # determine if stop sign was classified
            if classification == 14:
                m.data = 1
            else:
                m.data = 0
        
        # publish
        # print(m)
        # publisher.publish(m)
        if m.data == 1:
            print("stop sign detected")
        else:
            print("no")
        
        publisher.publish(m)

        loop_end_time = time.time()
        print("loop_time", (loop_end_time-loop_start_time))


        # maintain rate
        rate.sleep()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
