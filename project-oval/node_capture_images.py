#!/usr/bin/env python

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 as cv # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import curses
import time
import threading

stdscr = curses.initscr()

# Initializations
frame = None # frame is the OpenCV image
last_frame_time = time_since_last_saved = time.time() # Used to compare the times in between frames
br = CvBridge() # Used to convert ROS2 frames to OpenCV frames
received_frame = False

def image_callback(msg):
    # Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
    global frame, last_frame_time
    frame = br.imgmsg_to_cv2(msg)
    last_frame_time = time.time()
    
def main(args = None):
    global frame, time_since_last_saved, received_frame, last_frame_time

    rclpy.init(args = args)
    node = Node("capture_images_node")
    zed_img_subscription = node.create_subscription(
        Image,
        '/zed/zed_node/left/image_rect_color',
        image_callback,
        5
    )

    file_path = '/home/wolfwagen/oval_ws/src/project-oval/captured_images/'
    count = 0

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 20
    rate = node.create_rate(FREQ , node.get_clock())


    # ROS2 loop while ROS2 is still running and is ok
    while rclpy.ok():

        # Checking if there are frames (waiting for ZED node to start publishing)
        if frame is not None:

            # Set the received frames to true
            received_frame = True

            # If the time in-between frames is too long
            if time.time() - last_frame_time > 3:
                break
            
            # Checks if it has been enough time to save the next frame as png
            if time.time() - time_since_last_saved > 3:

                # Saves the frame as a png in the specified file path with a changing name depending on the count
                cv.imwrite(file_path + str(count) + '.jpg', frame)

                # Increase the count by one
                count += 1

                # Reset the time since the last image was saved
                time_since_last_saved = time.time()
            

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'CAPTURE IMAGES NODE')
        stdscr.addstr(3, 5, 'Capturing Images: %s                 ' % str(received_frame))

        rate.sleep()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    