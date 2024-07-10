#!/usr/bin/env python

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 as cv # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from gps_msgs.msg import GPSFix

import curses
import time
import datetime
import threading
import numpy as np
import os
import csv

stdscr = curses.initscr()

# Initializations

frame = None # frame is the OpenCV image
last_frame_time = time_since_last_saved = time.time() # Used to compare the times in between frames
br = CvBridge() # Used to convert ROS2 frames to OpenCV frames
received_frame = False

latitude = longitude = heading = spherical_err = horizontal_err = vertical_err = track_err = 0

def rtk_callback(data):
    
    global latitude, longitude, heading, spherical_err, horizontal_err, vertical_err, track_err

    latitude = data.latitude
    longitude = data.longitude
    heading = data.track
    spherical_err = data.err
    horizontal_err = data.err_horz
    vertical_err = data.err_vert
    track_err = data.err_track

def image_callback(msg):
    # Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
    global frame, frame_darker, frame_darkest, last_frame_time
    frame = br.imgmsg_to_cv2(msg)

    # Adjusts the gamma of the same frame

    frame_darker = adjust_gamma(frame, 0.5)

    frame_darkest = adjust_gamma(frame, 0.3)

    last_frame_time = time.time()

def adjust_gamma(image, gamma):

    # Adjusts the brightness of frame to make it easier or harder to see colors
    # Increasing gamma makes it darker, decreasing gamma makes it brighter
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                  for i in np.arange(0 , 256)]).astype("uint8")
    return cv.LUT(image , table)

# Logs the data to the given file
def log_data(data):

    CSV_FILE = '/home/wolfwagen/oval_ws/src/project-oval/log/data_logger.csv'

    FIELD_NAMES = ['timestamp', 'latitude', 'longitude', 'heading','img_name', 'darker_img_name', 'darkest_img_name', 'spherical_error', 'horizontal_error', 'vertical_error', 'track_error']

    # Saves the data as a dictionary
    saved_data = {
        FIELD_NAMES[0]: data[0],
        FIELD_NAMES[1]: data[1],
        FIELD_NAMES[2]: data[2],
        FIELD_NAMES[3]: data[3],
        FIELD_NAMES[4]: data[4],
        FIELD_NAMES[5]: data[5],
        FIELD_NAMES[6]: data[6],
        FIELD_NAMES[7]: data[7],
        FIELD_NAMES[8]: data[8],
        FIELD_NAMES[9]: data[9],
        FIELD_NAMES[10]: data[10]
    }

    # Checks if the file exists already
    file_exists = os.path.isfile(CSV_FILE)

    # Opens the file
    with open(CSV_FILE, 'a', newline='') as csvfile:

        # Makes a writer for a dictionary and sets the field names
        writer = csv.DictWriter(csvfile, fieldnames=FIELD_NAMES)

        # Checks if the file does not exist and then writes a header for it
        if not file_exists:
            writer.writeheader()

        # Writes a row on the csv with the data
        writer.writerow(saved_data)

def main(args = None):
    global frame, time_since_last_saved, received_frame, last_frame_time, latitude, longitude, heading, spherical_err, horizontal_err, vertical_err, track_err

    rclpy.init(args = args)
    node = Node("capture_images_node")
    zed_img_subscription = node.create_subscription(
        Image,
        '/zed/zed_node/left/image_rect_color',
        image_callback,
        5
    )

    rtk_coord_subscription = node.create_subscription(GPSFix, '/gpsfix', rtk_callback, 1)

    file_path = '/home/wolfwagen/oval_ws/src/project-oval/log/captured_images/'
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

                now = datetime.datetime.now()
                timer = now.strftime('%H:%M:%S')

                img_name = timer + '.jpg'
                img_darker_name = timer + '_darker.jpg'
                img_darkest_name = timer + '_darkest.jpg'

                # Saves the frame as a png in the specified file path with a changing name depending on the count
                cv.imwrite(file_path + img_name, frame)
                cv.imwrite(file_path + img_darker_name, frame_darker)
                cv.imwrite(file_path + img_darkest_name, frame_darkest)

                # Saves the data and the names of the images to a csv
                data = [timer, latitude, longitude, heading, img_name, img_darker_name, img_darkest_name, spherical_err, horizontal_err, vertical_err, track_err]
                log_data(data)

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