#!/usr/bin/env python

# Imports necessary ROS2 packages
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 as cv # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from gps_msgs.msg import GPSFix # GPS message type

# Other necessary packages
import curses
import time
import datetime
import threading
import numpy as np
import os
import csv

# Initializes the terminal display
stdscr = curses.initscr()

# Initializes the frames and positional data
frame = frame_darker = frame_darkest = latitude = longitude = heading = spherical_err = horizontal_err = vertical_err = track_err = None # frame is the OpenCV image

# Initializes the last time a frame was captured
last_frame_time = time.time()

# Used to convert ROS2 frames to OpenCV frames
br = CvBridge() 

# Initializes the booleans to false
received_frame = received_swift = False

# Path to the directory where the images will be saved
DIR_PATH = '/home/wolfwagen/oval_ws/src/project-oval/log/captured_images/'

# CSV file where the data will be logged to
CSV_PATH = '/home/wolfwagen/oval_ws/src/project-oval/log/image_logger.csv'

# The names of the fields for the CSV
FIELD_NAMES = ['timestamp', 'latitude', 'longitude', 'heading','img_name', 'darker_img_name', 'darkest_img_name', 'spherical_error', 'horizontal_error', 'vertical_error', 'track_error']

# Frequency for the ROS2 node
FREQ = 20

def rtk_callback(msg):
    """Callback from the swift GNSS system node"""

    # Calls all of the global variables that will be changed from the message from the swift GNSS node
    global latitude, longitude, heading, spherical_err, horizontal_err, vertical_err, track_err, received_swift

    # Sets all of the necessary positional data from the message
    latitude = msg.latitude
    longitude = msg.longitude
    heading = msg.track
    spherical_err = msg.err
    horizontal_err = msg.err_horz
    vertical_err = msg.err_vert
    track_err = msg.err_track

    # Sets the received swift data to true
    received_swift = True

def image_callback(msg):
    """Callback from the image node from the zed"""

    # Calls all of the global variables that will be changed from the message from the zed node
    global frame, frame_darker, frame_darkest, last_frame_time, received_frame

    # Gets the frame by converting the message to opencv format using the bridge
    frame = br.imgmsg_to_cv2(msg)

    # Adjusts the gamma of the same frame to darker and darkest
    frame_darker = adjust_gamma(frame, 0.5)
    frame_darkest = adjust_gamma(frame, 0.3)

    # Sets a new last time a frame was taken
    last_frame_time = time.time()

    # Changes received frame to true
    received_frame = True

def adjust_gamma(image, gamma):
    """Adjusts the brightness of the image input into the function"""

    # Inverts the gamma input
    invGamma = 1.0 / gamma

    # Alters all pixels in the image by the inverted gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                  for i in np.arange(0 , 256)]).astype("uint8")
    
    # Returns the new image which is now altered by the gamma
    return cv.LUT(image , table)

def log_data(data):
    """Logs the data captured to a csv to be analyzed following its collection"""

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
    file_exists = os.path.isfile(CSV_PATH)

    # Opens the file
    with open(CSV_PATH, 'a', newline='') as csvfile:

        # Makes a writer for a dictionary and sets the field names
        writer = csv.DictWriter(csvfile, fieldnames=FIELD_NAMES)

        # Checks if the file does not exist and then writes a header for it
        if not file_exists:
            writer.writeheader()

        # Writes a row on the csv with the data
        writer.writerow(saved_data)

def main(args = None):
    """Main function to be ran after the script is started"""

    # Initializes ROS2
    rclpy.init(args = args)

    # Creates a ROS2 node
    node = Node("capture_images_node")

    # Creates a subscription for the images captured by the zed camera
    zed_img_subscription = node.create_subscription(Image, '/zed/zed_node/left/image_rect_color', image_callback, 5)

    # Creates a subscription for the gps coordinates captured by the swift GNSS system
    rtk_coord_subscription = node.create_subscription(GPSFix, '/gpsfix', rtk_callback, 1)

    # Threads the node so that it constantly spins and receives messages
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # Creates a rate for the node to spin at
    rate = node.create_rate(FREQ , node.get_clock())

    # Initialize the time since there was a save last
    time_since_last_saved = time.time()

    # Initializes saving images to false
    saving_images = False

    # ROS2 loop while ROS2 is still running and is ok
    while rclpy.ok():

        # Checking if there are frames (waiting for ZED node to start publishing)
        if frame is not None:

            # If the time in-between frames is too long
            if time.time() - last_frame_time > 3:
                break

            # Checks if it has been enough time to save the next frame as png and that there is a path to the directory and that GPS data is being received
            if time.time() - time_since_last_saved > 3 and os.path.isdir(DIR_PATH) and received_swift:
                
                # Sets the saving images to True
                saving_images = True

                # Gets the current timestamp
                timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S')

                # Saves the image with the timestamp in the name as reference
                img_name = timestamp + '.jpg'
                img_darker_name = timestamp + '_darker.jpg'
                img_darkest_name = timestamp + '_darkest.jpg'

                # Saves the frame as a png in the specified file path with a changing name depending on the timestamp
                cv.imwrite(DIR_PATH + img_name, frame)
                cv.imwrite(DIR_PATH + img_darker_name, frame_darker)
                cv.imwrite(DIR_PATH + img_darkest_name, frame_darkest)

                # Creates data that will be logged to the csv
                data = [timestamp, latitude, longitude, heading, img_name, img_darker_name, img_darkest_name, spherical_err, horizontal_err, vertical_err, track_err]
                
                # Logs the data to the csv
                log_data(data)

                # Reset the time since the last image was saved
                time_since_last_saved = time.time()
            

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'CAPTURE IMAGES NODE')

        stdscr.addstr(3, 5, 'Receiving Images: %s                 ' % str(received_frame))
        stdscr.addstr(4, 5, 'Receiving GPS: %s                    ' % str(received_swift))
        stdscr.addstr(5, 5, 'Saving Images: %s                    ' % str(saving_images))

        rate.sleep()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    