#!/usr/bin/env python

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from gps_msgs.msg import GPSFix
from geometry_msgs.msg import PoseStamped, Pose, Point

import curses
import time
import datetime
import threading
import numpy as np
import os
import csv
import math

stdscr = curses.initscr()

# Initializations

time_since_last_saved = time.time() # Used to compare the times in between frames
received_data = False

latitude = longitude = altitude = heading = spherical_err = horizontal_err = vertical_err = track_err = zed_x = zed_y = zed_z = zed_yaw = 0

def rtk_callback(data):
    
    global latitude, longitude, altitude, heading, spherical_err, horizontal_err, vertical_err, track_err

    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    heading = data.track
    spherical_err = data.err
    horizontal_err = data.err_horz
    vertical_err = data.err_vert
    track_err = data.err_track

# Logs the data to the given file
def log_data(data):

    CSV_FILE = '/home/wolfwagen/oval_ws/src/project-oval/log/data_zed_logger.csv'

    FIELD_NAMES = ['timestamp', 'latitude', 'longitude', 'altitude', 'heading','spherical_error', 'horizontal_error', 'vertical_error', 'track_error', 'zed_x', 'zed_y', 'zed_z', 'zed_yaw']

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
        FIELD_NAMES[10]: data[10],
        FIELD_NAMES[11]: data[11],
        FIELD_NAMES[12]: data[12]
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

def zed_callback(msg):
    global zed_x, zed_y, zed_z, zed_yaw, received_data

    received_data = True

    # Euler position
    zed_x = msg.pose.position.x
    zed_y = msg.pose.position.y
    zed_z = msg.pose.position.z

    # Quaternion position to get the correct yaw
    q_x = msg.pose.orientation.x
    q_y = msg.pose.orientation.y
    q_z = msg.pose.orientation.z
    q_w = msg.pose.orientation.w

    # Get the yaw from the quaternion position
    zed_yaw = math.degrees(math.atan2(2.0 * (q_y * q_z + q_w * q_x), q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z))
    



def main(args = None):
    
    global time_since_last_saved, received_data, latitude, longitude, altitude, heading, spherical_err, horizontal_err, vertical_err, track_err, zed_x, zed_y, zed_z, zed_yaw

    rclpy.init(args = args)
    node = Node("capture_gps_node")
    zed_subscription = node.create_subscription(
        PoseStamped,
        '/zed/zed_node/pose',
        zed_callback,
        5
    )
    rtk_sub = node.create_subscription(GPSFix, '/gpsfix', rtk_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 20
    rate = node.create_rate(FREQ , node.get_clock())

    # ROS2 loop while ROS2 is still running and is ok
    while rclpy.ok():
        
        # Checks that data is being received from the GPS
        if received_data:

            # Checks if it has been enough time to save the next frame as png
            if time.time() - time_since_last_saved > 3:

                now = datetime.datetime.now()
                timer = now.strftime('%H:%M:%S')

                # Saves the data and the names of the images to a csv
                data = [timer, latitude, longitude, altitude, heading, spherical_err, horizontal_err, vertical_err, track_err, zed_x, zed_y, zed_z, zed_yaw]
                log_data(data)

                # Reset the time since the last image was saved
                time_since_last_saved = time.time()
            

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'GPS (CHEAP) NODE')
        stdscr.addstr(3, 5, 'Logging Data: %s                 ' % str(received_data))

        rate.sleep()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    