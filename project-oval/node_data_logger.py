#!/usr/bin/env python

# Import necessary ROS2 packages
import rclpy
from gps_msgs.msg import GPSFix
from geometry_msgs.msg import PoseStamped
from nmea_msgs.msg import Gpgga
from rclpy.node import Node

# Import other necessary packages
import threading
import time
import datetime
import csv
import curses
import os
import math
import joblib
import numpy as np

# Intialize the controller of the terminal page
stdscr = curses.initscr()

# Set the frequency of the ROS2 node
FREQ = 10

# Initialize the data being collected
swift_latitude = swift_longitude = swift_heading = zed_x = zed_y = zed_yaw = gps_latitude = gps_longitude = swift_horizontal_error = swift_vertical_error = swift_track_error = None

# Initialize the booleans of receiving the messages
swift_received = gps_received = zed_received = False

# Initialize the file where the data will be saved
FILE = '/home/wolfwagen/oval_ws/src/project-oval/log/position_data_logger.csv'

# Initialize the names of the fields of the data
FIELD_NAMES = ['timestamp', 'swift_latitude', 'swift_longitude', 'swift_heading', 'swift_horizontal_error', 'swift_vertical_error','swift_track_error','zed_x', 'zed_y', 'zed_yaw', 'gps_latitude', 'gps_longitude', 'ml_latitude', 'ml_longitude']

# Initialize the machine learning model
model = joblib.load('/home/wolfwagen/oval_ws/src/project-oval/ML/gps_adjuster.pkl')

def swift_callback(msg):
    """Callback for the swift messages"""
    
    # Global variables that are set by the callback message
    global swift_latitude, swift_longitude, swift_heading, swift_received, swift_horizontal_error, swift_vertical_error, swift_track_error

    # Set the data to be saved from the swift node from the message
    swift_latitude = msg.latitude
    swift_longitude = msg.longitude
    swift_heading = msg.track
    swift_horizontal_error = msg.err_horz
    swift_vertical_error = msg.err_vert
    swift_track_error = msg.err_track

    # Set the swift received to True
    swift_received = True

def gps_callback(data):
    """Callback for the gps messages"""

    # Global variables that are set by the callback messages
    global gps_latitude, gps_longitude, gps_received

    # Set the data to be saved from the gps node from the message
    lat = data.lat
    lon = data.lon

    # Checks that the lat and lon are not Nan

    if not np.isnan(lat) and not np.isnan(lon):
        # Converts the gps data from DDmm.mmmm to dd.dddd
        gps_latitude, gps_longitude = gps_converter(lat), gps_converter(lon)
        gps_longitude = gps_longitude * -1.0
        # Set the gps received to True
        gps_received = True

def zed_callback(data):
    """Callback for the zed messages"""

    # Global variables that are set by the callback messages
    global zed_x, zed_y, zed_yaw, zed_received

    # Set the positional data to be saved from the zed node from the message
    zed_x = data.pose.position.x
    zed_y = data.pose.position.y

    # Get the pose orientation in quaternion from the message
    q_x = data.pose.orientation.x
    q_y = data.pose.orientation.y
    q_z = data.pose.orientation.z
    q_w = data.pose.orientation.w

    # Calculate the yaw of the zed from the quaternion variables
    zed_yaw = math.degrees(math.atan2(2.0 * (q_y * q_z + q_w * q_x), q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z))

    # Sets the zed received to True
    zed_received = True

def gps_converter(point):
    """Converts the gps coordinates from DDmm.mmmm to dd.dddd so that it can be compared directly with swift GNSS readings"""

    # Gets the degrees from the point
    degrees = np.floor(point / 100)

    # Gets the minutes from the point
    minutes = point - degrees * 100

    # Returns the converted coordinate
    return degrees + minutes / 60

def gps_adjuster(lat, lon):
    """Uses the ML model to predict a more accurate GPS location"""
    
    # Sets the gps raw latitude and longitude data as an np array
    point = np.array([[lat, lon]])

    # Predicts the latitude and longitude from the model
    corrected_point = model.predict(point)

    # Returns the corrected latitude and longitude
    return corrected_point[0][0], corrected_point[0][1]

def log_data(data):
    """Logs the data from this node to be saved and analyzed after collecting the data"""

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
        FIELD_NAMES[12]: data[12],
        FIELD_NAMES[13]: data[13]
    }

    # Checks if the file exists already
    file_exists = os.path.isfile(FILE)

    # Opens the file
    with open(FILE, 'a', newline='') as csvfile:

        # Makes a writer for a dictionary and sets the field names
        writer = csv.DictWriter(csvfile, fieldnames=FIELD_NAMES)

        # Checks if the file does not exist and then writes a header for it
        if not file_exists:
            writer.writeheader()

        # Writes a row on the csv with the data
        writer.writerow(saved_data)

def main():
    """Main function to run after the script starts"""

    # Initializes ROS2
    rclpy.init()

    # Creates a logger node
    logger_node = rclpy.create_node('logger_node')

    # Subscribes to the swift GNSS, gps, and zed and they all go back to their callbacks
    swift_sub = logger_node.create_subscription(GPSFix, '/gpsfix', swift_callback, 1)
    gps_sub = logger_node.create_subscription(Gpgga, '/gps/gpgga', gps_callback, 1)
    zed_sub = logger_node.create_subscription(PoseStamped, '/zed/zed_node/pose', zed_callback, 1)

    # Threads the node so it spins and constantly receives new messages
    thread = threading.Thread(target=rclpy.spin, args=(logger_node,), daemon=True)
    thread.start()

    # Creates a rate for the node using the specified frequency
    rate = logger_node.create_rate(FREQ, logger_node.get_clock())

    # Time between logs is set in seconds
    time_between_logs = 2 # Wait 2 seconds between each log

    # Initialize the time since last logged
    time_last_logged = time.time()

    # While loop that runs while the ROS2 node is ok
    while rclpy.ok():
        
        # Checks if the swift, gps, and zed messages have all been received and that it has been more than time between logs to log again
        if swift_received and gps_received and zed_received and time.time() - time_last_logged > time_between_logs:
            
            # Reset time last logged
            time_last_logged = time.time()

            # Get the timestamp
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S')

            # Gets the output of the ml model for the latitude and longitude
            ml_latitude, ml_longitude = gps_adjuster(gps_latitude, gps_longitude)

            # Format the data to be saved
            data = [timestamp, swift_latitude, swift_longitude, swift_heading, swift_horizontal_error, swift_vertical_error, swift_track_error, zed_x, zed_y, zed_yaw, gps_latitude, gps_longitude, ml_latitude, ml_longitude]

            # Log the data
            log_data(data)

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'LOGGER NODE')

        stdscr.addstr(3, 5, 'Received Swift Data : %s		        ' % str(swift_received))
        stdscr.addstr(4, 5, 'Received GPS Data : %s		        ' % str(gps_received))
        stdscr.addstr(5, 5, 'Received ZED Data : %s		        ' % str(zed_received))

        # Sleeps at the created rate between updating messages
        rate.sleep()

    # Not necessary but it destroys the node and shuts down ROS2
    logger_node.destroy_node()
    rclpy.shutdown()

# Initiates main
if __name__ == '__main__':
    main()
