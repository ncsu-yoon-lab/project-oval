import rclpy
from std_msgs.msg import Float64MultiArray
import threading
from rclpy.node import Node
import time
import csv
import curses
import os

stdscr = curses.initscr()

FREQ = 10

latitude = longitude = heading = yaw = current_x = current_y = x_goal = y_goal = lookahead_x = lookahead_y = None

rtk_called = pure_pursuit_called = False

FILE = '/home/wolfwagen/oval_ws/src/project-oval/log/data_logger.csv'

FIELD_NAMES = ['timestamp', 'latitude', 'longitude', 'heading', 'current_x', 'current_y', 'x_goal', 'y_goal', 'lookahead_x', 'lookahead_y', 'yaw']

def rtk_callback(data):
    
    global latitude, longitude, heading, rtk_called

    latitude = data.data[0]
    longitude = data.data[1]
    heading = data.data[2]

    rtk_called = True


def pure_pursuit_callback(data):

    global current_x, current_y, yaw, x_goal, y_goal, lookahead_x, lookahead_y, pure_pursuit_called

    current_x = data.data[0]
    current_y = data.data[1]
    yaw = data.data[2]
    x_goal = data.data[3]
    y_goal = data.data[4]
    lookahead_x = data.data[5]
    lookahead_y = data.data[6]

    pure_pursuit_called = True

def log_data(data):

    # Saves the data as a dictionary
    saved_data = {
        FIELD_NAMES[0]: time.time(),
        FIELD_NAMES[1]: data[0],
        FIELD_NAMES[2]: data[1],
        FIELD_NAMES[3]: data[2],
        FIELD_NAMES[4]: data[3],
        FIELD_NAMES[5]: data[4],
        FIELD_NAMES[6]: data[5],
        FIELD_NAMES[7]: data[6],
        FIELD_NAMES[8]: data[7],
        FIELD_NAMES[9]: data[8],
        FIELD_NAMES[10]: data[9],
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
    global current_x, current_y, yaw, x_goal, y_goal, lookahead_x, lookahead_y, pure_pursuit_called, latitude, longitude, heading, rtk_called

    rclpy.init()
    logger_node = rclpy.create_node('logger_node')

    rtk_sub = logger_node.create_subscription(Float64MultiArray, 'rtk_logging_topic', rtk_callback, 1)
    pure_pursuit_sub = logger_node.create_subscription(Float64MultiArray, 'pure_pursuit_logging_topic', pure_pursuit_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(logger_node,), daemon=True)
    thread.start()

    
    rate = logger_node.create_rate(FREQ, logger_node.get_clock())

    is_logging = False

    while rclpy.ok():
        
        # If both callbacks have been reached then it can start logging
        if rtk_called and pure_pursuit_called:
            is_logging = True
            data = [latitude, longitude, heading, current_x, current_y, x_goal, y_goal, lookahead_x, lookahead_y, yaw]
            log_data(data)

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'LOGGER NODE')

        stdscr.addstr(3, 5, 'Is Logging : %s		        ' % str(is_logging))

        rate.sleep()

    rtk_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
