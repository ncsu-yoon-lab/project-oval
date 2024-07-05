import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

import threading
import curses
import requests
import csv

from requests.exceptions import RequestException
from lib.coords_to_cartesian import CoordsToCartesian as c2c

stdscr = curses.initscr()

latitude = longitude = 0.0

waypoints = []

is_connected = False

def rtk_callback(data):
    global latitude, longitude

    latitude = data.data[0]
    longitude = data.data[1]

def server_send(current_pos):

    try:
        # Path to the CSV file
        file_path = '/home/wolfwagen/oval_ws/src/project-oval/Server/pos.csv'

        # Open the CSV file in write mode to clear its contents and write new data
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(current_pos)

        url = 'http://3.16.149.178/upload'
        files = {'file': open(file_path, 'rb')}
        response = requests.post(url, files=files)
    except RequestException:
        pass

# Calls to get the most recent data from the server
def server_receive():
    global waypoints, is_connected

    # Tries to connect to the server if it is available
    try:
        url = 'http://3.16.149.178/download/waypoints.csv'

        # Gets the response from the server
        response = requests.get(url)

        # Decode the byte string
        csv_content = response.content.decode('utf-8')

        # Split the content by lines
        lines = csv_content.splitlines()

        # Parse the lines into a 2D array
        waypoints = [line.split(',') for line in lines]

        # Sets the connection to true
        is_connected = True

    except RequestException:
        is_connected = False

def main():
    global latitude, longitude, is_connected, waypoints

    rclpy.init()
    server_node = rclpy.create_node('server_node')

    server_sub = server_node.create_subscription(Float64MultiArray, 'coord_topic', rtk_callback, 1)
    server_pub = server_node.create_publisher(Float64MultiArray, 'waypoints_topic', 1)
    server_coord_pub = server_node.create_publisher(Float64MultiArray, 'waypoints_coord_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(server_node,), daemon=True)
    thread.start()

    FREQ = 10
    rate = server_node.create_rate(FREQ, server_node.get_clock())
    
    converter = c2c()

    while rclpy.ok():

        # Read the most recent waypoints from the server
        server_receive()

        # Initialize the data as a Float64MultiArray
        data = Float64MultiArray()

        # Goes through all the points in the waypoints and adds each latitude and longitude to the Float64MultiArray
        for point in waypoints:

            # Tries to convert string to float, if caught it will not append that point
            try:

                # Converts the point into an x y coordinate
                converted_point = converter.get_cartesian((float(point[0]), float(point[1])))
                for coord in converted_point:
                    data.data.append(coord)
                    
            except ValueError:
                pass

        # Publish the waypoints as x y
        server_pub.publish(data)

        # Publish the waypoints as lat long
        data = Float64MultiArray()
        for point in waypoints:
            for coord in point:
                data.data.append(float(coord))
        server_coord_pub.publish(data)

        # Send the current position to the server if it is connected
        if is_connected:
            server_send([(latitude, longitude)])

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'SERVER NODE')

        stdscr.addstr(3, 5, 'Connected to server : %s		        ' % is_connected)

        stdscr.addstr(5, 5, 'Current Latitude : %.4f                  ' % float(latitude))
        stdscr.addstr(6, 5, 'Current Longitude : %.4f                  ' % float(longitude))

        stdscr.addstr(8, 5, 'Waypoints : %s          ' % str(waypoints))

        rate.sleep()

    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
