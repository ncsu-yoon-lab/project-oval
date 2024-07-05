import rclpy
from std_msgs.msg import Int64, Float64MultiArray
from gps_msgs.msg import GPSFix
from lib.coords_to_cartesian import CoordsToCartesian as c2c
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses
from geographiclib.geodesic import Geodesic

stdscr = curses.initscr()

throttle = latitude = longitude = heading = altitude = previous_error = count = 0

FREQ = 10

waypoints = []

previous_coord = None

last_time = time.time()

def server_callback(data):
    global latitude, longitude, last_time, count, waypoints
    
    # Checks if new waypoints have been sent, if so change the waypoints
    if waypoints != data.data:
        waypoints = data.data
        count = 0
    
    # Every 3 seconds move to the next point on the waypoints and get the new latitude longitude
    if time.time() - last_time > 3:
        latitude = waypoints[count]
        longitude = waypoints[count + 1]
        count += 2
        last_time = time.time()

def main():
    global throttle, latitude, longitude, altitude, heading, previous_coord

    converter = c2c()

    rclpy.init()
    rtk_sim_node = rclpy.create_node('rtk_sim_node')

    server_sub = rtk_sim_node.create_subscription(Float64MultiArray, 'waypoints_coord_topic', server_callback, 1)
    speed_pub = rtk_sim_node.create_publisher(Int64, 'speed_topic', 1)
    pos_pub = rtk_sim_node.create_publisher(Float64MultiArray, 'pos_topic', 1)
    coord_pub = rtk_sim_node.create_publisher(Float64MultiArray, 'coord_topic', 1)
    log_pub = rtk_sim_node.create_publisher(Float64MultiArray, 'rtk_logging_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(rtk_sim_node,), daemon=True)
    thread.start()

    
    rate = rtk_sim_node.create_rate(FREQ, rtk_sim_node.get_clock())

    while rclpy.ok():

        # Publishes the throttle based on its current speed
        data = Int64()
        data.data = throttle
        speed_pub.publish(data)

        # Publishes the latitude and longitude from the rtk
        data = Float64MultiArray()
        data.data.append(latitude)
        data.data.append(longitude)
        coord_pub.publish(data)

        # Converts the latitude and longitude to x, y coordinates with origin at center of path between EB1 and EB3, y axis towards hunt (parallel to sidewalk from EB1 to FW), x axis towards EB3 (parallel to sidewalk from EB1 to EB3)
        point = converter.get_cartesian((latitude, longitude))

        # Checks if the previous point is already known
        if previous_coord:
            if previous_coord != (latitude, longitude):
                heading = Geodesic.WGS84.Inverse(previous_coord[0], previous_coord[1], latitude, longitude).get('azi1')
            else:
                pass
        else:
            heading = 0.0

        yaw = converter.heading_to_yaw(heading)

        # Publishes the position and yaw
        data = Float64MultiArray()
        data.data.append(point[0])
        data.data.append(point[1])
        data.data.append(altitude)
        data.data.append(yaw)
        pos_pub.publish(data)

        # Sets the previous point as the current point
        previous_coord = (latitude, longitude)

        # Publishes the data of the RTK Node to be logged
        data = Float64MultiArray()
        data.data = [float(latitude), float(longitude), float(heading)]
        log_pub.publish(data)

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'RTK SIM NODE')

        stdscr.addstr(3, 5, 'Throttle : %.4f		        ' % float(throttle))
        stdscr.addstr(4, 5, 'Latitude : %.4f                  ' % float(latitude))
        stdscr.addstr(5, 5, 'Longitude : %.4f                  ' % float(longitude))
        stdscr.addstr(6, 5, 'Heading : %.4f                  ' % float(heading))
        stdscr.addstr(7, 5, 'Yaw : %.4f                  ' % float(yaw))

        rate.sleep()

    rtk_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
