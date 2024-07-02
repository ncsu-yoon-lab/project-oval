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

stdscr = curses.initscr()

throttle = latitude = longitude = heading = altitude = previous_error = 0

FREQ = 10

def rtk_callback(data):
    
    global throttle, latitude, longitude, altitude, heading
    
    throttle = velocity_PID(data.speed, FREQ)
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude
    heading = data.track


def velocity_PID(velocity, FREQ):
    global previous_error

    # K values of PID
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0
    dt = 1 / float(FREQ)
    integral = 0

    # We want the car to maintain a 0.5 m/s velocity
    set_point = 0.5
    error = set_point - velocity
    integral = integral + error * dt
    derivative = (error - previous_error) / dt
    throttle = Kp * error + Ki * integral + Kd * derivative
    previous_error = error

    return int(throttle)

def main():
    global throttle, latitude, longitude, altitude, heading

    converter = c2c()

    rclpy.init()
    rtk_node = rclpy.create_node('rtk_node')

    rtk_sub = rtk_node.create_subscription(GPSFix, '/gpsfix', rtk_callback, 1)
    speed_pub = rtk_node.create_publisher(Int64, 'speed_topic', 1)
    pose_pub = rtk_node.create_publisher(Float64MultiArray, 'pos_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(rtk_node,), daemon=True)
    thread.start()

    
    rate = rtk_node.create_rate(FREQ, rtk_node.get_clock())

    # Initialize data
    array = [0, 0, 0, 0]

    while rclpy.ok():
        
        if longitude != 0:
            # Publishes the throttle based on its current speed
            data = Int64()
            data.data = throttle
            speed_pub.publish(data)

            # Converts the latitude and longitude to x, y coordinates with origin at center of path between EB1 and EB3, y axis towards hunt (parallel to sidewalk from EB1 to FW), x axis towards EB3 (parallel to sidewalk from EB1 to EB3)
            point = converter.get_cartesian((latitude, longitude))

            # Converts the given heading to a yaw in degrees
            yaw = converter.heading_to_yaw(10.0)

            # Publishes the position and heading
            data = Float64MultiArray()
            array[0] = float(point[0])
            array[1] = point[1]
            array[2] = altitude
            array[3] = yaw
            data.data = array
            pose_pub.publish(data)

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'RTK NODE')

        stdscr.addstr(3, 5, 'Throttle : %.4f		        ' % float(throttle))
        stdscr.addstr(4, 5, 'Latitude : %.4f                  ' % float(latitude))
        stdscr.addstr(5, 5, 'Longitude : %.4f                  ' % float(longitude))
        stdscr.addstr(6, 5, 'Heading : %.4f                  ' % float(heading))

        stdscr.addstr(8, 5, 'X : %.4f                  ' % float(array[0]))
        stdscr.addstr(9, 5, 'Y : %.4f                  ' % float(array[1]))
        stdscr.addstr(10, 5, 'Yaw : %.4f                  ' % float(array[3]))


        rate.sleep()

    rtk_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
