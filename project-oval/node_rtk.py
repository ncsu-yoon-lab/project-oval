import rclpy
from std_msgs.msg import Int64
from sensor_msgs.msg import GPSFix
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses

stdscr = curses.initscr()

throttle = 0

def speed_callback(data):
    global throttle
    
    pass


def gps_callback(data):
    pass


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


def calibrate_speed():
    global x_offset, y_offset, z_offset, a_x, a_y, a_z, calibrated

    # Check that accelerations in x y and z are first received
    if a_x is not None and a_y is not None and a_z is not None:
        x_offset = a_x
        y_offset = a_y
        z_offset = a_z
        calibrated = True
        return True
    else:
        return False



def main():

    rclpy.init()
    gps_node = rclpy.create_node('gps_node')

    gps_sub = gps_node.create_subscription(speed, '/zed/zed_node/speed/data', speed_callback, 1)
    speed_pub = speed_node.create_publisher(Int64, 'speed_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(speed_node,), daemon=True)
    thread.start()

    FREQ = 10
    rate = speed_node.create_rate(FREQ, speed_node.get_clock())

    # Waits until the speed zeros the accelerometer
    while not calibrate_speed():
        continue

    while rclpy.ok():

        if calibrated:
            velocity = get_velocity(linear_acceleration)
        else:
            velocity = 0.0

        throttle = velocity_PID(velocity, FREQ)

        data = Int64()

        data.data = throttle

        speed_pub.publish(data)

        # Display of all the important messages
        stdscr.refresh()
        stdscr.addstr(1, 5, 'speed NODE')
        stdscr.addstr(3, 5, 'Acceleration : %.4f		        ' % float(linear_acceleration))
        stdscr.addstr(4, 5, 'Velocity : %.4f                  ' % float(velocity))
        stdscr.addstr(5, 5, 'Throttle : %d                  ' % throttle)

        rate.sleep()

    speed_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
