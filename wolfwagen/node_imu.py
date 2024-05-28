import rclpy
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses

stdscr = curses.initscr()

linear_velocity = angular_velocity = 0

def imu_callback(data):
    global linear_velocity, angular_velocity
    print(type(data))
    print(data)

def get_velocity(linear_velocity, angular_velocity):
    

## Creating a Subscription
def main():
    global linear_velocity, angular_velocity

    rclpy.init()
    imu_node = rclpy.create_node('imu_node')

    imu_sub = imu_node.create_subscription(Imu, '/zed/zed_node/imu/data', imu_callback, 1)
    imu_pub = imu_node.create_publisher(Int64, 'imu_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(imu_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = imu_node.create_rate(FREQ, imu_node.get_clock())

    while rclpy.ok():

        velocity = get_velocity(linear_velocity, angular_velocity)

        speed = velocity_PID(velocity)

        data = Int64()

        data.data = speed

        imu_pub.publish(data)
        # Display of all the important messastdscr.refresh()
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'Linear Velocity :  %.4f		         ' % float(x_coord))
        stdscr.addstr(2 , 5 , 'Angular Velocity :  %.4f	                ' % float(y_coord))
        stdscr.addstr(3 , 5 , 'Velocity : %.4f		        ' % math.degrees(float((z_yaw))))
        stdscr.addstr(4 , 5 , '')

        rate.sleep()

    zed_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
