import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import serial
import curses
x_coord = y_coord = z_yaw = 0

stdscr = curses.initscr()

def zed_callback(msg):
    global x_coord, y_coord , z_yaw

    x_coord = msg.pose.position.x
    y_coord = msg.pose.position.y

    x = msg.pose.orientation.x
    y = msg.pose.orientation.y
    z = msg.pose.orientation.z
    w = msg.pose.orientation.w
    
    t0 = 2.0 * (w * x * y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    z_yaw = math.atan2(t3, t4)

## Creating a Subscription
def main():
    global x_coord, y_coord, z_yaw

    rclpy.init()
    zed_node = rclpy.create_node('zed_node')

    zed_sub = zed_node.create_subscription(PoseStamped, '/zed/zed_node/pose', zed_callback, 1)
    pos_pub = zed_node.create_publisher(Float64MultiArray, 'zed_pos', 1)

    thread = threading.Thread(target=rclpy.spin, args=(zed_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = zed_node.create_rate(FREQ, zed_node.get_clock())

    while rclpy.ok():

        data = Float64MultiArray()

        data.data = [float(x_coord), float(y_coord), float(z_yaw)]

        pos_pub.publish(data)
        # Display of all the important messastdscr.refresh()
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'X :  %.4f		         ' % float(x_coord))
        stdscr.addstr(2 , 5 , 'Y :  %.4f	                ' % float(y_coord))
        stdscr.addstr(3 , 5 , 'Yaw: %.4f		        ' % math.degrees(float((z_yaw))))

        rate.sleep()

    zed_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
