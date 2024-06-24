import sys
import math
import rclpy
from rclpy.node import Node
from pyvicon_datastream import tools
from std_msgs.msg import *
import math
import threading
import curses
import os

VICON_TRACKER_IP = "eb2-2235-win00.csc.ncsu.edu"
OBJECT_NAME = "OVAL"

#This will try to connect to the VICON TRACKER
vicontracker = tools.ObjectTracker(VICON_TRACKER_IP)
exception = "None"


def get_position():
    global exception

    raw_data = vicontracker.get_position(OBJECT_NAME)
    if raw_data is False:
        exception = f"Cannot find object {OBJECT_NAME}"
        return

    _, _, pos_data = raw_data

    if pos_data != []:
        xyz = pos_data[0][2:5]
        orientation = pos_data[0][7]

        return xyz, math.degrees(orientation)
    else:
        exception = "No Vicon Data!"


def main(args=None):
    global exception

    if not vicontracker.is_connected:
        sys.exit(0)

    rclpy.init(args=args)
    node = Node("vicon_pos")
    vicon_publisher = node.create_publisher(Float64MultiArray, 'pos_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())

    data = [0.0] * 4

    data1 = Float64MultiArray()

    os.system('clear')

    stdscr = curses.initscr()

    while rclpy.ok():
        try:
            xyz, yaw = get_position()

            data1.data = [float(xyz[0] / 1000), float(xyz[1] / 1000), float(xyz[2] / 1000), float(yaw)]

            data[0] = float(xyz[0] / 1000)
            data[1] = float(xyz[1] / 1000)
            data[2] = float(xyz[2] / 1000)
            data[3] = float(yaw)
            try:
                vicon_publisher.publish(data1)
                exception = "None"
            except Exception as e:
                exception = repr(e)
        except TypeError as e:
            exception = "Car not being picked up by camera"

        stdscr.refresh()
        stdscr.addstr(1, 5, 'VICON NODE')
        if (data[0] != 0.0):
            stdscr.addstr(3, 5, 'Current X (m) :  %.4f	                ' % data[0])
            stdscr.addstr(4, 5, 'Current Y (m) :  %.4f		         ' % data[1])
            stdscr.addstr(5, 5, 'Current Z (m) :  %.4f	                ' % data[2])
            stdscr.addstr(6, 5, 'YAW (Degrees) :  %.4f		         ' % data[3])
        stdscr.addstr(8, 5, 'Exceptions :  %s	                ' % str(exception))

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
