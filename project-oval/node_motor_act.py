#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import curses
from std_msgs.msg import Int64MultiArray

pp_throttle = pp_steer = manual_throttle = manual_steer = mode = 0

steer_offset = 5

exception = "None"

stdscr = curses.initscr()


def manual_steering_callback(data):
    global manual_steer

    manual_steer = data.data + steer_offset


def manual_throttle_callback(data):
    global manual_throttle
    manual_throttle = data.data


def mode_switch_callback(data):
    global mode
    mode = (mode + 1) % 2


def pure_pursuit_callback(data):
    global pp_throttle, pp_steer

    pp_throttle = data.data[0]
    pp_steer = data.data[1] + steer_offset


def main(args=None):
    global manual_steer, manual_throttle, pp_steer, pp_throttle, exception
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)

    rclpy.init(args=args)
    node = Node("driver")

    sub_to_pure_pursuit = node.create_subscription(Int64MultiArray, 'pure_pursuit_motor_topic', pure_pursuit_callback,
                                                   1)
    sub_manual_steering = node.create_subscription(Int64, 'manual_steering', manual_steering_callback, 1)
    sub_manual_throttle = node.create_subscription(Int64, 'manual_throttle', manual_throttle_callback, 1)
    sub_mode_switch = node.create_subscription(Int64, 'mode_switch', mode_switch_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        if mode % 2 == 0:
            throttle = manual_throttle
            steer = manual_steer
            str_mode = "Manual"
        else:
            throttle = manual_throttle
            steer = pp_steer
            str_mode = "Pure Pursuit"

        try:
            can_data = struct.pack('>hhI', throttle, steer, 0)
            msg = can.Message(arbitration_id=0x1, data=can_data, is_extended_id=False)
            bus.send(msg)
        except Exception as error:
            exception = error
        finally:

            stdscr.refresh()
            stdscr.addstr(1, 5, 'MOTOR ACTUATION NODE')
            stdscr.addstr(3, 5, 'Throttle :  %s                 ' % str(throttle))
            stdscr.addstr(4, 5, 'Steer :  %s                    ' % str(steer))
            stdscr.addstr(5, 5, 'Mode :  %s                ' % str_mode)
            stdscr.addstr(7, 5,
                          'Exceptions :  %s                                                                               ' % exception)

            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
