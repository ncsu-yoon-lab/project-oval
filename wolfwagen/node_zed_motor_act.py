#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import time
import math
from std_msgs.msg import Float64MultiArray

pid_steer = 0
manual_throttle = 0
manual_steer = 0
mode = 0

def manual_steering_callback(data):
    global manual_steer
    manual_steer = data.data

def manual_throttle_callback(data):
    global manual_throttle
    manual_throttle = data.data

def mode_switch_callback(data):
     global mode
     mode = (mode + 1) % 2

def zed_callback(msg):
	global pid_steer
      
	pid_steer = msg.data


def main(args=None):
    global manual_steer, manual_throttle, pid_steer
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    subscription_steering = node.create_subscription(Int64,'pid_steering', zed_callback, 1)
    subscription_manual_steering = node.create_subscription(Int64,'manual_steering', manual_steering_callback, 1)
    subscription_manual_throttle = node.create_subscription(Int64,'manual_throttle', manual_throttle_callback, 1)
    subscription_mode_switch = node.create_subscription(Int64, 'mode_switch', mode_switch_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        if (mode % 2 == 0):
             throttle = manual_throttle
             steer = manual_steer
        else:
             throttle = manual_throttle
             steer = pid_steer

        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            can_data = struct.pack('>hhI', throttle, steer, 0)
            msg = can.Message(arbitration_id=0x1,data=can_data, is_extended_id = False)
            bus.send(msg)
        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()