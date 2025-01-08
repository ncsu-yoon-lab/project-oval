#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
import struct
import can
import threading
import time

throttle = 0
steer = 0
manual_mode = True

def manual_mode_callback(data):
    global manual_mode
    manual_mode = data.data

def manual_throttle_callback(data):
    if(manual_mode):
        global throttle
        throttle = data.data

def manual_steering_callback(data):
    if(manual_mode):
        global steer
        steer = data.data

def cv_throttle_callback(data):
    if(not manual_mode):
        global throttle
        throttle = data.data

def cv_steering_callback(data):
    if(not manual_mode):
        global steer
        steer = data.data

def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    manual_mode_sub = node.create_subscription(Bool, "manual_mode", manual_mode_callback, 10)
    manual_throttle_sub = node.create_subscription(Int64, "manual_throttle", manual_throttle_callback, 10)
    manual_steering_sub = node.create_subscription(Int64, "manual_steer", manual_steering_callback, 10)
    cv_steering_sub = node.create_subscription(Int64, "cv_steer", cv_steering_callback, 10)
    cv_throttle_sub = node.create_subscription(Int64, "cv_throttle", cv_throttle_callback, 10)

    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            # DO NOT COMPUTE THE PWM VALUES IN ORIN. Just send the raw command values. 
            can_data = struct.pack('>hhI', throttle, steer, 0)
            can_msg = can.Message(arbitration_id=0x1, data=can_data, is_extended_id = False)
            ret = bus.send(can_msg)

        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()