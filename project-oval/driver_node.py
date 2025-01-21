#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import time

throttle = 0
steer = 0

def manual_steering_callback(data):
    global steer
    steer = data.data

def manual_throttle_callback(data):
    global throttle
    throttle = data.data

def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    subscription_manual_steering = node.create_subscription(Int64,'manual_steering', manual_steering_callback, 10)
    subscription_manual_throttle = node.create_subscription(Int64,'manual_throttle', manual_throttle_callback, 10)

    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():
        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            can_data = struct.pack('>hhI', throttle, steer, 0)
            msg = can.Message(arbitration_id=0x1,data=can_data, is_extended_id = False)
            ret = bus.send(msg)
        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()