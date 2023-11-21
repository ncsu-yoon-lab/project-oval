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

throttle = 0
steer = 0

def pure_pursuit_callback(msg):
	global throttle, steer
      
	goal_threshold = 4

	distance_to_goal = msg.data[0]
	current_x = msg.data[1]
	current_y = msg.data[2]
	alpha = math.degrees(msg.data[3])
	path_curvature = msg.data[4]
	steering_angle = msg.data[5]
      



      
	steer = int(steering_angle)
	throttle = 15

	print(f"Distance to goal: {distance_to_goal}")
	print(f"Current x_pos: {current_x}\nCurrent y_pos: {current_y}")
	print(f"Alpha: {alpha}")
	print(f"Path curvature: {path_curvature}")
	print(f"Steering: {steering_angle}")

	if distance_to_goal <= goal_threshold:
		throttle = 0
		steer = 0
		print("goal reached")   #This is probably in the wrong location but the location but the logic should be super straightforward

def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    subscription_steering = node.create_subscription(Float64MultiArray,'pure_pursuit', pure_pursuit_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
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