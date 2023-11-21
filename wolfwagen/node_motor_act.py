#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_msgs.msg import Float64, Float64MultiArray
import struct
import can
import os
import threading
import time
import curses
import math
import numpy as np

#distributed ros doesn't work now, so let's use mqtt for now
#import paho.mqtt.client as paho
#broker_ip="eb2-3254-ub01.csc.ncsu.edu"
#broker_port=12345

stdscr = curses.initscr()

#TODO: pwm should be moved to Teensy microcontroller
in_min = -100
in_max = +100

max_throttle = 28
max_steer = 100
throttle = 0
steer = 0
mode = 0

pid_steer = 0
auto_throttle = 0	#published (and controller) by xbox_controller	

lidar_min_dist = 1000000	#for LIDAR-based obstacle detection/avoidance
SAFE_DISTANCE = 0.50

goal_threshold = 0.25
distance_to_goal = 0
current_x = 0
current_y = 0
alpha = 0
path_curvature = 0
steering_angle = 0
waypoints = []

# Measured in meters
wheel_base = 0.208
lookahead = 1.00 #not sure what to set this number to


def pwm(val):
	#TODO: input range check
	out_min = -100
	out_max = 100
	return (val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

# def pwm2(val):
# 	out_min = 0
# 	out_max = 180
# 	return (val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def mode_switch_callback(data):
	global mode
	mode = (mode + 1) % 2	#for now we have only two modes: manual and CV-based auto
		
def manual_steering_callback(data):
	global steer
	steer = data.data

def manual_throttle_callback(data):
	global throttle
	throttle = data.data

def auto_throttle_callback(data):
	global auto_throttle
	auto_throttle = data.data

def pid_steering_callback(data):
	global pid_steer

	steer = data.data
	
	if steer > 100:
		steer = 100
	elif steer < -100:
		steer = -100
	if (steer==100):
			steer = 99
	
	pid_steer = steer

def lidar_min_dist_callback(data):
	global lidar_min_dist
	lidar_min_dist = data.data	

#stop sign-related
stop_sign_detected = False
last_stop_time = time.time()
def stop_sign_callback(data):
	global stop_sign_detected, last_stop_time
	
	if time.time() > (last_stop_time + 5):
		# if it hasn't been 5 seconds since we detected a stop sign, ignore this message (-->to handle "stop")
		if data.data == 1:
			stop_sign_detected = True
			last_stop_time = time.time()
			# print("stop sign detected")

#ROS-based voice command handler
def voice_cmd_callback(data):
    global mode, throttle
    cmd = data.data
    # print('voice_cmd =', cmd)
    if cmd == 'stop':
        throttle = 0
        mode = 0
    elif cmd == 'start':
        mode = 1
    
    #elif cmd == 'left':
    #    print('left -- todo')
    #elif cmd == 'right':
    #    print('right -- todo')

#MQTT-based voice command handler
def on_voice_cmd_mqtt_message(client, userdata, message):
	global mode, throttle
	cmd = str(message.payload.decode("utf-8"))
	# print("voice_cmd_received =", cmd )
	if cmd == 'stop':
		throttle = 0
		mode = 0
	elif cmd == 'start':
		mode = 1
	elif cmd == 'left':
		print('left -- todo')
	elif cmd == 'right':
		print('right -- todo')

def pure_pursuit_callback(msg):
	global goal_threshold, distance_to_goal, current_x, current_y, alpha, path_curvature, steering_angle, auto_throttle, pid_steer
	distance_to_goal = msg.data[0]
	current_x = msg.data[1]
	current_y = msg.data[2]
	alpha = math.degrees(msg.data[3])
	path_curvature = msg.data[4]
	steering_angle = msg.data[5]

	print(f"Distance to goal: {distance_to_goal}")
	print(f"Current x_pos: {current_x}\nCurrent y_pos: {current_y}")
	print(f"Alpha: {alpha}")
	print(f"Path curvature: {path_curvature}")
	print(f"Steering: {steering_angle}")

	if distance_to_goal <= goal_threshold:
		auto_throttle = 0
		pid_steer = 0
		print("goal reached")   #This is probably in the wrong location but the location but the logic should be super straightforward


def find_nearest_waypoint():
	"""
	get closest index to the car
	"""
	global current_x, current_y, waypoints, wheel_base
	curr_xy = np.array([current_x, current_y])
	waypoints_xy = waypoints[:, :2]
	nearest_idx = np.argmin(np.sum((curr_xy - waypoints_xy)**2, axis=1))
	return nearest_idx


def idx_closest_to_lookahead(idx):
	"""
	Get closest index to lookahead that is greater than the lookahead
	"""
	global lookahead
	while distance_to_goal < lookahead:
		idx += 1
	return idx - 1

# def pure_pursuit():



def main(args=None):
	warning = ""
	can_message = ""
	print("Driver node")
	rclpy.init(args=args)
	node = Node("Drive_node")

	try:
		#to receive voice command over mqtt
		client= paho.Client("client-orin")
		client.on_message=on_voice_cmd_mqtt_message		
		client.connect(broker_ip, broker_port)
		client.loop_start()
		client.subscribe("voice_cmd_mqtt")#subscribe
	except:
		print("no mqtt")
		pass


	# try:
	# 	bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
	# 	print ("Opened CAN bus")
	# except IOError:
	# 	print ("Cannot open CAN bus")
	# 	return 
	
	subscription_manual_steering = node.create_subscription(Int64,'manual_steering', manual_steering_callback, 1)
	subscription_manual_throttle = node.create_subscription(Int64,'manual_throttle', manual_throttle_callback, 1)
	subscription_auto_throttle = node.create_subscription(Int64,'auto_throttle', auto_throttle_callback, 1)	
	subscription_pid_steering = node.create_subscription(Int64 , 'pid_steering' , pid_steering_callback , 1)
	subscription_mode_switch = node.create_subscription(Int64 , "mode_switch" , mode_switch_callback , 1)	
	subscription_voice_cmd = node.create_subscription(String , "voice_cmd" , voice_cmd_callback , 1)		
	subscription_lidar_min_dist = node.create_subscription(Float64 , "lidar_min_dist" , lidar_min_dist_callback , 1)		
	subscription_stop_sign = node.create_subscription(Int64 , 'stop_sign' , stop_sign_callback , 1)
	subscription_pure_pursuit = node.create_subscription(Float64MultiArray, "pure_pursuit", pure_pursuit_callback, 1)
	thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
	
	while rclpy.ok():
		
		if mode == 0:
			# manual mode
			# pwm_throttle = pwm(throttle)
			# pwm_steer = pwm(steer)
			pwm_steer = steering_angle
			pwm_throttle = 7
		else:
			# auto mode
			pwm_throttle = pwm(auto_throttle)
			pwm_steer = pwm(pid_steer)

		
		safe_distance_violation = False
		if lidar_min_dist < SAFE_DISTANCE:
			# print("Safe distance violation. Setting throttle to 0")
			safe_distance_violation = True
			if pwm_throttle > pwm(0):
				pwm_throttle = pwm(0)
		
		stop_sign_message = None
		if mode == 1 and stop_sign_detected and time.time() < (last_stop_time + 1.5): 
			#stop sign (in auto mode) -- stop for 1.5 seconds
			# print("STOP SIGN!")
			stop_sign_message = "-- Stop sign --"
			if pwm_throttle > pwm(0):
				pwm_throttle = pwm(0)


		# print("mode: %s, throttle: %d (auto: %d), steering: %d (auto: %d)" % ("Manual" if mode == 0 else "Auto", throttle, auto_throttle, steer, pid_steer))

		th = throttle
		st = steer
		if (mode==1):
			th = auto_throttle
			st = pid_steer

		stdscr.refresh()
		stdscr.addstr(1, 5, 'Mode: %s       ' % ("Manual" if mode == 0 else "Auto"))
		stdscr.addstr(2, 5, 'Throttle: %.2f  ' % throttle)
		stdscr.addstr(3, 5, 'Steering: %.2f  ' % steer)
		stdscr.addstr(2, 5, 'Throttle: %.2f  ' % (th * (100/max_throttle)))
		stdscr.addstr(3, 5, 'Steering: %.2f  ' % st)
		stdscr.addstr(7 , 5 , 'Warning: %s              ' % warning)
		stdscr.addstr(8 , 5 , 'ESC Battery: %s     ' % can_message)
		
		if safe_distance_violation:
			stdscr.addstr(4, 5, '-- Safe distance violation (%.2f m)--' % lidar_min_dist)
		else:
			stdscr.addstr(4, 5, '                             ')
		if stop_sign_message:
			stdscr.addstr(5, 5, stop_sign_message)
		else:
			stdscr.addstr(5, 5, '                             ')
		stdscr.addstr(6,0,'')


		# can_data = struct.pack('>hhI', pwm_throttle, pwm_steer, 0)
		# new_msg = can.Message(arbitration_id=0x1,data=can_data, is_extended_id = False)
		# bus.send(new_msg)
		#send actuation command to teensy over CAN bus

		# CAN does not send negative values properly so I just made the values positive by making them 
		try:
			with can.interface.Bus(bustype = 'socketcan' , channel = 'can0' , bitrate = 250000) as bus:
					can_data = struct.pack('>hhI', pwm_throttle + max_throttle, pwm_steer + max_steer , 0)
					new_msg = can.Message(arbitration_id=0x1,data=can_data, is_extended_id = False)
					try:
						bus.send(new_msg)
						warning = "None"
					except:
						warning = "Message not sent"
					try:
						can_message = int.from_bytes(bus.recv().data, 'little')
					except:
						warning = "Message not received"
		except IOError:
			warning = "Failed to find CAN bus"

		rate.sleep()

	rclpy.spin(node)
	rclpy.shutdown()

	
if __name__ == '__main__':
	main()
