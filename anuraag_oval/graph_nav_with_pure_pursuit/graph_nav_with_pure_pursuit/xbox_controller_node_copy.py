#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64, Bool, String
import threading
import curses
import time

stdscr = curses.initscr()

# Throttle max and min
MAX_THROTTLE_FORWARD = 100
MAX_THROTTLE_REVERSE = 100
# Split-stick control:
#   right stick up/down -> throttle
#   left stick left/right -> steering
THROTTLE_AXIS = 2
STEER_AXIS = 1

# steer should be bounded between [-100, +100]
manual = True
toggle_flag = False
steer = 0
throttle = 0
switch_time = time.time()
a_button_pressed = False

def joy_callback(data):
	global steer, throttle, manual, switch_time, a_button_pressed
	a_button_pressed = data.buttons[7] == 1

	# Check for toggle between manual and auto driving (A Button)
	if(data.buttons[0] == 1 and time.time() - switch_time > 0.5):
		manual = not manual
		switch_time = time.time()
	# Input for throttle and steer are between [-1.0, 1.0]

	# If the a button is being pressed, we want to be in turbo mode
	throttle_input = (data.axes[THROTTLE_AXIS]**3) * (1 if a_button_pressed else 0.3)
	steer_input = (data.axes[STEER_AXIS]**3) * (1 if a_button_pressed else 0.3)

	# Convert this to [-20, +20] for throttle and [-100, 100] for steer
	throttle = int(100 * throttle_input)
	if throttle > MAX_THROTTLE_FORWARD:
		throttle = MAX_THROTTLE_FORWARD
	elif throttle < (-1 * MAX_THROTTLE_REVERSE):
		throttle = (-1 * MAX_THROTTLE_REVERSE)
	
	steer = int(100 * steer_input)

def telemetry_callback(data):
	global manual, switch_time
	if (data.data == "cmd:mode_switch" and time.time() - switch_time > 0.5):
		manual = not manual
		switch_time = time.time()

def main(args=None):

	rclpy.init(args=args)
	node = Node("xbox_controller_node")

	# Subscription to joy topic - gets info from controller
	joy_sub = node.create_subscription(Joy, "joy", joy_callback, 5)
	# Publishers to manual throttle and steer - publishes bounded number pre-PWM
	throttle_pub = node.create_publisher(Int64, "/xbox_controller/throttle", 10)
	steer_pub = node.create_publisher(Int64, "/xbox_controller/steer", 10)
	mode_pub = node.create_publisher(Bool, "/xbox_controller/mode", 10)
	tel_msg_pub = node.create_publisher(String, "/telemetry_message", 10)
	
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())

	prev_manual = manual  # Track previous manual state
		
	while rclpy.ok():

		try:
			# If we're in manual mode, we send the actuation
			if(manual):
				send_data = Bool()
				send_data.data = True
				mode_pub.publish(send_data)
				# Publishing for actuation
			
				steer_msg = Int64()
				steer_msg.data = steer
				steer_pub.publish(steer_msg)

				throttle_msg = Int64()
				throttle_msg.data = throttle
				throttle_pub.publish(throttle_msg)

			else:
				send_data = Bool()
				send_data.data = False
				mode_pub.publish(send_data)
			
			
			# Only publish telemetry message when manual changes
			if manual != prev_manual:
				tel_msg = String()
				tel_msg.data = f'mode:{"manual" if manual else "auto"}'
				tel_msg_pub.publish(tel_msg)
				prev_manual = manual

			stdscr.refresh()
			stdscr.addstr(1, 25, 'Xbox Controller       ')
			stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
			stdscr.addstr(3, 25, 'Steer: %.2f  ' % steer)
			stdscr.addstr(4, 25, 'Manual: %s  ' % str(manual))
			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
