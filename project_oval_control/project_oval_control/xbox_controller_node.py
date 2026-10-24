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

# Zero throttle/steer if no joy message arrives for this long (controller off or out of range).
# Requires joy_node to run with autorepeat_rate > 0, otherwise a steadily held stick goes silent too.
JOY_TIMEOUT_S = 1.0

# /joy axis indices for the Xbox Wireless Controller over Bluetooth
AXIS_STEER = 2  # right stick X
AXIS_RT = 4     # rests at +1.0, -1.0 fully pressed
AXIS_LT = 5

# steer should be bounded between [-100, +100]
manual = True
toggle_flag = False
steer = 0
throttle = 0
switch_time = time.time()
a_button_pressed = False
mode_button_down = False
last_joy_time = None
triggers_seen_at_rest = set()

def trigger_pressed(data, axis):
	"""How far a trigger is pressed, 0.0 to 1.0.

	Reads as released until the trigger has reported its rest position once: some
	drivers report 0.0 (which maps to half pressed) before a trigger is first touched.
	"""
	value = data.axes[axis]
	if value >= 0.9:
		triggers_seen_at_rest.add(axis)
	if axis not in triggers_seen_at_rest:
		return 0.0
	return (1.0 - value) / 2.0

def joy_callback(data):
	global steer, throttle, manual, switch_time, a_button_pressed, mode_button_down, last_joy_time
	last_joy_time = time.time()
	a_button_pressed = data.buttons[7] == 1

	# Check for toggle between manual and auto driving (A Button).
	# Toggle only on the press itself: with autorepeat a held button keeps re-sending 1.
	mode_pressed = data.buttons[0] == 1
	if(mode_pressed and not mode_button_down and time.time() - switch_time > 0.5):
		manual = not manual
		switch_time = time.time()
	mode_button_down = mode_pressed
	# Input for throttle and steer are between [-1.0, 1.0]

	# If the a button is being pressed, we want to be in turbo mode
	# In manual mode the driver turns the car with /xbox_controller/throttle and drives it
	# straight with /xbox_controller/steer, so the triggers feed steer and the stick feeds throttle.
	# RT drives forward, LT in reverse
	drive_input = ((trigger_pressed(data, AXIS_RT) - trigger_pressed(data, AXIS_LT))**3) * (1 if a_button_pressed else 0.3)
	turn_input = (data.axes[AXIS_STEER]**3) * (1 if a_button_pressed else 0.3)

	throttle = int(100 * turn_input)
	if throttle > MAX_THROTTLE_FORWARD:
		throttle = MAX_THROTTLE_FORWARD
	elif throttle < (-1 * MAX_THROTTLE_REVERSE):
		throttle = (-1 * MAX_THROTTLE_REVERSE)

	steer = int(100 * drive_input)

def telemetry_callback(data):
	global manual, switch_time
	if (data.data == "cmd:mode_switch" and time.time() - switch_time > 0.5):
		manual = not manual
		switch_time = time.time()

def joy_is_stale():
	return last_joy_time is None or time.time() - last_joy_time > JOY_TIMEOUT_S

def main(args=None):
	global throttle, steer

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
	prev_stale = False
		
	while rclpy.ok():

		try:
			# Controller watchdog: never keep driving on stale stick values
			stale = joy_is_stale()
			if stale:
				throttle = 0
				steer = 0
			if stale != prev_stale:
				node.get_logger().warn('No joy input for %.1fs: throttle and steer zeroed' % JOY_TIMEOUT_S if stale else 'Joy input restored')
				prev_stale = stale

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
			stdscr.addstr(5, 25, 'Joy: %s  ' % ('LOST' if stale else 'OK'))
			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
