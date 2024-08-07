#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading
import curses

#Joy node should be running  
#ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0

stdscr = curses.initscr()

steering = 0
throttle = 0
auto_throttle = 15  #in auto mode

MAX_MANUAL_THROTTLE = 40
MAX_AUTO_THROTTLE = 17

mode_switch_requested = 0
logging_switch_requested = 0

axis_throttle = 1
axis_steering = 2
axis_mode = 0
x_button = 2

topic_manual_steering = "manual_steering"
topic_manual_throttle = "manual_throttle"
topic_mode_switch = "mode_switch"

sensitivity = 28  #used to be 100 on old motor


def joy_callback(data):
	global steering, throttle, auto_throttle, mode_switch_requested, logging_switch_requested

	throttle = int(data.axes[axis_throttle]*sensitivity)
	if throttle>MAX_MANUAL_THROTTLE:
		throttle = MAX_MANUAL_THROTTLE
	elif throttle<-MAX_MANUAL_THROTTLE:
		throttle = -MAX_MANUAL_THROTTLE

	# LB/RB to decrease/increase auto throttle (=fixed throttle when the car in the auto mode) 
	# just publish auto_throttle from here. Driver (pwm_gen) will use it. 
	if data.buttons[4]:
		auto_throttle -= 1
	if data.buttons[5]:
		auto_throttle += 1

	steering = int(-data.axes[axis_steering]*100)
	
	# Let the mode switch happen in the driver, not here
	if data.buttons[axis_mode]:
		mode_switch_requested = 1
		 
	

				
def main(args=None):
	global mode_switch_requested

	global logging_switch_requested

	print("xbox_controller")
	rclpy.init(args=args)
	node = Node("xbox_controller_node")
	node.create_subscription(Joy, 'joy', joy_callback, 1)
	
	pub_manual_steering = node.create_publisher(Int64, topic_manual_steering, 1)
	pub_manual_throttle = node.create_publisher(Int64, topic_manual_throttle, 1)
	pub_mode_switch = node.create_publisher(Int64 , topic_mode_switch , 1)

	
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())

	while rclpy.ok():
		m = Int64()
		log = Int64()
		m.data = steering
		pub_manual_steering.publish(m)

		m.data = throttle
		pub_manual_throttle.publish(m)
		
		# instead of keeping track of the mode here, just let the driver know that there is a mode-switch request
		if mode_switch_requested:
			m.data = 1
			pub_mode_switch.publish(m)
			mode_switch_requested = 0

		stdscr.refresh()
		stdscr.addstr(1, 5, 'XBOX CONTROLLER NODE')
		stdscr.addstr(3, 5, 'Manual Throttle : %d                ' % throttle)
		stdscr.addstr(4, 5, 'Steering : %d                       ' % steering)
		
		rate.sleep()

	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
    main()
