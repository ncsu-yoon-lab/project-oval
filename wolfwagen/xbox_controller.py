import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading

#Usage: 
# ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0


steering = 0
throttle = 0
auto_throttle = 23	#in auto mode

mode = 0

mode_switch_requested = 0

axis_throttle = 1
axis_steering = 2
axis_mode = 0

topic_manual_steering = "manual_steering"
topic_manual_throttle = "manual_throttle"
topic_auto_throttle = "auto_throttle"
topic_mode_switch = "mode_switch"



def joy_callback(data):
	global steering, throttle, auto_throttle, mode, mode_switch_requested
	
	'''
	if data.axes[4] != 1:
		throttle = int((1 - data.axes[4])*100)
	elif data.axes[5] != 1:
		throttle = int((1 - data.axes[5])*-100)
	else:
		throttle = 0
	'''

	throttle = int(data.axes[axis_throttle]*100)
	if throttle>25:
	    throttle = 25
	elif throttle<-25:
		throttle = -25

	# LB/RB to decrease/increase auto throttle (=fixed throttle when the car in the auto mode) 
	# just publish auto_throttle from here. Driver (pwm_gen) will use it. 
	if data.buttons[4]:
		auto_throttle -= 1
	if data.buttons[5]:
		auto_throttle += 1
	if auto_throttle>30:
	    auto_throttle = 30
	elif auto_throttle<-30:
		auto_throttle = -30


	steering = int(-data.axes[axis_steering]*100)

	
	# if data.buttons[axis_mode]:
	# 	mode = (mode + 1) % 2 	# assuming there are only two modes. If there are N modes, % N
	
	# Let the mode switch happen in the driver, not here
	if data.buttons[axis_mode]:
		mode_switch_requested = 1
		 
	

				
def main(args=None):
	global mode_switch_requested

	print("xbox_controller")
	rclpy.init(args=args)
	node = Node("xbox_controller_node")
	node.create_subscription(Joy, 'joy', joy_callback, 1)
	
	pub_manual_steering = node.create_publisher(Int64, topic_manual_steering, 1)
	pub_manual_throttle = node.create_publisher(Int64, topic_manual_throttle, 1)
	pub_auto_throttle = node.create_publisher(Int64, topic_auto_throttle, 1)
	pub_mode_switch = node.create_publisher(Int64 , topic_mode_switch , 1)
	
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
	while rclpy.ok():
		m = Int64()
		m.data = steering
		pub_manual_steering.publish(m)
		m.data = throttle
		pub_manual_throttle.publish(m)
		m.data = auto_throttle
		pub_auto_throttle.publish(m)

		# m.data = mode
		# pub_mode_switch.publish(m)

		# instead of keeping track of the mode here, just let the driver know that there is a mode-switch request
		if mode_switch_requested:
			m.data = 1
			pub_mode_switch.publish(m)
			mode_switch_requested = 0


		# print("mode: %d, throttle: %d (auto: %d), steering: %d" % (mode, throttle, auto_throttle, steering))
		print("throttle: %d (auto: %d), steering: %d" % (throttle, auto_throttle, steering))
		
		rate.sleep()


	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
