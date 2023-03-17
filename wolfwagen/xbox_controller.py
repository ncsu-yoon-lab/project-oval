import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading

steering = 0
throttle = 0
mode = 0

topic_manual_steering = "manual_steering"
topic_manual_throttle = "manual_throttle"
topic_mode_switch = "mode_switch"

def joy_callback(data):
	global steering, throttle, mode
	# print(data.axes)
	if data.axes[4] != 1:
		throttle = int((1 - data.axes[4])*100)
	elif data.axes[5] != 1:
		throttle = int((1 - data.axes[5])*-100)
	else:
		throttle = 0
	steering = int(-data.axes[2]*100)
	
	if throttle>25:
	    throttle = 25
	elif throttle<-25:
		throttle = -25

	# is the mode for type of driving
	mode = data.buttons[0]	

	# print(throttle, steering)
	

				
def main(args=None):
	print("xbox_controller")
	rclpy.init(args=args)
	node = Node("xbox_controller_node")
	node.create_subscription(Joy, 'joy', joy_callback, 10)
	
	pub_manual_steering = node.create_publisher(Int64, topic_manual_steering, 10)
	pub_manual_throttle = node.create_publisher(Int64, topic_manual_throttle, 10)
	pub_mode_switch = node.create_publisher(Int64 , topic_mode_switch , 10)
	
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
	while rclpy.ok():
		m = Int64()
		m.data = steering
		pub_manual_steering.publish(m)
		m.data = throttle
		pub_manual_throttle.publish(m)
		print(throttle, steering)
		
		# putting a timer before and after the collection of the button data to stop it from taking another input too quickly
		time = node.create_rate(20, node.get_clock())
		time.sleep()

		m.data = mode
		
		# putting a timer before and after the collection of the button data to stop it from taking another input too quickly
		time = node.create_rate(20, node.get_clock())
		time.sleep()

		pub_mode_switch.publish(m)
		rate.sleep()


	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
