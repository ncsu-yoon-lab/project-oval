import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Int64
import struct
import can
import os
import threading

#os.system('sudo ip link set can0 up type can bitrate 250000')



class PWMSubscriber(Node):

	def __init__(self):
		super().__init__('pwm_subscriber')
		
		self.in_min = -100
		self.in_max = +100
		self.out_min = 6554
		self.out_max = 13108

		self.throttle = 0
		self.steer = 0
		self.mode = 0
		self.pid_steer = 0
			
		self.subscription_manual_steering = self.create_subscription(Int64,'manual_steering', self.manual_steering_callback, 10)
		self.subscription_manual_throttle = self.create_subscription(Int64,'manual_throttle', self.manual_throttle_callback, 10)
		self.subscription_mode_switch = self.create_subscription(Int64 , "mode_switch" , self.mode_switch_callback , 10)
		
		self.subscription_pid_steering = self.create_subscription(Int64 , 'pid_steering' , self.pid_steering_callback , 10)

		try:
			bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
			print ("Opened CAN bus")
		except IOError:
			print ("Cannot open CAN bus")
			return 

		rate = self.create_rate(20, self.get_clock())
		print("test")
		
		thread = threading.Thread(target = rclpy.spin, args=(self, ), daemon=True)
		thread.start()

		while rclpy.ok():
			if self.mode == 0:
				pwm_throttle = self.pwm(self.throttle)
				pwm_steer = self.pwm(self.steer)
				
			else:
				pwm_throttle = self.pwm(23)
				pwm_steer = self.pwm(self.pid_steer)

				
			print ('throttle: %d, steer: %d' % (pwm_throttle, pwm_steer))	
			can_data = struct.pack('>hhI', pwm_throttle, pwm_steer, 0)
			new_msg = can.Message(arbitration_id=0x1,data=can_data, is_extended_id = False)
			bus.send(new_msg)
			rate.sleep()


	def mode_switch_callback(self, data):
		if data.data == 1:
			self.mode = (1 if self.mode == 0 else 0)
			print("mode switched")

			
	def manual_steering_callback(self, data):
		self.steer = data.data

	def manual_throttle_callback(self, data):
		self.throttle = data.data
		

		
	def pwm(self, val):
		#TODO: input range check
		return (val - self.in_min) * (self.out_max - self.out_min) // (self.in_max - self.in_min) + self.out_min

	def pid_callback_throttle(self, data):
		global throttle
		throttle = data.data

	def pid_steering_callback(self, data):

		steer = data.data
		print(steer)
		if steer > 100:
			steer = 100
		elif steer < -100:
			steer = -100
			
		if (steer==100):
				steer = 99
		
		self.pid_steer = steer



				
def main(args=None):
	print("pwm_node")
	rclpy.init(args=args)
	node = PWMSubscriber()

	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
