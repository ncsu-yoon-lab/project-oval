#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_msgs.msg import Float64
import struct
import os
import threading
import time
import curses
import serial
import math

#distributed ros doesn't work now, so let's use mqtt for now
import paho.mqtt.client as paho
broker_ip="eb2-3254-ub01.csc.ncsu.edu"
broker_port=12345

stdscr = curses.initscr()

teensy = serial.Serial(port = '/dev/ttyACM0' , baudrate = 115200 , timeout = .1)

#TODO: pwm should be moved to Teensy microcontroller
in_min = -100
in_max = +100
out_min = 6554
out_max = 13108

pwm_throttle_max = 2000
pwm_throttle_min = 1000

pwm_steer_max = 140
pwm_steer_min = 40

throttle = 0
steer = 0
mode = 0

pid_steer = 0
auto_throttle = 0	#published (and controller) by xbox_controller	

lidar_min_dist = 1000000	#for LIDAR-based obstacle detection/avoidance
SAFE_DISTANCE = 0.50	

def pwm(val):
	#TODO: input range check
	return (val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

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
	

	pid_steer = 0.9 * float(steer) +  90.0

	# if steer > 100:
	# 	steer = 100
	# elif steer < -100:
	# 	steer = -100
	# if (steer==100):
	# 		steer = 99
	
	# pid_steer = steer

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

def write_to_teensy(x , y):
    array = bytearray([x,y])
    #print(array)
    teensy.write(array)
    #print("SENT TO TEENSY")
    #teensy.write(bytes(y))
    #time.sleep(0.05)
    #arduino.write(bytes(y , 'utf-8'))
    #data = teensy.readline()
    #return data


def main(args=None):
	print("Driver node")
	rclpy.init(args=args)
	node = Node("Drive_node")

	# LOG FILE
	#f = open(file_name, "w+")

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
	
	subscription_manual_steering = node.create_subscription(Int64,'manual_steering', manual_steering_callback, 1)
	subscription_manual_throttle = node.create_subscription(Int64,'manual_throttle', manual_throttle_callback, 1)
	subscription_auto_throttle = node.create_subscription(Int64,'auto_throttle', auto_throttle_callback, 1)	
	subscription_pid_steering = node.create_subscription(Int64 , 'pid_steering' , pid_steering_callback , 1)
	subscription_mode_switch = node.create_subscription(Int64 , "mode_switch" , mode_switch_callback , 1)	
	subscription_voice_cmd = node.create_subscription(String , "voice_cmd" , voice_cmd_callback , 1)		
	subscription_lidar_min_dist = node.create_subscription(Float64 , "lidar_min_dist" , lidar_min_dist_callback , 1)		
	subscription_stop_sign = node.create_subscription(Int64 , 'stop_sign' , stop_sign_callback , 1)
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
	
	while rclpy.ok():
		
		if mode == 0:
			# manual mode
			pwm_throttle = pwm(throttle)
			pwm_steer = int(pwm(steer) * (100.0 / 3277.0) - 210.0)
		else:
			# auto mode
			pwm_throttle = pwm(auto_throttle)
			pwm_steer = int(pid_steer)

		
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
			# pwm_steer = pid_steer
		pwm_throttle = math.floor(pwm_throttle * (500.0 / 3277.0))
		#pwm_steer = math.floor(pwm_steer * (100.0 / 3277.0) - 210.0)
		if pwm_throttle > pwm_throttle_max:
			pwm_throttle = pwm_throttle_max
		if pwm_throttle < pwm_throttle_min:
			pwm_throttle = pwm_throttle_min
		if pwm_steer > pwm_steer_max:
			pwm_steer = pwm_steer_max
		if pwm_steer < pwm_steer_min:
			pwm_steer = pwm_steer_min

		stdscr.refresh()
		stdscr.addstr(1, 5, 'Mode: %s       ' % ("Manual" if mode == 0 else "Auto"))
		stdscr.addstr(2, 5, 'Throttle: %.2f  ' % pwm_throttle)
		stdscr.addstr(3, 5, 'Steering: %.2f  ' % pwm_steer)
		stdscr.addstr(4 , 5 , 'PID Steering: %.2f' % pid_steer)
		#f.write(str(time.time()) + "+" + str(pwm_steer))

		pwm_throttle = pwm_throttle // 10
		#print(pwm_throttle)
		write_to_teensy(pwm_throttle , pwm_steer)
		
		print("auto throttle: %d, auto steering: %d" % (auto_throttle, pid_steer))
		# if safe_distance_violation:
		# 	stdscr.addstr(4, 5, '-- Safe distance violation (%.2f m)--' % lidar_min_dist)
		# else:
		# 	stdscr.addstr(4, 5, '                             ')
		# if stop_sign_message:
		# 	stdscr.addstr(5, 5, stop_sign_message)
		# else:
		# 	stdscr.addstr(5, 5, '                             ')
		# stdscr.addstr(6,0,'')

		#send actuation command to teensy over CAN bus

		rate.sleep()

	rclpy.spin(node)
	rclpy.shutdown()
	#f.close()

	
if __name__ == '__main__':
	main()