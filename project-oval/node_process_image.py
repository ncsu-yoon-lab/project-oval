#!/usr/bin/env python

'''
Simplified LaneDetection that is constantly running lane detection and intersection detection functions.

Intersection detection is constantly running:
- When intersection is detected (within some confidence) it will turn or stay straight
	- Runs turning or straight command for a set amount of time then checks for new intersection
- When intersection is not detected it will stay within the lines detected (within some confidence) using PID

Both will output steering values that will be communicated over CAN to the Teensy
'''


import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int64
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import numpy as np
import curses
import time
import math
import random
import threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

stdscr = curses.initscr()
SHOW_IMAGES = True

# Initializations
frame = None # frame is the OpenCV image

last_frame_time = time.time() # Used to compare the times in between frames
br = CvBridge() # Used to convert ROS2 frames to OpenCV frames



########################### Callbacks ############################

def image_callback(msg):
	# Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
	global frame, last_frame_time
	frame = br.imgmsg_to_cv2(msg)
	last_frame_time = time.time()


########################### Image Processing ############################

def crop_main(image , width , height):
	# Crops the main image to just see the road ahead
	center = int(width / 2)
	cut_width = int(width / 4)
	image = np.delete(image , slice(center - cut_width , center + cut_width) , 1)

	image  = image[int(height / 2) : height - 1 , :]
	return image

def process_img(frame):

	img = frame
	
	height , width , _ = img.shape # 720 , 2560

	main_image = crop_main(img , width , height)
	color_frame_main = main_image

	bw_frame_main = cv.cvtColor(main_image , cv.COLOR_BGR2GRAY)
	bw_frame_main = main_image

	height , width, _ = main_image.shape  # Cropped size

	# Remove noise
	kernel_size = 7
	bw_frame_main = cv.GaussianBlur(bw_frame_main , (kernel_size , kernel_size) , 0)

	# Thresholding. If seeing some noise, increase the lower threshold
	lower_threshold = 160
	upper_threshold = 255
	_, bw_frame_main = cv.threshold(bw_frame_main , lower_threshold , upper_threshold , cv.THRESH_BINARY)


	# Option of showing all the images, can be toggled at top
	if SHOW_IMAGES:
		cv.imshow('main color' , frame)
		cv.imshow('main black and white' , bw_frame_main)
		cv.waitKey(1)



########################### Main ############################

def main(args = None):
	global frame
	rclpy.init(args = args)
	node = Node("Process_image_node")
	zed_img_subscription = node.create_subscription(
		Image,
		'/zed/zed_node/stereo/image_rect_color',
		image_callback,
		5
	)

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	FREQ = 20
	rate = node.create_rate(FREQ , node.get_clock())

	is_frame = False

	# ROS2 loop while ROS2 is still running and is ok
	while rclpy.ok():

		# Checking if there are frames (waiting for ZED node to start publishing)
		if frame is not None:
			is_frame = True

			# If the time in-between frames is too long
			if time.time() - last_frame_time > 3:
				break
			
			# Process the frame received from ZED
			process_img(frame)


		# Display of all the important messages
		stdscr.refresh()
		stdscr.addstr(1 , 5 , 'Process Image Node')
		stdscr.addstr(3 , 5 , 'Frame Received: %s	  ' % str(is_frame))

		rate.sleep()
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()	
