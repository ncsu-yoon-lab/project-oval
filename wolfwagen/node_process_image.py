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
DRAW_LINE_IMG = True


# Initializations
frame = None # frame is the OpenCV image
pose = None # pose is the orientation of the ZED
previous_error = 0 # error for PID
message = ""
turning = ""
steer = 0
previous_error = 0

last_frame_time = time.time() # Used to compare the times in between frames
br = CvBridge() # Used to convert ROS2 frames to OpenCV frames



########################### Callbacks ############################

def image_callback(msg):
	# Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
	global frame, last_frame_time
	frame = br.imgmsg_to_cv2(msg)
	last_frame_time = time.time()


# def calib_callback(msg):
# 	global camera_info
# 	camera_info = msg
# 	print(type(msg))
# 	print(msg)


########################### Image Processing ############################

def adjust_gamma(image , gamma = 8.7):
	# Adjusts the brightness of frame to make it easier or harder to see colors
	# Increasing gamma makes it darker, decreasing gamma makes it brighter
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
					  for i in np.arange(0 , 256)]).astype("uint8")
	return cv.LUT(image , table)

def crop_main(image , width , height):
	# Crops the main image to just see the road ahead
	center = int(width / 2)
	cut_width = int(width / 4)
	image = np.delete(image , slice(center - cut_width , center + cut_width) , 1)

	image  = image[int(height / 2) : height - 1 , :]
	return image

def crop_front(image , width , height):
	# Crops the front image to see if there is a line in front at an intersection
	center = int(width / 2)
	cut_width = int(width / 4)
	height_modifier = 0.5 # Use to modify the height of crop_front image to check for going straight at intersection
	image = np.delete(image , slice(center - cut_width , center + cut_width) , 1)
	
	# Maybe crop this exact amount where left and right crop are
	image = image[int(height * (height_modifier)) : height - 1 , int(center/2)-100:int(center/2)+100]
	return image

def process_img(frame):
	# Process the images to change the colors and crop them

	
	global last_turn_time , left_crop_img , right_crop_img , lane_on_right , lane_on_left , keep_turning

	global gamma
	global gamma_debug


	
	#img = cv.cvtColor(frame , cv.COLOR_BGR2GRAY)
	img = frame

	if(gamma_debug):
		img = adjust_gamma(img, gamma)
		gamma += 0.01
	
	height , width , _ = img.shape # 720 , 2560
	
	front_image = crop_front(img , width , height)
	main_image = crop_main(img , width , height)
	color_frame_main = main_image

	bw_frame_front = cv.cvtColor(front_image , cv.COLOR_BGR2GRAY)
	bw_frame_front = front_image

	bw_frame_main = cv.cvtColor(main_image , cv.COLOR_BGR2GRAY)
	bw_frame_main = main_image

	height , width, _ = main_image.shape  # Cropped size

	# Remove noise
	kernel_size = 7
	bw_frame_main = cv.GaussianBlur(bw_frame_main , (kernel_size , kernel_size) , 0)
	bw_frame_front = cv.GaussianBlur(bw_frame_front , (kernel_size , kernel_size) , 0)

	# Thresholding. If seeing some noise, increase the lower threshold
	lower_threshold = 200
	upper_threshold = 255
	_, bw_frame_main = cv.threshold(bw_frame_main , lower_threshold , upper_threshold , cv.THRESH_BINARY)
	_, bw_frame_front = cv.threshold(bw_frame_front , lower_threshold , upper_threshold , cv.THRESH_BINARY)

	# Canny edge detection
	min = 70
	max = 200
	edges_frame = cv.Canny(bw_frame_main , min , max)

	# Edges could be too thin (which could make it difficult for Hough Transform to detect lines)
	# So, make the edges thicker
	kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
	edges_frame = cv.dilate(edges_frame, kernel, iterations=1)

	# Cropping to check for intersection openings
	left_crop_image = edges_frame[100 : 300, 0 : 200]

	final_frame , CTE = check_lanes(edges_frame , color_frame_main, width)

	# Option of showing all the images, can be toggled at top
	if SHOW_IMAGES:
		#cv.imshow('main color' , frame)
		#cv.imshow('main black and white' , bw_frame_main)
		#cv.imshow('front color' , color_frame_front)
		cv.imshow('front black and white' , bw_frame_front)
		cv.imshow('edges frame' , edges_frame)
		cv.imshow('left crop' , left_crop_image)
		cv.imshow('final frame' , final_frame)
		cv.waitKey(1)
	
	return lane_on_left



########################### Lane Processing ############################

def check_lanes(edges_frame , color_frame_main, width):
	DOT_COLOR = [61 , 217 , 108]
	DOT_SIZE = 5
	
	LINE_COLOR = (255 , 0 , 0)
	LINE_THICKNESS = 2
	
	LANE_COLOR = (255, 255, 0)
	LANE_THICKNESS = 5

	LANE_REGION_COLOR = (0, 255, 0)
	LANE_CENTER_COLOR = (0, 0, 255)

	CAR_CENTER_COLOR = (180, 180, 0)
	
	lines = cv.HoughLines(edges_frame , 1 , np.pi/180 , 150 , None , 0 , 0)

	# Cross Track Error	
	CTE = 0
	
	if lines is None: 
		final = color_frame_main
		CTE = 0		
		keep_turning = True
	else:
		keep_turning = False

		left_line_x = []	#x-values of left lines 
		left_line_y = []	#y-values of left lines 

		cnt_left = 0	# number of left lines

		if DRAW_LINE_IMG:					
			line_image = np.copy(color_frame_main)*0

		for line in lines:

			# Note: we are using the standard Hough Transform, not the probabilistic version			
			rho = line[0][0]
			theta = line[0][1]
			x1, y1, x2, y2 = get_end_points(rho, theta)
			
			# for x1, y1, x2, y2 in line:	
			if True:
				
				if x2 - x1 == 0:
					continue

				slope = (y2 - y1) / float(x2 - x1)
				slope_threshold = 0.2
				if abs(slope) < slope_threshold:
					continue

				if DRAW_LINE_IMG:										
					cv.line(line_image, (x1, y1), (x2, y2), LINE_COLOR, LINE_THICKNESS)
					cv.circle(line_image, (x1, y1), DOT_SIZE, DOT_COLOR, -1)
					cv.circle(line_image, (x2, y2), DOT_SIZE, DOT_COLOR, -1)
		
				if slope <= 0:
					left_line_x.extend([x1, x2])
					left_line_y.extend([y1, y2])					
					cnt_left += 1										
		
		
			
		MIN_Y = 0	# <-- top of lane markings
		MAX_Y = color_frame_main.shape[0] # <-- bottom of lane markings

		left_polyfit = None
		right_polyfit = None

		
		if cnt_left > 0:
			#do 1D fitting
			left_polyfit = np.polyfit(left_line_y, left_line_x, deg=1)
			poly_left = np.poly1d(left_polyfit)
			left_x_start = int(poly_left(MAX_Y))
			left_x_end = int(poly_left(MIN_Y))
			
			if DRAW_LINE_IMG:
				cv.line(line_image, (left_x_start, MAX_Y), (left_x_end, MIN_Y), LANE_COLOR, LANE_THICKNESS)

		
		car_center = int(color_frame_main.shape[1]/2)	#center of camera

		if cnt_left > 0:
			# Lines detected on left and right

			# Find CTE
			lane_center = (left_x_start)
			CTE = car_center - lane_center			
			
			if DRAW_LINE_IMG:
				cv.line(line_image, ( int((lane_center)), MAX_Y), ( int((left_x_end)), MIN_Y), LANE_CENTER_COLOR, 5)
				cv.line(line_image, (car_center, MAX_Y), (car_center, MIN_Y), (255,255,0), 3)
				
				#Draw lane region
				mask = np.zeros_like(line_image)				
				vertices = np.array([[(left_x_start+10,MAX_Y),(left_x_end+10, MIN_Y)]], dtype=np.int32)
				cv.fillPoly(mask, vertices, LANE_REGION_COLOR)

				line_image = cv.addWeighted(line_image, 0.8, mask, 0.2, 0)				

		elif cnt_left == 0:
			# No lines detected on left or right

			CTE = 0


		final = color_frame_main
		
		if DRAW_LINE_IMG:
			final = cv.addWeighted(final, 1, line_image, 1, 0)
		

	return (final, CTE)
	

def get_end_points(rho , theta):
	# Gets the two points of the line detected with cv.HoughLines
	x = math.cos(theta)
	y = math.sin(theta)

	x0 = x * rho
	y0 = y * rho

	x1 = int(x0 + 1000 * (-y))
	y1 = int(y0 + 1000 * (x))
	x2 = int(x0 - 1000 * (-y))
	y2 = int(y0 - 1000 * (x))

	return x1 , y1 , x2 , y2



########################### Controls ############################

def PID(CTE , FREQ):
	global previous_error
	Kp = 0.15	
	Ki = 0.0
	Kd = 0.01
	dt = 1/float(FREQ)
	integral = 0

	set_point = 0
	error = set_point - CTE
	integral = integral + error * dt
	derivative = (error - previous_error) / dt
	steering_cmd = Kp * error + Ki * integral + Kd * derivative
	previous_error = error

	return steering_cmd


########################### Main ############################

def main(args = None):
	global message , steer , turning , keep_turning , lane_on_left , lane_on_right

	global gamma
	global gamma_debug
	# global camera_info
	# camera_info = ""

	# a proper gamma value, given the current thresholds seems to be between 0.7-1.7
	# Gamma values can differ for right and left frames given small differences in brightness
	# 
	gamma = 0.001
	gamma_debug = False

	lane_on_left = True
	lane_on_right = True
	keep_turning = True
	rclpy.init(args = args)
	node = Node("Process_image_node")
	zed_img_subscription = node.create_subscription(
		Image,
		'/zed/zed_node/stereo/image_rect_color',
		image_callback,
		5
	)


	steering_publisher = node.create_publisher(Int64 , 'pid_steering' , 1)

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	FREQ = 20
	rate = node.create_rate(FREQ , node.get_clock())


	# ROS2 loop while ROS2 is still running and is ok
	while rclpy.ok():

		# Checking if there are frames (waiting for ZED node to start publishing)
		if frame is not None:

			# If the time in-between frames is too long
			if time.time() - last_frame_time > 3:
				break
			
			# Get the values needed from process_img function
			# intersection_detection = (boolean) if an intersection is detected
			# turn = (int) direction to turn if an intersection is detected
			# CTE = (int) measured distance error
			# keep_turning = (boolean) if turn should be completed because lines are detected
			# lane_on_left = (boolean) if lane is detected on left with left frame
			# lane_on_right = (boolean) if lane is detected on right with right frame
			CTE = process_img(frame)

			turning = "PID MODE"
			steer = PID(CTE , FREQ)

			# Send the steering values to pwm_gen
			# Might want to eventually include a throttle publisher so we can stop when no lines are detected
			# or if there is some other error
			msg = Int64()
			msg.data = int(steer)
			steering_publisher.publish(msg)

		# Display of all the important messages
		stdscr.refresh()
		stdscr.addstr(1 , 5 , 'Turning Condition: %s		   ' % str(steer))

		rate.sleep()
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()	
