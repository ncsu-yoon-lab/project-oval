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

stdscr = curses.initscr()
SHOW_IMAGES = True
DRAW_LINE_IMG = True


# Initializations
frame = None # frame is the OpenCV image
pose = None # pose is the orientation of the ZED

last_frame_time = time.time() # Used to compare the times in between frames
br = CvBridge() # Used to convert ROS2 frames to OpenCV frames


def image_callback(msg):
    # Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
    global frame, last_frame_time
    frame = br.imgmsg_to_cv2(msg)
    last_frame_time = time.time()

def adjust_gamma(image , gamma = 1.0):
    # Adjusts the brightness of frame to make it easier or harder to see colors
    # Increasing gamma makes it darker, decreasing gamma makes it brighter
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0 , 256)]).astype("uint8")
    return cv.LUT(image , table)

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
    height_modifier = 0.43 # Use to modify the height of crop_front image to check for going straight at intersection
    image = np.delete(image , slice(center - cut_width , center + cut_width) , 1)
    
    # Maybe crop this exact amount where left and right crop are
    image = image[int(height * (height_modifier)) : height - 1 , int(center/2)-100:int(center/2)+100]

def process_img(frame):
    # Process the images to change the colors and crop them
    global last_turn_time , left_crop_img , right_crop_img

    img = cv.cvtColor(frame , cv.COLOR_BGR2GRAY)
    height , width = img.shape # 720 , 2560

    front_image = crop_front(img , width , height)
    main_image = crop_main(img , width , height)

    color_frame_front = front_image
    bw_frame_front = cv.cvtColor(front_image , cv.COLOR_BGR2GRAY)
    color_frame_main = main_image
    bw_frame_main = cv.cvtColor(main_image , cv.COLOR_BGR2GRAY)

    height , width = main_image.shape  # Cropped size

    # Remove noise
    kernel_size = 7
    bw_frame_main = cv.GaussianBlur(bw_frame_main , (kernel_size , kernel_size) , 0)
    bw_frame_front = cv.GaussianBlur(bw_frame_front , (kernel_size , kernel_size) , 0)

    # Thresholding. If seeing some noise, increase the lower threshold
    lower_threshold = 160
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
    # I want to include from image here if possible
    left_crop_image = edges_frame[170 : , : 300]
    right_crop_image = edges_frame[170 : , 700 :]

    intersection_detection , turn = check_intersection(left_crop_image , right_crop_image , bw_frame_front)

    final_frame , lanes_detection , steering = check_lanes(edges_frame , color_frame_main)

    if SHOW_IMAGES:
        cv.imshow('main color' , color_frame_main)
        cv.imshow('main black and white' , bw_frame_main)
        cv.imshow('front color' , color_frame_front)
        cv.imshow('front black and white' , bw_frame_front)
        cv.imshow('edges frame' , edges_frame)
        cv.imshow('right crop' , right_crop_image)
        cv.imshow('left crop' , left_crop_image)
		cv.imshow('final frame' , final_frame)
        cv.waitKey(1)

def check_intersection(left , right , front):
    openings = np.array([0 , 0 , 0])
    intersection_check = False
    if left.sum() < 1000:
        openings[0] = 1
        intersection_check = True
    if right.sum() < 1000:
        openings[2] = 1
        intersection_check = True
    if front.sum() < 500000:
        openings[1] = 1

    # Random turn chooser
    if intersection_check:

        # Find available turns
        possible_turns = np.where(openings == 1)[0]

        # Choose random turn direction (0 = left , 1 = straight , 2 = right)
        turn = random.choice(possible_turns)
    
    return intersection_check , turn

def check_lanes(edges_frame , color_frame_main):
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
		#print("No lines detected")
		final = color_frame_main
		CTE = 0		
	else:
		left_line_x = []    #x-values of left lines 
		left_line_y = []    #y-values of left lines 
		right_line_x = []   #x-values of right lines 
		right_line_y = []   #y-values of right lines

		cnt_left = 0    # number of left lines
		cnt_right = 0   # number of right lines

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
				else:
					right_line_x.extend([x1, x2])
					right_line_y.extend([y1, y2])					
					cnt_right += 1
		
		
			
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
			

		if cnt_right > 0:
			#do 1D fitting                
			right_polyfit = np.polyfit(right_line_y, right_line_x, deg=1)
			poly_right = np.poly1d(right_polyfit)
			right_x_start = int(poly_right(MAX_Y))
			right_x_end = int(poly_right(MIN_Y))
			
			if DRAW_LINE_IMG:
				cv.line(line_image, (right_x_start, MAX_Y), (right_x_end, MIN_Y), LANE_COLOR, LANE_THICKNESS)

		
		car_center = int(color_frame_main.shape[1]/2)	#center of camera

		if cnt_left>0 and cnt_right>0:
			# Find CTE
			lane_center = (right_x_start+left_x_start)/2
			CTE = car_center - lane_center			
			
			if DRAW_LINE_IMG:
				cv.line(line_image, ( int((left_x_start+right_x_start)/2), MAX_Y), ( int((left_x_end + right_x_end)/2), MIN_Y), LANE_CENTER_COLOR, 5)
				cv.line(line_image, (car_center, MAX_Y), (car_center, MIN_Y), (255,255,0), 3)
				
				#Draw lane region
				mask = np.zeros_like(line_image)				
				vertices = np.array([[(left_x_start+10,MAX_Y),(left_x_end+10, MIN_Y), (right_x_end-10, MIN_Y), (right_x_start-10, MAX_Y)]], dtype=np.int32)
				cv.fillPoly(mask, vertices, LANE_REGION_COLOR)

				line_image = cv.addWeighted(line_image, 0.8, mask, 0.2, 0)				

		elif cnt_left+cnt_right == 0:
			CTE = 0
			#print('cannot find any lane markings')
		else:
			if cnt_left==0:
				CTE = 500
				#print('cannot find left lane marking')
			else:
				CTE = -500
				#print('cannot find right lane marking')


		final = color_frame_main
		
		if DRAW_LINE_IMG:
			final = cv.addWeighted(final, 1, line_image, 1, 0)
		

	return (final, CTE)
    