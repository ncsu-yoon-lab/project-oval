#!/usr/bin/env python

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import time
import threading


# Initializations
frame = None # frame is the OpenCV image

br = CvBridge() # Used to convert ROS2 frames to OpenCV frames

def image_callback(msg):
	# Receives ROS2 message of frame from ZED and converts it into OpenCV frame and stores last_frame_time
	global frame
	frame = br.imgmsg_to_cv2(msg)

def main(args = None):

	rclpy.init(args = args)
	node = Node("Process_image_node")
	node.create_subscription(
		Image,
		'/zed/zed_node/left/image_rect_color',
		image_callback,
		5
	)
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	FREQ = 20
	rate = node.create_rate(FREQ , node.get_clock())
	last_time = time.time()

	while rclpy.ok():

		if frame is not None:
			if time.time() - last_time > 1.0:
				cv.imwrite('/media/wolfwagen1/c9c2a9fe-c435-4115-9237-57bc783cf964/outdoor_test_images/frame_' + str(int(time.time() * 1000)) + '.png', frame)
				last_time = time.time()


		rate.sleep()
		
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()	