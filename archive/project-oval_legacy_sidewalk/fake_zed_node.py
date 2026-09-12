from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import threading
import curses
import time
import glob
import os
from natsort import natsorted

def main(args=None):

	rclpy.init(args=args)
	node = Node("fake_zed_node")

	# Subscription to joy topic - gets info from controller
	# Publishers to manual throttle and steer - publishes bounded number pre-PWM
	image_pub = node.create_publisher(Image, "/fake_zed/image", 10)

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(10, node.get_clock())
	
	# Get all image files
	image_extension = '*.jpg'
	image_files = []
	image_files.extend(glob.glob(os.path.join("/home/wolfwagen/ext-vol/ww_data_collection/images_small_path", image_extension)))
	# image_files.extend(glob.glob(os.path.join("onnx_segmentation/test_images", image_extension.upper())))
	
	image_files = natsorted(image_files)
	image_files_idx = 0

	print(len(image_files))

	br = CvBridge()
		
	while rclpy.ok():

		try:
			
			image = cv2.imread(image_files[image_files_idx])

			image_msg = br.cv2_to_imgmsg(image)

			image_pub.publish(image_msg)

			image_files_idx += 1

			if image_files_idx == len(image_files):
				image_files_idx = 0

			rate.sleep()
		except KeyboardInterrupt:
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
