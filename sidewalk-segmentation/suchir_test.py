# #!/usr/bin/env python
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
import cv2
# from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
# from std_msgs.msg import Int64

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = "/zed/zed_node/stereo/image_rect_color"
#PID Coefficients
Kp = 0.3
Kd = 0.05
# NOTE: no need for I shown via testing
Ki = 0

FIXED_THROTTLE = 4

# For color filtering
lower_yellow_threshold = np.array([0, 200, 200], dtype= "uint8")
upper_yellow_threshold = np.array([150, 255, 255], dtype= "uint8")

# br = CvBridge()
image = None
# def camera_callback(data):
#     global image
#     image = br.imgmsg_to_cv2(data)
#     image = image[:,:,:3]

def process_image(img):

    img[img==0]=1

    # Convert segmentation mask to proper image format
    processed_img = np.zeros_like(img, dtype=np.uint8)

    # Assuming you want to detect lines for specific classes
    # Adjust these values based on which classes represent roads/paths
    road_classes = [1, 3, 4]  # modify these values based on your segmentation classes
    for class_id in road_classes:
        processed_img[img == class_id] = 255

    np.savetxt('output3.txt', processed_img, fmt="%d")

    # Apply Gaussian blur to smooth edges
    blurred = cv2.GaussianBlur(processed_img, (5, 5), 0)

    # Apply Canny with appropriate thresholds for binary image
    edges = cv2.Canny(blurred, 30, 100)

    # Dilate edges to connect nearby lines
    kernel = np.ones((3,3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)

    # Modify HoughLinesP parameters for smaller image
    lines = cv2.HoughLinesP(
        edges, 
        rho=1,
        theta=np.pi/180,
        threshold=15,        # Lower threshold due to smaller image
        minLineLength=20,    # Adjust based on image size
        maxLineGap=5        # Smaller gap due to image size
    )
    return processed_img, edges
    # Debug: show intermediate results
    cv2.imshow('Processed Binary', processed_img)
    cv2.imshow('Edges', edges)
    cv2.waitKey(1)

    # Rest of your line processing code...
# def main():

#     rclpy.init()
#     node = rclpy.create_node('lane_follower')
#     node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
#     cv_steering_pub = node.create_publisher(Int64, "cv_steer", 10)
#     cv_throttle_pub = node.create_publisher(Int64, "cv_throttle", 10)

#     thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
#     thread.start()

#     FREQ = 10
#     rate = node.create_rate(FREQ, node.get_clock())
    
#     while rclpy.ok() and image is None:
#         print("Not receiving image topic")
#         rate.sleep()

#     cte = 0
#     curr_time = time.time()
#     integrated_error = 0 

#     while rclpy.ok():
#         # Process one image. The return value will be use for `something` later.
#         old_cte = cte
#         old_time = curr_time
#         cte = process_image(image)
#         curr_time = time.time()
#         integrated_error += (curr_time - old_time) * cte
#         response = -1 * Kp * cte - Kd * (cte - old_cte)/(curr_time - old_time) - Ki * integrated_error
        
#         published_response = Int64()
#         published_response.data = int(response)

#         print("PID response: " + str(published_response.data))
#         cv_steering_pub.publish(published_response)

#         cv_throttle_val = Int64()
#         cv_throttle_val.data = int(FIXED_THROTTLE)
#         cv_throttle_pub.publish(cv_throttle_val)

#         rate.sleep()

#     node.destroy_node()
#     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
