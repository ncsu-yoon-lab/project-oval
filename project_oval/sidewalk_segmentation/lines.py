#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
from std_msgs.msg import Int64

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

br = CvBridge()
image = None
def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]


def process_image(img):
    height, width = img.shape
    img[img > 1] = 30
    print(img.shape)


    processed_img(img)
    
    # Line detection
    edges = cv2.Canny(processed_img, 250, 400)
    edges_copy = np.copy(edges)

    lines = cv2.HoughLinesP(
        edges_copy, 
        rho = 1,   
        theta = np.pi / 180,
        threshold = 10,
        lines = np.array([]),
        minLineLength = 30,
        maxLineGap = 10
    )

    line_img = np.zeros_like(processed_img, dtype=np.uint8)
    line_color = [0, 255, 0]
    line_thickness = 1
    dot_color = [0, 255, 0]
    dot_size = 3
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    if lines is None:
        lane_center = processed_img.shape[1] / 2
        print("NO LINES DETECTED!")
        return 0
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1)/(x2 - x1)
            # Filter slopes that aren't steep enough
            if abs(slope) >= 0.25:
                # Sort into left, right lines
                if slope > 0:
                    left_line_x.append(x1)
                    left_line_x.append(x2)
                    left_line_y.append(y1)
                    left_line_y.append(y2)
                else:
                    right_line_x.append(x1)
                    right_line_x.append(x2)
                    right_line_y.append(y1)
                    right_line_y.append(y2)
                
                cv2.line(line_img, (x1, y1), (x2, y2), line_color, line_thickness)

    lane_center = 0
    poly_left = None
    poly_right = None
    # If we can't see any lines at all (uh oh)

    # If we can't see left or right lines
    if len(left_line_x) == 0:
        print("No right line detected")
        return -250
    elif len(right_line_x) == 0:
        print("No left line detected")
        return 250
    else:
        # Create a polynomial fit of left lines, right lines
        left_line_fit = np.polyfit(left_line_y, left_line_x, deg=1)
        right_line_fit = np.polyfit(right_line_y, right_line_x, deg=1)

        # Create function such that f(y) = x
        poly_left = np.poly1d(left_line_fit)
        poly_right = np.poly1d(right_line_fit)

        # MAX_Y is the "bottom" of our image
        MAX_Y = processed_img.shape[0]
        # MIN_Y = min(left_line_y.append(right_line_y))

        # Find the close end of the left, right lanes
        left_x_close = poly_left(MAX_Y)
        right_x_close = poly_right(MAX_Y)
        # Find the center of our calculated lanes
        lane_center = (left_x_close + right_x_close) / 2
    # Center of our car is based off camera location
    car_center = processed_img.shape[1] / 2

    if poly_left is not None and poly_right is not None:
        print("Left lane slope", poly_left.c)
        print("Right lane slope", poly_right.c)
    print("Center of lane (calculated):", lane_center)
    print("Center of car", car_center)
    print("CTE", car_center - lane_center)
    # cv2.circle(line_img, (x1, y1), dot_size, dot_color, -1)
    # cv2.circle(line_img, (x2, y2), dot_size, dot_color, -1)
    overlay = cv2.addWeighted(processed_img, 1.0, line_img, 1.0, 0.0)
    processed_img = overlay
    
    cv2.imshow('Complete processed image (with lines)', processed_img)
    cv2.waitKey(1)
    # Positive value == car is too far right, negative too far left
    return car_center - lane_center

def main():

    rclpy.init()
    node = rclpy.create_node('lane_follower')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    cv_steering_pub = node.create_publisher(Int64, "cv_steer", 10)
    cv_throttle_pub = node.create_publisher(Int64, "cv_throttle", 10)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    cte = 0
    curr_time = time.time()
    integrated_error = 0 

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        old_cte = cte
        old_time = curr_time
        cte = process_image(image)
        curr_time = time.time()
        integrated_error += (curr_time - old_time) * cte
        response = -1 * Kp * cte - Kd * (cte - old_cte)/(curr_time - old_time) - Ki * integrated_error
        
        published_response = Int64()
        published_response.data = int(response)

        print("PID response: " + str(published_response.data))
        cv_steering_pub.publish(published_response)

        cv_throttle_val = Int64()
        cv_throttle_val.data = int(FIXED_THROTTLE)
        cv_throttle_pub.publish(cv_throttle_val)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
