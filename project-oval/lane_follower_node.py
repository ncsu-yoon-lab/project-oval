#!/usr/bin/env python

"""
ROS2 Lane Following Node with PID Control

This node subscribes to edge detection data and publishes steering commands
to keep the vehicle at a target distance from lane edges using PID control.
"""

# Import required ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Int64, Bool  # Added Bool import (was missing)
from onnx_segmentation.path_segmentation import PathSegmentation
from onnx_segmentation.lane_detector import LaneDetector

# Other libraries
import time
import math

# PID Controller Coefficients
Kp = 0.02
Kd = 0.0025
Ki = 0

# Target distance from lane edge
SETPOINT = 1280/2

SHOW_IMAGE = False
DEBUG_VIDEO = False

# ROS2 Topic Names
STEER_TOPIC = '/lane_follower/steer'

if DEBUG_VIDEO:
    IMAGE_TOPIC = "/fake_zed/image"
else:
    IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"

class LaneFollowerNode(Node):
    """
    ROS2 Node that implements lane following using PID control
    
    Subscribes to:
    - Edge distance measurements
    - Edge detection status
    
    Publishes:
    - Steering commands
    """
    
    def __init__(self):
        """Initialize the lane follower node with subscribers, publishers, and state variables"""
        # Initialize the ROS2 node with name "driver"
        super().__init__("driver")

        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )
        
        # Create publisher for steering commands
        self.steer_pub = self.create_publisher(
            Int64,              # Message type
            STEER_TOPIC,        # Topic name (fixed variable name)
            10                  # Queue size
        )

        self.path_segmenter = PathSegmentation('./sidewalk_segmentation/sidewalk_segmentation_model/model.onnx')
        self.lane_detector = LaneDetector()
        self.br = CvBridge()

        # State variables for edge detection and distance
        self.edge_distance = 0.0      # Current distance to lane edge
        self.edge_detected = True     # Whether an edge is currently detected
        
        # PID Controller state variables
        self.previous_error = 0.0     # Previous error for derivative calculation
        self.integral = 0.0           # Accumulated error for integral term
        self.previous_time = time.time()  # Previous timestamp for time delta calculation

    def image_callback(self, msg: Image):
        """
        Callback for Zed images
        """
        
        zed_image = self.br.imgmsg_to_cv2(msg)
        current_image = zed_image[:,:,:3]

        segmented_image = self.path_segmenter.segment_image(frame=current_image)
        final_image, vector_left, vector_right = self.lane_detector.process_single_image(current_image, segmented_image)

        cv2.line(final_image, (640, 0), (640, 720), (0, 0, 255), 4)

        # Only calculate steering if an edge is detected
        if vector_left and vector_right:  # Fixed variable reference (was edge_detected without self)
            # Calculate error: how far we are from target distance
            extended_x_intercept_left = self.extend_vector(vector_left)
            extended_x_intercept_right = self.extend_vector(vector_right)
            print(extended_x_intercept_left)
            print(extended_x_intercept_right)
            center_x = (extended_x_intercept_left + extended_x_intercept_right) / 2

            CTE = SETPOINT - center_x

            steer = self.PID(CTE)

            cv2.line(final_image, (int(center_x), 0), (int(center_x), 720), (0, 255, 0), 4)
            
            # Use PID controller to calculate steering correction [-100, 100]
            # Get image dimensions
            height, width = final_image.shape[:2]
            center_x, center_y = width // 2, height // 2

            # Map steer [-100, 100] to angle [-90, 90] degrees
            angle_degrees = (steer / 100.0) * 90.0

            # Convert to radians (note: positive angle goes clockwise in image coordinates)
            angle_radians = math.radians(angle_degrees)

            # Calculate end point of the line (length 100 pixels)
            line_length = 100
            end_x = int(center_x + line_length * math.sin(angle_radians))
            end_y = int(center_y - line_length * math.cos(angle_radians))  # Subtract because y increases downward

            # Draw the red steering line
            cv2.line(final_image, (int(center_x), int(center_y)), (end_x, end_y), (0, 255, 255), 3)

            print(f"CTE: {CTE}, Steer: {steer}")
        else:
            # No edge detected - don't steer (go straight)
            steer = 0
            print(f"No edge detected. Steer: 0")
        
        if SHOW_IMAGE and final_image is not None:
            cv2.imshow("Final Image", final_image)
            
            cv2.waitKey(1)
            
        # Publish the calculated steering command
        self.publish_steer(steer)
    
    def extend_vector(self, vector):
        # Reverse the vector to find where it intersects with y=0

        if (vector[0][0] - vector[1][0]) == 0:
            m = 99999
        else:
            m = (vector[0][1] - vector[1][1]) / (vector[0][0] - vector[1][0])
        
        b = vector[0][1] - vector[0][0] * m

        return (720-b)/m

    def PID(self, error: float):
        """
        PID Controller implementation
        
        Args:
            error: Current error (difference between actual and target distance)
            
        Returns:
            PID controller output (steering correction)
        """
        # Get current time and calculate time delta since last call
        current_time = time.time()
        dt = current_time - self.previous_time
        
        # Prevent division by zero if callbacks happen too quickly
        if dt <= 0.0:
            dt = 0.01  # Use small default time step
        
        # Proportional term: current error * proportional gain
        P = Kp * error
        
        # Integral term: accumulated error over time * integral gain
        self.integral += error * dt
        I = Ki * self.integral
        
        # Derivative term: rate of error change * derivative gain
        derivative = (error - self.previous_error) / dt
        D = Kd * derivative
        
        # Calculate final PID output by summing all three terms
        output = int(P + I + D)
        
        # Update state variables for next iteration
        self.previous_error = error
        self.previous_time = current_time

        if output > 100:
            output = 100
        elif output < -100:
            output = -100
        
        return output
    
    def publish_steer(self, steer):
        """
        Publish steering command to ROS2 topic
        
        Args:
            steer: Steering value to publish (will be converted to int)
        """
        # Create Int64 message
        msg = Int64()
        msg.data = -1 * steer  # Convert to integer as required by message type
        
        # Publish the message
        self.steer_pub.publish(msg)

def main():
    """
    Main function that initializes and runs the ROS2 node
    """
    # Initialize the ROS2 Python client library
    rclpy.init()
    
    # Create instance of our lane follower node
    lane_follower = LaneFollowerNode()
    
    try:
        # Start the ROS2 event loop - this will call our callbacks when messages arrive
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print('Shutting down due to keyboard interrupt')
        # Note: lane_follower.shutdown() method doesn't exist - this line will cause error
        # lane_follower.shutdown()  # Should be removed or implemented
    except Exception as e:
        # Handle any other unexpected errors
        print(f'Unexpected error: {str(e)}')
    finally:
        # Clean up resources
        lane_follower.destroy_node()  # Destroy the node
        rclpy.shutdown()              # Shutdown ROS2 client library

# Standard Python idiom - only run main() if this script is executed directly
if __name__ == '__main__':
    main()