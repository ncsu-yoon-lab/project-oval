#!/usr/bin/env python

"""
ROS2 Lane Following Node with PID Control

This node subscribes to edge detection data and publishes steering commands
to keep the vehicle at a target distance from lane edges using PID control.
"""

# Import required ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from lib.lane_detector import LaneDetector

# Other libraries
import time

# PID Controller Coefficients
Kp = 0.3    # Proportional gain - how aggressively to respond to current error
Kd = 0.05   # Derivative gain - how much to dampen based on rate of error change
Ki = 0      # Integral gain - how much to correct for accumulated past errors

# Target distance from lane edge
target_distance = 2.0

# ROS2 Topic Names
SEGMENTATION_TOPIC = '/segmentation/mask'  # Input: distance to lane edge

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

        # Create subscribers to listen for edge detection data
        self.create_subscription(
            Image,                        # Message type
            SEGMENTATION_TOPIC,           # Topic name
            self.segmentation_mask_callback,   # Callback function
            10                             # Queue size
        )

        # State variables for edge detection and distance
        self.bridge = CvBridge()
        self.mask_image = None
        self.lane_detector = LaneDetector()
        
        # PID Controller state variables
        self.previous_error = 0.0     # Previous error for derivative calculation
        self.integral = 0.0           # Accumulated error for integral term
        self.previous_time = time.time()  # Previous timestamp for time delta calculation

    def segmentation_mask_callback(self, msg: Image):
        """
        Callback for edge detection status messages
        
        Args:
            msg: ROS2 Image message for segmentation mask
        """
        self.mask_image = self.bridge.imgmsg_to_cv2(msg)


        lanes_detected = self.lane_detector.detect_lanes(self.mask_image)

        if lanes_detected is not None:
            left_lane = lanes_detected['left_lane']
            right_lane = lanes_detected['right_lane']
            lane_center = lanes_detected['path_center']

            distance = abs((self.mask_image.shape[0] / 2) - lane_center)

            self.find_steering(self, distance)

        cv2.imshow()
            
            

    def find_steering(self, distance: float):
        """
        Main callback that processes edge distance and calculates steering
        
        Args:
            msg: ROS2 message containing distance to lane edge
        """

        # Only calculate steering if an edge is detected
        if self.edge_detected:  # Fixed variable reference (was edge_detected without self)
            # Calculate error: how far we are from target distance
            error = distance - target_distance
            
            # Use PID controller to calculate steering correction
            steer = self.PID(error)
            print(f"Edge detected. Distance: {distance}, Error: {error}, Steer: {steer}")
        else:
            # No edge detected - don't steer (go straight)
            steer = 0
            print(f"No edge detected. Steer: 0")
            
        # Publish the calculated steering command
        self.publish_steer(steer)

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
        
        return output
    
    def publish_steer(self, steer: int):
        """
        Publish steering command to ROS2 topic
        
        Args:
            steer: Steering value to publish (will be converted to int)
        """
        # Create Int64 message
        msg = Int64()
        msg.data = steer  # Convert to integer as required by message type
        
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