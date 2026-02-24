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
import numpy as np

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
STEERING_TOPIC = '/steering'  # Output: steering commands
SEGMENTATION_CONTROL_VISUALIZATION_TOPIC = '/segmentation/control_visualization'  # Visualization output

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

        # Create publisher for steering commands
        self.steer_pub = self.create_publisher(
            Int64,
            STEERING_TOPIC,
            10
        )

        # Publisher node for visualization of lane detection
        self.visualization_pub = self.create_publisher(
            Image, 
            SEGMENTATION_CONTROL_VISUALIZATION_TOPIC, 
            10
        )

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
        self.edge_detected = False  # Track if lanes are detected
        
        # PID Controller state variables
        self.previous_error = 0.0     # Previous error for derivative calculation
        self.integral = 0.0           # Accumulated error for integral term
        self.previous_time = time.time()  # Previous timestamp for time delta calculation

    def visualize_lanes(self, left_lane, right_lane, path_center):
        """
        Visualize detected lanes and path center on the mask image
        
        Args:
            left_lane: Left lane edge points (x, y coordinates)
            right_lane: Right lane edge points (x, y coordinates)
            path_center: Center point between lanes (x, y coordinates)
        """
        # Create a color copy of the mask for visualization
        if len(self.mask_image.shape) == 2:
            # Convert grayscale to BGR for color visualization
            vis_image = cv2.cvtColor(self.mask_image, cv2.COLOR_GRAY2BGR)
        else:
            vis_image = self.mask_image.copy()
        
        # Draw left lane edge in green with lines connecting points
        if left_lane is not None and len(left_lane) > 0:
            # Draw circles at each point
            for point in left_lane:
                cv2.circle(vis_image, (int(point[0]), int(point[1])), 3, (0, 255, 0), -1)
            
            # Draw lines connecting the points
            if len(left_lane) > 1:
                pts = np.array([[int(p[0]), int(p[1])] for p in left_lane], np.int32)
                cv2.polylines(vis_image, [pts], False, (0, 255, 0), 2)
        
        # Draw right lane edge in blue with lines connecting points
        if right_lane is not None and len(right_lane) > 0:
            # Draw circles at each point
            for point in right_lane:
                cv2.circle(vis_image, (int(point[0]), int(point[1])), 3, (255, 0, 0), -1)
            
            # Draw lines connecting the points
            if len(right_lane) > 1:
                pts = np.array([[int(p[0]), int(p[1])] for p in right_lane], np.int32)
                cv2.polylines(vis_image, [pts], False, (255, 0, 0), 2)
        
        # Draw path center in red with larger circle
        if path_center is not None:
            cv2.circle(vis_image, (int(path_center[0]), int(path_center[1])), 8, (0, 0, 255), -1)
            
            # Draw a vertical line at image center for reference (cyan)
            image_center_x = int(self.mask_image.shape[1] / 2)
            cv2.line(vis_image, (image_center_x, 0), (image_center_x, vis_image.shape[0]), (255, 255, 0), 2)
            
            # Draw line from image center to path center (magenta)
            cv2.line(vis_image, (image_center_x, int(path_center[1])), 
                    (int(path_center[0]), int(path_center[1])), (255, 0, 255), 2)
        
        # Calculate and display information
        image_center_x = self.mask_image.shape[1] / 2
        distance = 0
        error = 0
        
        if path_center is not None:
            distance = abs(image_center_x - path_center[0])
            error = distance - target_distance
        
        # Calculate current steering angle using PID (for display only)
        steer_angle = 0
        if self.edge_detected:
            # Get current time and calculate time delta
            current_time = time.time()
            dt = current_time - self.previous_time
            if dt <= 0.0:
                dt = 0.01
            
            # Calculate PID components
            P = Kp * error
            I = Ki * self.integral
            derivative = (error - self.previous_error) / dt
            D = Kd * derivative
            steer_angle = int(P + I + D)
        
        # Add text overlay with information
        cv2.putText(vis_image, f"Edge Detected: {self.edge_detected}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        cv2.putText(vis_image, f"Distance: {distance:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        cv2.putText(vis_image, f"Steering Angle: {steer_angle}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # # Display the visualization
        cv2.imshow("Lane Detection", vis_image)
        cv2.waitKey(1)  # Required for OpenCV window updates
        return vis_image

    def segmentation_mask_callback(self, msg: Image):
        """
        Callback for edge detection status messages
        
        Args:
            msg: ROS2 Image message for segmentation mask
        """
        # Convert ROS Image to OpenCV format (keep as grayscale)
        self.mask_image = self.bridge.imgmsg_to_cv2(msg)
        print(f"Mask shape: {self.mask_image.shape}")
        
        # Detect lanes
        lanes_detected = self.lane_detector.detect_lanes(self.mask_image)

        if lanes_detected is not None:
            self.edge_detected = True
            left_lane = lanes_detected['left_lane']
            right_lane = lanes_detected['right_lane']
            path_center = lanes_detected['path_center']
            
            # Extract x-coordinate from path_center tuple
            lane_center_x = path_center[0]
            
            # Calculate distance from image center
            image_center_x = self.mask_image.shape[1] / 2
            distance = abs(image_center_x - lane_center_x)
            
            # Calculate steering based on distance
            self.find_steering(distance)
            
            # Visualize the detected lanes
            viz = self.visualize_lanes(left_lane, right_lane, path_center)
            # Publish visualization image
            viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
            self.visualization_pub.publish(viz_msg)
        else:
            self.edge_detected = False
            self.find_steering(0)  # No lanes detected

    def find_steering(self, distance: float):
        """
        Main callback that processes edge distance and calculates steering
        
        Args:
            distance: Distance from target position
        """
        # Only calculate steering if an edge is detected
        if self.edge_detected:
            # Calculate error: how far we are from target distance
            error = distance - target_distance
            
            # Use PID controller to calculate steering correction
            steer = self.PID(error)
            print(f"Edge detected. Distance: {distance:.2f}, Error: {error:.2f}, Steer: {steer}")
        else:
            # No edge detected - don't steer (go straight)
            steer = 0
            print("No edge detected. Steer: 0")
            
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
    print("Working...")
    # Create instance of our lane follower node
    lane_follower = LaneFollowerNode()
    
    try:
        # Start the ROS2 event loop - this will call our callbacks when messages arrive
        rclpy.spin(lane_follower)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print('Shutting down due to keyboard interrupt')
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