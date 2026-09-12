#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
import threading
import csv
import os
from datetime import datetime
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library

class ZedImuLogger(Node):
    def __init__(self):
        super().__init__("zed_imu_node")
        
        # Initialize timestamp and folders
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_timestamp = timestamp
        
        # Create images folder
        self.images_folder = f"zed_images_{timestamp}"
        os.makedirs(self.images_folder, exist_ok=True)
        
        # Initialize CSV file
        self.csv_filename = f"zed_imu_data_{timestamp}.csv"
        self.csv_file = None
        self.csv_writer = None
        self.frame = None
        self.image_counter = 0
        self.setup_csv()
        self.bridge = CvBridge()
        
        # Subscription to IMU topic
        self.create_subscription(Imu, "/zed/zed_node/imu/data", self.zed_imu_callback, 10)
        self.create_subscription(Image, "/zed/zed_node/left/image_rect_color", self.zed_image_callback, 10)
        
        self.get_logger().info(f"Starting IMU data logging to: {self.csv_filename}")
        self.get_logger().info(f"Images will be saved to: {self.images_folder}")

    def zed_image_callback(self, msg):
        """Callback to receive and store the latest image"""
        try:
            # Convert ROS image message to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

    def save_current_image(self):
        """Save the current frame to disk and return the filename"""
        if self.frame is not None:
            # Create filename with timestamp and counter
            current_time = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # microseconds to milliseconds
            self.image_counter += 1
            image_filename = f"image_{current_time}_{self.image_counter:06d}.jpg"
            image_path = os.path.join(self.images_folder, image_filename)
            
            # Save the image
            success = cv.imwrite(image_path, self.frame)
            if success:
                self.get_logger().info(f"Saved image: {image_filename}")
                return image_filename
            else:
                self.get_logger().error(f"Failed to save image: {image_filename}")
                return ""
        else:
            self.get_logger().warn("No image available to save")
            return ""

    def setup_csv(self):
        """Initialize the CSV file with headers"""
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write CSV headers
        headers = [
            # Timestamp
            'timestamp_sec', 'timestamp_nanosec', 'frame_id', 'image_filename',
            # Orientation (quaternion)
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            # Orientation covariance (9 elements)
            'ori_cov_0', 'ori_cov_1', 'ori_cov_2', 'ori_cov_3', 'ori_cov_4',
            'ori_cov_5', 'ori_cov_6', 'ori_cov_7', 'ori_cov_8',
            # Angular velocity
            'angular_vel_x', 'angular_vel_y', 'angular_vel_z',
            # Angular velocity covariance (9 elements)
            'ang_vel_cov_0', 'ang_vel_cov_1', 'ang_vel_cov_2', 'ang_vel_cov_3',
            'ang_vel_cov_4', 'ang_vel_cov_5', 'ang_vel_cov_6', 'ang_vel_cov_7', 'ang_vel_cov_8',
            # Linear acceleration
            'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
            # Linear acceleration covariance (9 elements)
            'lin_accel_cov_0', 'lin_accel_cov_1', 'lin_accel_cov_2', 'lin_accel_cov_3',
            'lin_accel_cov_4', 'lin_accel_cov_5', 'lin_accel_cov_6', 'lin_accel_cov_7', 'lin_accel_cov_8'
        ]
        
        self.csv_writer.writerow(headers)
        self.csv_file.flush()

    def zed_imu_callback(self, data):
        """Callback function to process and log IMU data"""
        try:
            # Save the current image (if available)
            image_filename = self.save_current_image()
            
            # Extract data from the IMU message
            row = [
                # Timestamp
                data.header.stamp.sec,
                data.header.stamp.nanosec,
                data.header.frame_id,
                image_filename,  # Add the saved image filename
                # Orientation (quaternion)
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
                # Orientation covariance
                *data.orientation_covariance,
                # Angular velocity
                data.angular_velocity.x,
                data.angular_velocity.y,
                data.angular_velocity.z,
                # Angular velocity covariance
                *data.angular_velocity_covariance,
                # Linear acceleration
                data.linear_acceleration.x,
                data.linear_acceleration.y,
                data.linear_acceleration.z,
                # Linear acceleration covariance
                *data.linear_acceleration_covariance
            ]
            
            # Write to CSV
            self.csv_writer.writerow(row)
            self.csv_file.flush()  # Ensure data is written immediately
            
            # Optional: Print some basic info (remove if too verbose)
            self.get_logger().info(f"Logged IMU data - Accel: [{data.linear_acceleration.x:.3f}, "
                                 f"{data.linear_acceleration.y:.3f}, {data.linear_acceleration.z:.3f}]")
            
        except Exception as e:
            self.get_logger().error(f"Error logging IMU data: {str(e)}")

    def cleanup(self):
        """Close the CSV file properly"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed: {self.csv_filename}")
            self.get_logger().info(f"Total images saved: {self.image_counter}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create the node
        imu_logger = ZedImuLogger()
        
        # Spin the node
        rclpy.spin(imu_logger)
        
    except KeyboardInterrupt:
        print("Ctrl+C captured, ending...")
        
    finally:
        # Cleanup
        if 'imu_logger' in locals():
            imu_logger.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()