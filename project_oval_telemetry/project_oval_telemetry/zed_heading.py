#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import numpy as np
from collections import deque

class CompassNode(Node):
    def __init__(self):
        super().__init__('compass_node')
        
        # Declare parameters
        self.declare_parameter('magnetic_declination', -10.5)  # Raleigh, NC default
        self.declare_parameter('use_sensor_fusion', True)
        self.declare_parameter('calibration_samples', 100)
        self.declare_parameter('moving_average_window', 10)
        
        # Get parameters
        self.magnetic_declination = self.get_parameter('magnetic_declination').value
        self.use_sensor_fusion = self.get_parameter('use_sensor_fusion').value
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.window_size = self.get_parameter('moving_average_window').value
        
        # Calibration variables
        self.is_calibrating = True
        self.calibration_data = []
        self.mag_offset = np.array([0.0, 0.0, 0.0])
        self.mag_scale = np.array([1.0, 1.0, 1.0])
        
        # Moving average filter
        self.heading_history = deque(maxlen=self.window_size)
        
        # Latest sensor data
        self.latest_mag = None
        self.latest_imu = None
        self.latest_pose = None
        
        # Subscribe to multiple topics
        self.mag_sub = self.create_subscription(
            MagneticField,
            '/zed/zed_node/imu/mag',
            self.mag_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/zed/zed_node/pose_with_covariance',
            self.pose_callback,
            10
        )
        
        # Publisher for compass heading
        self.create_timer(0.1, self.publish_compass)  # 10 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Enhanced Compass Node Started')
        self.get_logger().info(f'Magnetic Declination: {self.magnetic_declination:.2f}°')
        self.get_logger().info(f'Sensor Fusion: {"Enabled" if self.use_sensor_fusion else "Disabled"}')
        self.get_logger().info('Starting calibration...')
        self.get_logger().info('=' * 60)
    
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        This is a manual implementation without tf_transformations
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def mag_callback(self, msg):
        """Process magnetometer data"""
        self.latest_mag = msg
        
        # Collect calibration data
        if self.is_calibrating and len(self.calibration_data) < self.calibration_samples:
            self.calibration_data.append([
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z
            ])
            
            if len(self.calibration_data) >= self.calibration_samples:
                self.perform_calibration()
    
    def imu_callback(self, msg):
        """Process IMU data with orientation"""
        self.latest_imu = msg
    
    def pose_callback(self, msg):
        """Process pose data"""
        self.latest_pose = msg
    
    def perform_calibration(self):
        """Perform hard-iron and soft-iron calibration"""
        data = np.array(self.calibration_data)
        
        # Calculate hard-iron offsets (center of the ellipsoid)
        self.mag_offset = np.mean(data, axis=0)
        
        # Calculate soft-iron scaling (simplified - assumes aligned ellipsoid)
        centered_data = data - self.mag_offset
        ranges = np.max(centered_data, axis=0) - np.min(centered_data, axis=0)
        avg_range = np.mean(ranges)
        
        # Avoid division by zero
        for i in range(3):
            if ranges[i] > 0:
                self.mag_scale[i] = avg_range / ranges[i]
            else:
                self.mag_scale[i] = 1.0
        
        self.is_calibrating = False
        self.get_logger().info('Calibration completed!')
        self.get_logger().info(f'Offsets: X={self.mag_offset[0]:.6e}, Y={self.mag_offset[1]:.6e}, Z={self.mag_offset[2]:.6e}')
        self.get_logger().info(f'Scales: X={self.mag_scale[0]:.3f}, Y={self.mag_scale[1]:.3f}, Z={self.mag_scale[2]:.3f}')
    
    def calculate_tilt_compensated_heading(self, mag, accel):
        """Calculate tilt-compensated magnetic heading"""
        # Normalize acceleration vector
        accel_norm = accel / np.linalg.norm(accel)
        
        # Calculate pitch and roll from accelerometer
        pitch = math.atan2(-accel_norm[0], math.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
        roll = math.atan2(accel_norm[1], accel_norm[2])
        
        # Tilt compensation
        mag_x = mag[0] * math.cos(pitch) + mag[1] * math.sin(roll) * math.sin(pitch) + mag[2] * math.cos(roll) * math.sin(pitch)
        mag_y = mag[1] * math.cos(roll) - mag[2] * math.sin(roll)
        
        # Calculate heading
        heading = math.atan2(-mag_y, mag_x)  # Negative mag_y for correct orientation
        heading_deg = math.degrees(heading)
        
        # Normalize to 0-360
        if heading_deg < 0:
            heading_deg += 360
            
        return heading_deg, pitch, roll
    
    def get_heading_from_quaternion(self, q):
        """Extract yaw/heading from quaternion"""
        # Convert quaternion to Euler angles using manual implementation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        heading_deg = math.degrees(yaw)
        # Normalize to 0-360
        if heading_deg < 0:
            heading_deg += 360
            
        return heading_deg
    
    def apply_moving_average(self, heading):
        """Apply circular moving average to heading"""
        self.heading_history.append(heading)
        
        if len(self.heading_history) < 2:
            return heading
        
        # Convert to unit vectors for proper averaging
        vectors = []
        for h in self.heading_history:
            rad = math.radians(h)
            vectors.append([math.cos(rad), math.sin(rad)])
        
        # Average the vectors
        avg_vector = np.mean(vectors, axis=0)
        
        # Convert back to angle
        avg_heading = math.degrees(math.atan2(avg_vector[1], avg_vector[0]))
        if avg_heading < 0:
            avg_heading += 360
            
        return avg_heading
    
    def publish_compass(self):
        """Main compass calculation and publishing"""
        if self.is_calibrating:
            remaining = self.calibration_samples - len(self.calibration_data)
            if remaining > 0 and len(self.calibration_data) % 10 == 0:
                self.get_logger().info(f'Calibrating... {remaining} samples remaining')
            return
        
        if self.latest_mag is None:
            return
        
        # Extract and calibrate magnetometer data
        mag_raw = np.array([
            self.latest_mag.magnetic_field.x,
            self.latest_mag.magnetic_field.y,
            self.latest_mag.magnetic_field.z
        ])
        
        # Apply calibration
        mag_calibrated = (mag_raw - self.mag_offset) * self.mag_scale
        
        # Calculate magnitude
        magnitude = np.linalg.norm(mag_calibrated)
        magnitude_uT = magnitude * 1e6
        
        # Get heading based on available sensors
        magnetic_heading = None
        heading_source = "Unknown"
        pitch_deg = 0
        roll_deg = 0
        
        if self.use_sensor_fusion and self.latest_imu is not None:
            # Method 1: Use IMU orientation if available
            q = self.latest_imu.orientation
            if (q.w != 0 or q.x != 0 or q.y != 0 or q.z != 0):
                # Check if quaternion is valid (normalized)
                quat_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
                if 0.9 < quat_norm < 1.1:  # Allow some tolerance
                    magnetic_heading = self.get_heading_from_quaternion(q)
                    heading_source = "IMU Quaternion"
                    # Also get pitch and roll for display
                    roll, pitch, _ = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
                    pitch_deg = math.degrees(pitch)
                    roll_deg = math.degrees(roll)
            
            # Method 2: Tilt-compensated magnetometer
            if magnetic_heading is None:
                accel = np.array([
                    self.latest_imu.linear_acceleration.x,
                    self.latest_imu.linear_acceleration.y,
                    self.latest_imu.linear_acceleration.z
                ])
                accel_norm = np.linalg.norm(accel)
                if accel_norm > 0.1:  # Make sure we have valid acceleration data
                    magnetic_heading, pitch, roll = self.calculate_tilt_compensated_heading(mag_calibrated, accel)
                    heading_source = "Tilt-Compensated Mag"
                    pitch_deg = math.degrees(pitch)
                    roll_deg = math.degrees(roll)
        
        # Method 3: Raw magnetometer (fallback)
        if magnetic_heading is None:
            # Simple 2D heading from magnetometer (assuming X=North, Y=East)
            magnetic_heading = math.degrees(math.atan2(-mag_calibrated[1], mag_calibrated[0]))
            if magnetic_heading < 0:
                magnetic_heading += 360
            heading_source = "Raw Magnetometer"
        
        # Apply moving average filter
        magnetic_heading_filtered = self.apply_moving_average(magnetic_heading)
        
        # Apply magnetic declination
        true_heading = (magnetic_heading_filtered + self.magnetic_declination) % 360
        
        # Get compass direction
        compass_dir = self.heading_to_compass(true_heading)
        
        # Evaluate field strength
        if 20 < magnitude_uT < 70:
            field_status = "Normal"
        elif magnitude_uT < 20:
            field_status = "Weak"
        else:
            field_status = "Strong"
        
        # Calculate heading confidence based on field strength and sensor status
        confidence = self.calculate_heading_confidence(magnitude_uT, heading_source)
        
        # Output results
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'COMPASS READING:')
        self.get_logger().info(f'  True Heading:     {true_heading:6.1f}° {compass_dir}')
        self.get_logger().info(f'  Magnetic Heading: {magnetic_heading_filtered:6.1f}°')
        self.get_logger().info(f'  Declination:      {self.magnetic_declination:+6.1f}°')
        self.get_logger().info(f'  Source:           {heading_source}')
        self.get_logger().info(f'  Confidence:       {confidence}')
        self.get_logger().info(f'  Tilt: Pitch={pitch_deg:+5.1f}° Roll={roll_deg:+5.1f}°')
        self.get_logger().info(f'  Field Strength:   {magnitude_uT:.1f} µT ({field_status})')
        
        # If we have pose data, compare with visual odometry
        if self.latest_pose is not None:
            pose_heading = self.get_heading_from_quaternion(
                self.latest_pose.pose.pose.orientation
            )
            diff = abs(pose_heading - true_heading)
            if diff > 180:
                diff = 360 - diff
            self.get_logger().info(f'  Visual Heading:   {pose_heading:6.1f}° (diff: {diff:.1f}°)')
            
            # Warn if difference is large
            if diff > 30:
                self.get_logger().warn('Large difference between magnetic and visual heading - possible interference!')
    
    def calculate_heading_confidence(self, field_strength_uT, source):
        """Calculate confidence level of heading"""
        confidence_score = 100
        
        # Reduce confidence based on field strength
        if field_strength_uT < 20 or field_strength_uT > 70:
            confidence_score -= 30
        elif field_strength_uT < 25 or field_strength_uT > 65:
            confidence_score -= 15
        
        # Adjust based on source
        if source == "IMU Quaternion":
            confidence_score += 10
        elif source == "Raw Magnetometer":
            confidence_score -= 10
        
        # Convert to text
        if confidence_score >= 80:
            return "High"
        elif confidence_score >= 60:
            return "Medium"
        else:
            return "Low"
    
    def heading_to_compass(self, heading):
        """Convert heading to 16-point compass direction"""
        directions = [
            'N', 'NNE', 'NE', 'ENE',
            'E', 'ESE', 'SE', 'SSE',
            'S', 'SSW', 'SW', 'WSW',
            'W', 'WNW', 'NW', 'NNW'
        ]
        index = round(heading / 22.5) % 16
        return directions[index]

def main(args=None):
    rclpy.init(args=args)
    
    compass_node = CompassNode()
    
    try:
        rclpy.spin(compass_node)
    except KeyboardInterrupt:
        pass
    finally:
        compass_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()