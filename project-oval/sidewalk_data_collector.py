#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, MagneticField
from nmea_msgs.msg import Gpgga, Gprmc
from gps_msgs.msg import GPSFix
from std_msgs.msg import Int64, Float64MultiArray
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
import os
import json
import sensor_msgs.msg as sensor_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import PointField
from rclpy.serialization import serialize_message, deserialize_message
import csv
from sensor_msgs_py import point_cloud2

# Topic definitions
RTK_TOPIC = '/gpsfix'
ZED_IMU_DATA_TOPIC = "/zed/zed_node/imu/data"
ZED_LEFT_IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"
LIDAR_TOPIC = "/velodyne_points"

DATA_DIR = '/home/wolfwagen/ext-vol/ww_data_collection'

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.br = CvBridge()
        
        # Initialize data storage variables
        self.image = None
        self.lidar_data = None
        self.rtk_data = None
        self.imu_data = None
        self.zed_pose_data = None
        
        # Define sampling frequencies (Hz)
        self.frequencies = {
            'timestamp': 50,            # 50 Hz
            'zed_image': 10,             # 10 Hz - ZED Frames
            'lidar': 10,                # 10 Hz - LIDAR
            'rtk': 1,                   # 1 Hz - RTK GPS
            'zed_pose': 5,              # 5 Hz - ZED Pose estimation
            'zed_imu': 50,              # 50 Hz - ZED IMU
        }
        
        # Track last capture times for each data type
        self.last_capture_times = {key: 0.0 for key in self.frequencies.keys()}
        self.last_save_time = time.time()
        
        # Data buffers to store high-frequency data between saves
        self.data_buffers = {
            'timestamp': [],
            'imu': []
        }
        
        # The master save timer runs at the highest frequency
        self.save_interval = 1.0/50.0  # 50Hz = 0.02s
        self.metadata_list = []
        
        # Create dataset directories
        if not os.path.exists(DATA_DIR):
            os.makedirs(DATA_DIR)
        
        os.makedirs(os.path.join(DATA_DIR, 'images'), exist_ok=True)
        os.makedirs(os.path.join(DATA_DIR, 'lidar'), exist_ok=True)
        
        # Define ZED Pose topic - add this to your topic definitions if needed
        ZED_POSE_TOPIC = "/zed/zed_node/pose"
        
        # Create subscribers with appropriate queue sizes for high-frequency topics
        self.create_subscription(Image, ZED_LEFT_IMAGE_TOPIC, self.camera_callback, 10)
        self.create_subscription(Imu, ZED_IMU_DATA_TOPIC, self.imu_callback, 100)  # Higher queue for 50Hz
        self.create_subscription(PoseStamped, ZED_POSE_TOPIC, self.zed_pose_callback, 100)
        self.create_subscription(GPSFix, RTK_TOPIC, self.rtk_callback, 10)
        self.subscription = self.create_subscription(PointCloud2, LIDAR_TOPIC, self.lidar_callback, 10)

        # Create timer for data collection and saving - runs at master frequency (50Hz)
        self.create_timer(self.save_interval, self.process_data)
        
        # Create separate timer for JSON file saving - runs at 1Hz
        self.create_timer(1.0, self.save_json_metadata)
        
        # Track last update times for each sensor
        self.last_updates = {
            'image': None,
            'lidar': None,
            'rtk': None,
            'imu': None,
            'zed_pose': None,
        }
        
        # CSV files for different frequency data
        self.csv_files = {
            'high_freq': DATA_DIR + '/high_freq_data.csv',  # 50Hz data (IMU, timestamps)
            'med_freq': DATA_DIR + '/med_freq_data.csv',    # 5Hz data (ZED pose)
            'low_freq': DATA_DIR + '/low_freq_data.csv',     # 1Hz data (RTK)
            'sidewalk': DATA_DIR + '/sidewalk_data.csv'  # Sidewalk data
        }
        
        # Initialize CSV files with headers
        self.initialize_csv_files()
        
        # Maximum age of sensor data to be considered valid (in seconds)
        self.max_data_age = 0.5  # 500ms
        
        self.get_logger().info('Data Collector initialized')

    def camera_callback(self, data):
        self.image = self.br.imgmsg_to_cv2(data)
        self.image = self.image[:,:,:3]  # Ensure RGB format
        self.last_updates['image'] = time.time()
        self.get_logger().debug('Received camera image')

    def lidar_callback(self, msg):
        try:
            self.lidar_data = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
            if not self.lidar_data:
                self.get_logger().warn('Received empty point cloud.')
                return
    
        except Exception as e:
            self.get_logger().error(f'Error receiving point cloud: {e}')
    
        self.last_updates['lidar'] = time.time()
        self.get_logger().debug('Received LIDAR data')

    def write_pcd(self, filename, points):
        # points: list of (x, y, z, intensity)
        with open(filename, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z intensity\n')
            f.write('SIZE 4 4 4 4\n')
            f.write('TYPE F F F F\n')
            f.write('COUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(points)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(points)}\n')
            f.write('DATA ascii\n')
            for pt in points:
                f.write(f"{pt[0]} {pt[1]} {pt[2]} {pt[3]}\n")

    def cheap_imu_callback(self, msg):
        self.cheap_imu_data = {
            'angular_velocity': {
                'x': msg.data[0],
                'y': msg.data[1],
                'z': msg.data[2]
            },
            'linear_acceleration': {
                'x': msg.data[3],
                'y': msg.data[4],
                'z': msg.data[5]
            }
        }

        self.last_updates['cheap_imu'] = time.time()
        self.get_logger().debug('Received Cheap IMU data')
    
    def imu_callback(self, msg):
        self.imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        self.last_updates['imu'] = time.time()
        self.get_logger().debug('Received IMU data')

    def gps_qual_callback(self, msg):
        self.gps_qual = {
                'gps_sats': msg.num_sats,
                'gps_qual': msg.gps_qual
            }
        self.last_updates['gps_qual'] = time.time()
        self.get_logger().debug('Received gps qual')
    
    def imu_raw_callback(self, msg):
        self.imu_raw_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        self.last_updates['imu_raw'] = time.time()
        self.get_logger().debug('Received raw IMU data')

    def imu_cam_callback(self, msg):
        self.imu_cam_transform = {
            'translation': {
                'x': msg.transform.translation.x,
                'y': msg.transform.translation.y,
                'z': msg.transform.translation.z
            },
            'rotation': {
                'x': msg.transform.rotation.x,
                'y': msg.transform.rotation.y,
                'z': msg.transform.rotation.z,
                'w': msg.transform.rotation.w
            }
        }
        self.last_updates['imu_cam'] = time.time()
        self.get_logger().debug('Received IMU-camera transform')

    def imu_mag_callback(self, msg):
        self.imu_mag_data = {
            'magnetic_field': {
                'x': msg.magnetic_field.x,
                'y': msg.magnetic_field.y,
                'z': msg.magnetic_field.z
            }
        }
        self.last_updates['imu_mag'] = time.time()
        self.get_logger().debug('Received IMU magnetic field data')

    def rtk_callback(self, msg):
        if not np.isnan(msg.latitude) and not np.isnan(msg.longitude):
            self.rtk_data = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'track': msg.track,
                'rtk_sats': msg.status.satellites_used
            }
            self.last_updates['rtk'] = time.time()
            self.get_logger().debug('Received RTK GPS data')
        else:
            self.get_logger().warn('Received invalid RTK GPS data (NaN values)')

    def gps_callback(self, msg):
        lat = msg.lat
        lon = msg.lon
        
        if not np.isnan(lat) and not np.isnan(lon):
            self.gps_data = {
                'latitude': self.gps_converter(lat),
                'longitude': self.gps_converter(lon) * -1.0,  # Convert to standard format
                'track': msg.track
            }
            self.last_updates['gps'] = time.time()
            self.get_logger().debug('Received GPS data')
        else:
            self.get_logger().warn('Received invalid GPS data (NaN values)')

    def right_throttle_callback(self, msg):
        self.right_throttle = msg.data
        self.get_logger().debug(f'Received right throttle: {self.right_throttle}')

    def left_throttle_callback(self, msg):
        self.left_throttle = msg.data
        self.get_logger().debug(f'Received left throttle: {self.left_throttle}')

    def gps_converter(self, point):
        """Convert GPS coordinates from NMEA format to decimal degrees"""
        degrees = np.floor(point / 100)
        minutes = point - degrees * 100
        return degrees + minutes / 60
    
    def check_data_fresh(self):
        """Check if all sensor data is available and fresh"""
        current_time = time.time()
        
        # List of critical sensors that must be present
        critical_sensors = ['image', 'gps']
        
        for sensor in critical_sensors:
            last_update = self.last_updates.get(sensor)
            if last_update is None:
                self.get_logger().warn(f'No {sensor} data received yet')
                return False
            
            if current_time - last_update > self.max_data_age:
                self.get_logger().warn(f'{sensor} data is too old: {current_time - last_update:.2f}s')
                return False
        
        return True
    
    def initialize_csv_files(self):
        """Initialize all CSV files with appropriate headers"""
        # High frequency data (50Hz) - IMU and timestamps
        with open(self.csv_files['high_freq'], 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'zed_imu_orientation_x', 'zed_imu_orientation_y', 'zed_imu_orientation_z', 'zed_imu_orientation_w',
                'zed_imu_angular_velocity_x', 'zed_imu_angular_velocity_y', 'zed_imu_angular_velocity_z',
                'zed_imu_linear_acceleration_x', 'zed_imu_linear_acceleration_y', 'zed_imu_linear_acceleration_z'
            ])
        
        # Medium frequency data (5Hz) - ZED pose
        with open(self.csv_files['med_freq'], 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'zed_pose_position_x', 'zed_pose_position_y', 'zed_pose_position_z',
                'zed_pose_orientation_x', 'zed_pose_orientation_y', 'zed_pose_orientation_z', 'zed_pose_orientation_w'
            ])
        
        # Low frequency data (1Hz) - RTK
        with open(self.csv_files['low_freq'], 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'rtk_lat', 'rtk_lon', 'rtk_alt', 'rtk_heading', 'rtk_sats',
            ])

         # Sidewalk data (10Hz) - Images, Lidar (geotagged images and lidar pcds)
        with open(self.csv_files['sidewalk'], 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'rtk_lat', 'rtk_lon', 'rtk_alt', 'rtk_heading', 'rtk_sats','image_file', 'lidar_file'
            ])
        
        # Create master JSON file for indexing
        with open(DATA_DIR + '/data_index.json', 'w') as f:
            json.dump({
                'high_freq_file': self.csv_files['high_freq'],
                'med_freq_file': self.csv_files['med_freq'],
                'low_freq_file': self.csv_files['low_freq'],
                'frequencies': self.frequencies
            }, f, indent=2)
    
    def zed_pose_callback(self, msg):
        """Callback for ZED pose data"""
        self.zed_pose_data = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.last_updates['zed_pose'] = time.time()
        self.get_logger().debug('Received ZED pose data')
    
    def process_data(self):
        """Process and save data at appropriate frequencies"""
        current_time = time.time()
        timestamp = self.get_clock().now().to_msg()
        timestamp_str = f'{timestamp.sec}.{timestamp.nanosec}'
        
        # Always collect timestamp data at highest frequency
        self.data_buffers['timestamp'].append(timestamp_str)
        
        # Process high-frequency data (50Hz)
        should_save_high_freq = self._should_capture('timestamp', current_time)
        if should_save_high_freq and self.imu_data:
            # Save IMU data at 50Hz
            self._save_high_freq_data(timestamp_str)

        # Save sidewalk data at 10Hz
        should_save_sidewalk_data = self._should_capture('zed_image', current_time)
        if should_save_sidewalk_data:
            self._save_sidewalk_data(timestamp_str)
            
        # Process medium-frequency data (5Hz)
        if self._should_capture('zed_pose', current_time) and self.zed_pose_data:
            self._save_med_freq_data(timestamp_str)
        
        # Process low-frequency data (1Hz)
        if self._should_capture('rtk', current_time):
            self._save_low_freq_data(timestamp_str)
            
    def _should_capture(self, data_type, current_time):
        """Check if it's time to capture data based on frequency"""
        interval = 1.0 / self.frequencies[data_type]
        if current_time - self.last_capture_times[data_type] >= interval:
            self.last_capture_times[data_type] = current_time
            return True
        return False
    
    def _save_high_freq_data(self, timestamp_str):
        """Save high-frequency data (50Hz)"""
        if not self.imu_data:
            return
            
        # Prepare data for CSV
        csv_row = [timestamp_str]
        
        # Add IMU data
        csv_row.extend([
            self.imu_data['orientation']['x'],
            self.imu_data['orientation']['y'],
            self.imu_data['orientation']['z'],
            self.imu_data['orientation']['w'],
            self.imu_data['angular_velocity']['x'],
            self.imu_data['angular_velocity']['y'],
            self.imu_data['angular_velocity']['z'],
            self.imu_data['linear_acceleration']['x'],
            self.imu_data['linear_acceleration']['y'],
            self.imu_data['linear_acceleration']['z']
        ])
        
        # Save to high-frequency CSV
        with open(self.csv_files['high_freq'], 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_row)
        
        # Buffer for metadata
        high_freq_data = {
            'timestamp': timestamp_str,
            'imu': self.imu_data,
        }
        
        # Add to metadata list
        self.metadata_list.append(high_freq_data)
        self.get_logger().info(f'Saved high-frequency data point at {timestamp_str}')
    
    def _save_med_freq_data(self, timestamp_str):
        """Save medium-frequency data (5Hz)"""
        if not self.zed_pose_data:
            return
            
        # Prepare data for CSV
        csv_row = [timestamp_str]
        
        # Add ZED pose data
        csv_row.extend([
            self.zed_pose_data['position']['x'],
            self.zed_pose_data['position']['y'],
            self.zed_pose_data['position']['z'],
            self.zed_pose_data['orientation']['x'],
            self.zed_pose_data['orientation']['y'],
            self.zed_pose_data['orientation']['z'],
            self.zed_pose_data['orientation']['w']
        ])
        
        # Save to medium-frequency CSV
        with open(self.csv_files['med_freq'], 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_row)
        
        # Create metadata entry
        med_freq_data = {
            'timestamp': timestamp_str,
            'zed_pose': self.zed_pose_data
        }
        
        # Add to metadata list
        self.metadata_list.append(med_freq_data)
    
    def _save_low_freq_data(self, timestamp_str):
        """Save low-frequency data (1Hz)"""
        # Prepare data for CSV
        csv_row = [timestamp_str]
        
        # Add RTK data
        if self.rtk_data:
            csv_row.extend([
                self.rtk_data.get('latitude', 0),
                self.rtk_data.get('longitude', 0),
                self.rtk_data.get('altitude', 0),
                self.rtk_data.get('track', 0),
                self.rtk_data.get('rtk_sats', 0)
            ])
        else:
            csv_row.extend([0, 0, 0, 0])
        
        # Save to low-frequency CSV
        with open(self.csv_files['low_freq'], 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_row)
        
        # Create metadata entry
        low_freq_data = {
            'timestamp': timestamp_str,
            'rtk': self.rtk_data
        }
        
        # Add to metadata list
        self.metadata_list.append(low_freq_data)
        
        self.get_logger().info(f'Saved low-frequency data point at {timestamp_str}')

    def _save_sidewalk_data(self, timestamp_str):
        """Save low-frequency data (1Hz)"""
         # Save image
        image_filename = DATA_DIR + f'/images/image_{timestamp_str.replace(".", "_")}.jpg'
        if self.image is not None:
            cv2.imwrite(image_filename, self.image)
            self.get_logger().info(f'Saved image to {image_filename}')
        else:
            image_filename = "none"
            self.get_logger().warn('No image data to save')

        # Save LIDAR data
        lidar_filename = DATA_DIR + f'/lidar/lidar_{timestamp_str.replace(".", "_")}.pcd'
        if self.lidar_data is not None:
            self.write_pcd(lidar_filename, self.lidar_data)
            self.get_logger().info(f'Saved LIDAR data to {lidar_filename}')
        else:
            lidar_filename = "none"
            self.get_logger().warn('No LIDAR data to save')
        
        # Prepare data for CSV
        csv_row = [timestamp_str]
        
        # Add RTK data
        if self.rtk_data:
            csv_row.extend([
                self.rtk_data.get('latitude', 0),
                self.rtk_data.get('longitude', 0),
                self.rtk_data.get('altitude', 0),
                self.rtk_data.get('track', 0),
                self.rtk_data.get('rtk_sats', 0)
            ])
        else:
            csv_row.extend([0, 0, 0, 0])

        csv_row.append(image_filename)
        csv_row.append(lidar_filename)
        
        # Save to low-frequency CSV
        with open(self.csv_files['sidewalk'], 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_row)
        
        # Create metadata entry
        sidewalk_data = {
            'timestamp': timestamp_str,
            'rtk': self.rtk_data
        }
        
        # Add to metadata list
        self.metadata_list.append(sidewalk_data)
        
        self.get_logger().info(f'Saved sidewalk data point at {timestamp_str}')
    
    def save_json_metadata(self):
        """Save metadata to JSON file - runs at 1Hz"""
        # Save the latest full dataset to a JSON file
        latest_data = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'image': self.image is not None,
            'rtk': self.rtk_data,
            'imu': self.imu_data,
            'zed_pose': self.zed_pose_data,
            'lidar': self.lidar_data is not None,
            'data_files': self.csv_files
        }
        
        # Save latest data for monitoring
        with open('dataset/latest_data.json', 'w') as f:
            json.dump(latest_data, f, indent=2)
            
        self.get_logger().info('Updated metadata JSON file')
    
    def shutdown(self):
        """Clean shutdown with metadata saving"""
        # Save all collected metadata to JSON file
        with open('dataset/metadata.json', 'w') as f:
            json.dump(self.metadata_list, f, indent=2)
            
        # Save summary statistics
        summary = {
            'total_data_points': {
                'high_freq': len(self.data_buffers['imu']),
                'med_freq': sum(1 for item in self.metadata_list if 'zed_pose' in item),
                'low_freq': sum(1 for item in self.metadata_list if 'rtk' in item),
                'sidewalk': len(1 for item in self.metadata_list if 'image' in item)
            },
            'collection_duration_seconds': time.time() - self.last_save_time,
            'data_files': self.csv_files,
            'frequencies': self.frequencies
        }
        
        with open(DATA_DIR + '/collection_summary.json', 'w') as f:
            json.dump(summary, f, indent=2)
            
        self.get_logger().info(f'Saved metadata and collection summary')

def main():
    rclpy.init()
    collector = DataCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Shutting down due to keyboard interrupt')
        collector.shutdown()
    except Exception as e:
        collector.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()