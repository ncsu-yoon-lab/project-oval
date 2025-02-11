#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nmea_msgs.msg import Gpgga, Gprmc
from std_msgs.msg import Int64
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np
import os
import json
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointField
from rclpy.serialization import serialize_message, deserialize_message
import csv

IMAGE_TOPIC = "/zed/zed_node/left/image_rect_color"
VELODYNE_TOPIC = "/velodyne_points"
GPS_TOPIC = '/gps/gprmc'

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.br = CvBridge()
        self.image = None
        self.scan = None
        self.gps_data = None
        self.last_save_time = time.time()
        self.save_interval = 1.0/2.0  # 1Hz
        self.metadata_list = []
        
        # Create dataset directories
        if not os.path.exists('dataset'):
            os.makedirs('dataset')
            os.makedirs('dataset/scans')
            os.makedirs('dataset/images')
        
        # Create CSV file for GPS data
        self.csv_filename = 'dataset/gps_data.csv'
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'track', 'scan_file', 'image_file', 'left_throttle', 'right_throttle'])
        
        # Create subscribers
        self.create_subscription(Image, IMAGE_TOPIC, self.camera_callback, 10)
        self.create_subscription(PointCloud2, VELODYNE_TOPIC, self.velodyne_callback, 10)
        self.create_subscription(Gprmc, GPS_TOPIC, self.gps_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/steer', self.right_throttle_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/throttle', self.left_throttle_callback, 10)
        
        # Create timer for data saving
        self.create_timer(self.save_interval, self.save_data)
        
        # Track last update times for each sensor
        self.last_updates = {
            'image': None,
            'scan': None,
            'gps': None
        }
        
        # Maximum age of sensor data to be considered valid (in seconds)
        self.max_data_age = 0.5  # 500ms

    def right_throttle_callback(self, msg):
        self.right_throttle = msg.data

    def left_throttle_callback(self, msg):
        self.left_throttle = msg.data
    
    def camera_callback(self, data):
        self.image = self.br.imgmsg_to_cv2(data)
        self.image = self.image[:,:,:3]
        self.last_updates['image'] = time.time()
    
    def velodyne_callback(self, data):
        self.scan = data
        self.last_updates['scan'] = time.time()

    def gps_callback(self, data):
        lat = data.lat
        lon = data.lon
        
        if not np.isnan(lat) and not np.isnan(lon):
            self.gps_data = {
                'latitude': self.gps_converter(lat),
                'longitude': self.gps_converter(lon) * -1.0,
                'track': data.track
            }
            self.last_updates['gps'] = time.time()
        else:
            self.gps_data = {
                'latitude': 0,
                'longitude': 0,
                'track': 0
            }
            self.last_updates['gps'] = time.time()

    def gps_converter(self, point):
        degrees = np.floor(point / 100)
        minutes = point - degrees * 100
        return degrees + minutes / 60
    
    def check_data_fresh(self):
        """Check if all sensor data is available and fresh"""
        current_time = time.time()
        
        for sensor, last_update in self.last_updates.items():
            if last_update is None:
                self.get_logger().warn(f'No {sensor} data received yet')
                return False
            
            if current_time - last_update > self.max_data_age:
                self.get_logger().warn(f'{sensor} data is too old: {current_time - last_update:.2f}s')
                return False
        
        return True
    
    def save_data(self):
        if not self.check_data_fresh():
            return
            
        timestamp = self.get_clock().now().to_msg()
        
        # Save raw PointCloud2 message
        scan_filename = f'dataset/scans/scan_{timestamp.sec}_{timestamp.nanosec}.msg'
        try:
            with open(scan_filename, 'wb') as f:
                serialized_data = serialize_message(self.scan)
                f.write(serialized_data)
        except Exception as e:
            self.get_logger().error(f'Error saving scan: {e}')
            return
        
        # Save image
        image_filename = f'dataset/images/image_{timestamp.sec}_{timestamp.nanosec}.jpg'
        cv2.imwrite(image_filename, self.image)
        
        # Save GPS data to CSV
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f'{timestamp.sec}.{timestamp.nanosec}',
                self.gps_data['latitude'],
                self.gps_data['longitude'],
                self.gps_data['track'],
                scan_filename,
                image_filename,
                self.left_throttle,
                self.right_throttle
            ])
        
        # Save metadata
        metadata = {
            'timestamp': {
                'sec': timestamp.sec,
                'nanosec': timestamp.nanosec
            },
            'scan_file': scan_filename,
            'image_file': image_filename,
            'gps': self.gps_data
        }
        self.metadata_list.append(metadata)
        
        self.get_logger().info(f'Saved synchronized data point at {timestamp.sec}.{timestamp.nanosec}')
    
    def shutdown(self):
        # Save metadata
        with open('dataset/metadata.json', 'w') as f:
            json.dump(self.metadata_list, f)
        self.get_logger().info('Saved metadata file')

def main():
    rclpy.init()
    collector = DataCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.shutdown()
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()