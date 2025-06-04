#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from rclpy.serialization import deserialize_message
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct
import json
import time

class DataPlayer(Node):
    def __init__(self):
        super().__init__('data_player')
        self.cloud_publisher = self.create_publisher(PointCloud2, 'replayed_pointcloud', 10)
        self.image_publisher = self.create_publisher(Image, 'replayed_image', 10)
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()  # Enable interactive mode
        
    def pointcloud2_to_xyz(self, cloud_msg):
        """Convert PointCloud2 message to numpy array of XYZ coordinates."""
        # Get cloud data as binary array
        points_data = cloud_msg.data
        
        # Get the number of points
        n_points = cloud_msg.width * cloud_msg.height
        
        # Get the point step and field offsets
        point_step = cloud_msg.point_step
        x_offset = y_offset = z_offset = None
        
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
                
        if None in (x_offset, y_offset, z_offset):
            raise ValueError("Point cloud is missing XYZ coordinates")
            
        # Create numpy array for XYZ coordinates
        xyz = np.zeros((n_points, 3))
        
        for i in range(n_points):
            start_idx = i * point_step
            xyz[i, 0] = struct.unpack('f', points_data[start_idx + x_offset:start_idx + x_offset + 4])[0]
            xyz[i, 1] = struct.unpack('f', points_data[start_idx + y_offset:start_idx + y_offset + 4])[0]
            xyz[i, 2] = struct.unpack('f', points_data[start_idx + z_offset:start_idx + z_offset + 4])[0]
            
        return xyz
        
    def visualize_pointcloud(self, xyz_points):
        """Visualize point cloud using matplotlib."""
        self.ax.cla()  # Clear previous points
        self.ax.scatter(xyz_points[:, 0], xyz_points[:, 1], xyz_points[:, 2], 
                       c=xyz_points[:, 2], cmap='viridis', marker='.', s=1)
        
        # Set labels and title
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Point Cloud Visualization')
        
        # Set consistent axis limits (adjust these based on your data)
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.ax.set_zlim([-5, 5])
        
        plt.draw()
        plt.pause(0.01)  # Small pause to update the plot

    def load_pointcloud(self, filename):
        """Load a saved PointCloud2 message from file."""
        try:
            with open(filename, 'rb') as f:
                serialized_data = f.read()
            return deserialize_message(serialized_data, PointCloud2)
        except Exception as e:
            self.get_logger().error(f'Error loading pointcloud from {filename}: {e}')
            return None
        
    def replay_data(self, metadata_file, playback_rate=2.0):
        """Replay the saved data at the specified rate."""
        try:
            with open(metadata_file, 'r') as f:
                metadata = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Error loading metadata: {e}')
            return

        self.get_logger().info(f'Starting replay of {len(metadata)} data points')
        
        for entry in metadata:
            if not rclpy.ok():
                break
                
            # Load and publish pointcloud
            cloud_msg = self.load_pointcloud(entry['scan_file'])
            if cloud_msg is not None:
                # Publish the message
                self.cloud_publisher.publish(cloud_msg)
                
                # Convert to XYZ points and visualize
                xyz_points = self.pointcloud2_to_xyz(cloud_msg)
                self.visualize_pointcloud(xyz_points)
                
                self.get_logger().info(f"Published and visualized pointcloud from {entry['scan_file']}")
            
            # Sleep to maintain playback rate
            time.sleep(1.0 / playback_rate)

def main():
    rclpy.init()
    player = DataPlayer()
    
    try:
        # Start replay at 2Hz
        player.replay_data('dataset/metadata.json', playback_rate=2.0)
        rclpy.spin(player)
    except KeyboardInterrupt:
        pass
    finally:
        player.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()