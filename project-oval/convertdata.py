#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import deserialize_message
import numpy as np
import json
import struct
import os

class DataExporter(Node):
    def __init__(self):
        super().__init__('data_exporter')
        # Create jsonscan directory if it doesn't exist
        self.output_dir = 'jsonscan'
        os.makedirs(self.output_dir, exist_ok=True)
        
    def pointcloud2_to_xyz(self, cloud_msg):
        """Convert PointCloud2 message to numpy array of XYZ coordinates."""
        points_data = cloud_msg.data
        n_points = cloud_msg.width * cloud_msg.height
        point_step = cloud_msg.point_step
        
        # Get field offsets
        offsets = {field.name: field.offset for field in cloud_msg.fields}
        if not all(coord in offsets for coord in ['x', 'y', 'z']):
            raise ValueError("Point cloud is missing XYZ coordinates")
            
        # Create numpy array and fill with XYZ data
        xyz = np.zeros((n_points, 3))
        for i in range(n_points):
            start_idx = i * point_step
            for j, coord in enumerate(['x', 'y', 'z']):
                offset = offsets[coord]
                xyz[i, j] = struct.unpack('f', points_data[start_idx + offset:start_idx + offset + 4])[0]
        
        # Filter out rows with NaN or null values
        valid_mask = ~np.isnan(xyz).any(axis=1)
        xyz_filtered = xyz[valid_mask]
        
        # Only return data if we have valid points
        if xyz_filtered.shape[0] > 0:
            return xyz_filtered.tolist()
        else:
            raise ValueError("No valid points found after filtering")
        
    def export_data(self, metadata_file):
        """Export all pointcloud scans to individual JSON files."""
        with open(metadata_file, 'r') as f:
            metadata = json.load(f)

        total_scans = len(metadata)
        self.get_logger().info(f'Exporting {total_scans} point cloud scans')
        
        processed_scans = 0
        skipped_scans = 0
        
        # Process all entries
        for i, entry in enumerate(metadata):
            try:
                with open(entry['scan_file'], 'rb') as f:
                    cloud_msg = deserialize_message(f.read(), PointCloud2)
                
                # Convert to XYZ and filter out invalid points
                try:
                    filtered_data = self.pointcloud2_to_xyz(cloud_msg)
                    
                    # Count number of points before and after filtering
                    original_points = cloud_msg.width * cloud_msg.height
                    filtered_points = len(filtered_data)
                    
                    frame_data = {
                        "filename": os.path.basename(entry['scan_file']),
                        "data": filtered_data,
                        "original_points": original_points,
                        "filtered_points": filtered_points
                    }
                    
                    # Create output filename based on original filename
                    base_filename = os.path.splitext(frame_data['filename'])[0]
                    output_file = os.path.join(self.output_dir, f"{base_filename}.json")
                    
                    # Save individual JSON file
                    with open(output_file, 'w') as f:
                        json.dump(frame_data, f)
                    
                    processed_scans += 1
                    self.get_logger().info(
                        f"Exported scan {i+1}/{total_scans} to {output_file} "
                        f"(Filtered {original_points - filtered_points} invalid points)"
                    )
                    
                except ValueError as ve:
                    skipped_scans += 1
                    self.get_logger().warning(f"Skipping scan {entry['scan_file']}: {ve}")
                    continue
                
            except Exception as e:
                skipped_scans += 1
                self.get_logger().error(f"Error processing {entry['scan_file']}: {e}")
                continue
            
        self.get_logger().info(
            f"Export complete. Processed {processed_scans} scans, "
            f"skipped {skipped_scans} scans. Files saved in {self.output_dir}/"
        )

def main():
    rclpy.init()
    exporter = DataExporter()
    
    try:
        exporter.export_data('dataset/metadata.json')
    except Exception as e:
        print(f"Error during export: {e}")
    finally:
        exporter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()