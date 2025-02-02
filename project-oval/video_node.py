#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
from datetime import datetime
import time

class ZEDRecorder(Node):
    def __init__(self):
        super().__init__('zed_recorder')
        
        # Create video output directory
        self.output_dir = "zed_recordings"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # Initialize the CV bridge
        self.bridge = CvBridge()
        
        # Initialize VideoWriter as None (will be created when first image arrives)
        self.video_writer = None
        
        # FPS control
        self.target_fps = 5
        self.last_frame_time = time.time()
        self.frame_interval = 1.0 / self.target_fps
        
        # Subscribe to the ZED camera topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/stereo/image_rect_color',
            self.camera_callback,
            10)
        
        self.get_logger().info(f'ZED recorder node started - Recording at {self.target_fps} FPS')

    def camera_callback(self, msg):
        current_time = time.time()
        
        # Skip frames to maintain target FPS
        if current_time - self.last_frame_time < self.frame_interval:
            return
            
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg)
        
        # Initialize video writer if it hasn't been created yet
        if self.video_writer is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.output_dir, f'zed_recording_{timestamp}.avi')
            
            height, width = current_frame.shape[:2]
            self.video_writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*'XVID'),
                self.target_fps,  # Reduced FPS
                (width, height)
            )
            self.get_logger().info(f'Started recording to {filename}')
        
        # Write the frame
        self.video_writer.write(current_frame)
        
        # Optional: Display the frame
        cv2.imshow('ZED Camera Feed', current_frame)
        cv2.waitKey(1)
        
        # Update last frame time
        self.last_frame_time = current_time

    def stop_recording(self):
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        self.get_logger().info('Recording stopped')

def main(args=None):
    rclpy.init(args=args)
    recorder = ZEDRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()