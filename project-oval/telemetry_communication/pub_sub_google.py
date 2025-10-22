# All the imports 
from std_msgs.msg import String
from std_msgs.msg import Float64 
import rclpy
from rclpy.node import Node
from google.cloud import pubsub_v1
import eventlet 
from datetime import datetime
import json
from geometry_msgs.msg import TwistStamped
from gps_msgs.msg import GPSFix
import cv2 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import time
import threading

PROJECT_ID = "yoon-lab"
ROBOT_POSITION_PUBLISH_TOPIC_NAME = "robot-position"
ROBOT_CAMERA_PUBLISH_TOPIC_NAME = "robot-camera"
CLOUD_SUB_ROBOT_COMMAND = "robot-commands-sub"
SUB_GPS_FIX = "/gpsfix"
SUB_GPS_VELOCITY = "/gps/velocity"
SUB_GPS_HEADING = "/gps/heading"
PUBLISH_DESTINATION_TOPIC = "/robot/command_destination"

class PubSubGoogleNode(Node):
    def __init__(self, project_id=PROJECT_ID, 
                 robot_position_publish_topic_name=ROBOT_POSITION_PUBLISH_TOPIC_NAME,
                 robot_camera_publish_topic_name=ROBOT_CAMERA_PUBLISH_TOPIC_NAME,
                 cloud_sub_robot_command=CLOUD_SUB_ROBOT_COMMAND):
        super().__init__('pubsub_google_node')
        
        # Initialize Pub/Sub client
        self.project_id = project_id
        self.robot_position_publish_topic_name = robot_position_publish_topic_name
        self.robot_camera_publish_topic_name = robot_camera_publish_topic_name
        self.cloud_sub_robot_command = cloud_sub_robot_command

        # OpenCV Video Capture
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video4')
        
        # Pub/Sub Publisher and Subscriber for Google Cloud
        self.publisher = pubsub_v1.PublisherClient()
        self.pub_position_topic_path = self.publisher.topic_path(self.project_id, self.robot_position_publish_topic_name)
        self.pub_camera_topic_path = self.publisher.topic_path(self.project_id, self.robot_camera_publish_topic_name)

        self.subscriber = pubsub_v1.SubscriberClient()
        self.subscription_path = self.subscriber.subscription_path(self.project_id, self.cloud_sub_robot_command)

        # Listen for incoming commands
        self.streaming_pull_future = self.subscriber.subscribe(self.subscription_path, callback=self.command_callback)

        # Publish destination command to ros2 topic
        self.command_pub = self.create_publisher(String, PUBLISH_DESTINATION_TOPIC, 10)

        # Robot state
        self.lat = 0.0
        self.lon = 0.0
        self.speed = 0.0
        self.heading = 0.0
        self.status = "initializing"
        
        # Camera publishing control
        self.last_camera_publish = time.time()
        self.camera_publish_interval = 0.1  # 10Hz

        # Subscribe to ROS2 topics RTK GPS data
        # GPSFix message contains lat, lon, speed, and track (heading)
        self.create_subscription(GPSFix, SUB_GPS_FIX, self.fix_callback, 10)
        
        # These callbacks now properly extract data from messages
        self.create_subscription(TwistStamped, SUB_GPS_VELOCITY, self.vel_callback, 10)
        self.create_subscription(Float64, SUB_GPS_HEADING, self.heading_callback, 10)
        
        # Start camera publishing thread
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

    def command_callback(self, message):
        try:
            data = json.loads(message.data.decode('utf-8'))
            
            self.get_logger().info(f'Received command message: {data}')
            
            robot_id = data.get('robot_id')
            timestamp = data.get('timestamp')
            target_lat = data.get('lat')
            target_lon = data.get('lon')
            motion = data.get('motion')
            
            if robot_id == "robot_001":
                self.target_lat = target_lat
                self.target_lon = target_lon
                
                self.get_logger().info(
                    f'Received command: Go to lat={target_lat}, lon={target_lon} '
                    f'(timestamp: {timestamp}, motion: {motion})'
                )
                
                # Publish to ROS2 topic 
                command_data = {
                    "robot_id": robot_id,
                    "timestamp": timestamp,
                    "target_lat": target_lat,
                    "target_lon": target_lon,
                    "motion": motion
                }

                command_msg = String()
                command_msg.data = json.dumps(command_data)
                self.command_pub.publish(command_msg)

                self.get_logger().info('Published destination to ROS2 topic')
                
            # Acknowledge the message
            message.ack()
            
        except Exception as e:
            self.get_logger().error(f'Error processing command message: {e}')
            message.nack()
    
    # Callback for GPS fix - extracts ALL data from GPSFix message
    def fix_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        # GPSFix message includes speed and track, extract them here
        self.speed = msg.speed  # m/s
        self.heading = msg.track  # degrees (0-360)
        self.status = "active"
        self.publish_position_to_cloud()
    
    # Callback for GPS velocity from TwistStamped
    def vel_callback(self, msg):
        # Extract linear velocity from TwistStamped message
        # Calculate speed from linear x, y, z components
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z
        self.speed = (vx**2 + vy**2 + vz**2)**0.5

    # Callback for GPS heading from Float64
    def heading_callback(self, msg):
        # Extract heading value from Float64 message
        self.heading = msg.data

    # Publish position data to Google Cloud Pub/Sub
    def publish_position_to_cloud(self):
        try:
            location_data = {
                "robot_id": "robot_001",
                "timestamp": datetime.utcnow().isoformat() + 'Z',
                "lat": self.lat,
                "lon": self.lon,
                "speed": self.speed,
                "heading": self.heading,
                "status": self.status
            }
            
            message_bytes = json.dumps(location_data).encode('utf-8')
            future = self.publisher.publish(self.pub_position_topic_path, data=message_bytes)
            future.result()
            
            self.get_logger().info(f'Published position: lat={self.lat:.6f}, lon={self.lon:.6f}, speed={self.speed:.2f}, heading={self.heading:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error publishing position to cloud: {e}')
    
    # Separate camera publishing loop to avoid blocking GPS updates
    def camera_loop(self):
        while rclpy.ok():
            try:
                current_time = time.time()
                if current_time - self.last_camera_publish >= self.camera_publish_interval:
                    self.publish_camera_to_cloud()
                    self.last_camera_publish = current_time
                else:
                    time.sleep(0.01)  # Small sleep to prevent busy-waiting
            except Exception as e:
                self.get_logger().error(f'Error in camera loop: {e}')
                time.sleep(1)  # Wait before retrying
    
    # Publish camera data to Google Cloud Pub/Sub
    def publish_camera_to_cloud(self):
        try:
            ret, frame = self.cap.read()
            
            if ret:
                # Split the side-by-side stereo image in half (left image only)
                height, width = frame.shape[:2]
                left_frame = frame[:, :width//2]
                
                _, buffer = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                image_bytes = buffer.tobytes()
                
                image_base64 = base64.b64encode(image_bytes).decode('utf-8')
                
                camera_data = {
                    "robot_id": "robot_001",
                    "timestamp": datetime.utcnow().isoformat() + 'Z',
                    "image": image_base64
                }

                message_bytes = json.dumps(camera_data).encode('utf-8')
                
                future_img = self.publisher.publish(self.pub_camera_topic_path, data=message_bytes)
                future_img.result()
                self.get_logger().info('Published camera image')
            else:
                self.get_logger().warn('Failed to read frame from camera')

        except Exception as e:
            self.get_logger().error(f'Error publishing camera to cloud: {e}')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'streaming_pull_future'):
            self.streaming_pull_future.cancel()
            
def main(args=None):
    rclpy.init(args=args)
    node = PubSubGoogleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()