import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
import serial
import struct
import numpy as np
import threading
import time

RTK_TOPIC = "/gpsfix"

# Set up serial
ser = serial.Serial('/dev/ttyTelemetry', baudrate=57600, timeout=1)

class GPSRadioTXNode(Node):
    def __init__(self):
        super().__init__('gps_radio_tx_node')
        self.declare_parameter("send_interval", 1.0)  # seconds
        
        self.subscription = self.create_subscription(
            GPSFix, RTK_TOPIC, self.rtk_callback, 10)
                
        self.latest_lat = None
        self.latest_lon = None
        
        # Timer for sending data periodically
        self.timer = self.create_timer(self.get_parameter("send_interval").value, self.send_data)
    
    def rtk_callback(self, msg):
        if not np.isnan(msg.latitude) and not np.isnan(msg.longitude):
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
            
        else:
            self.get_logger().warn("Received invalid RTK GPS data (NaN values)")
    
    def send_data(self):
        if self.latest_lat is not None and self.latest_lon is not None:
            # Use same format as CSV sender: '!dd' (double precision floats)
            data = struct.pack('!dd', self.latest_lat, self.latest_lon)
            ser.write(data)
            self.get_logger().info(f"Sent data: lat={self.latest_lat}, lon={self.latest_lon} -> {data.hex()} (size={len(data)} bytes)")
        else:
            self.get_logger().warn("No valid GPS data to send")

def main(args=None):
    rclpy.init(args=args)
    node = GPSRadioTXNode()
    
    try:
        rclpy.spin(node)
            
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()