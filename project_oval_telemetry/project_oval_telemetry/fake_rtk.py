#!/usr/bin/env python3
"""
Simple Fake RTK GPS Publisher
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

class FakeRTKGPSPublisher(Node):
    def __init__(self):
        super().__init__('fake_rtk_gps_publisher')
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/gps/velocity', 10)
        self.heading_pub = self.create_publisher(Float64, '/gps/heading', 10)
        
        # Simple position (Raleigh, NC)
        self.lat = 35.770653
        self.lon = -78.674890
        self.speed = 1.5
        self.heading = 45.0
        
        # Publish every 0.1 seconds (10Hz)
        self.timer = self.create_timer(0.1, self.publish_data)
        
        self.get_logger().info('Simple Fake RTK GPS started')
    
    def publish_data(self):
        # Small movement
        self.lat += 0.00001
        self.lon += 0.00001
        self.heading += 1.0
        if self.heading > 360.0:
            self.heading = 0.0
        
        # Publish position
        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps'
        fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = self.lat
        fix_msg.longitude = self.lon
        fix_msg.altitude = 100.0
        
        # Simple covariance (9 values)
        fix_msg.position_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.fix_pub.publish(fix_msg)
        
        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = fix_msg.header.stamp
        vel_msg.header.frame_id = 'base_link'
        vel_msg.twist.linear.x = self.speed
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0
        self.vel_pub.publish(vel_msg)
        
        # Publish heading
        heading_msg = Float64()
        heading_msg.data = self.heading
        self.heading_pub.publish(heading_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRTKGPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()