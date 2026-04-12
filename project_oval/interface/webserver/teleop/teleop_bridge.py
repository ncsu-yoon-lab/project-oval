import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
import config

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        

    def cmd_vel_callback(self, msg):

        payload = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }

        try:
            requests.post(
                f'https://webserver:{config.PORT}/cmd_vel',
                json=payload,
                cert=(config.CERT_FILE, config.KEY_FILE),
                verify=config.CA_FILE,
                timeout=0.1
            )
        except Exception as e:
            self.get_logger().error(f'Failed to forward cmd_vel: {e}')

def main():
    rclpy.init()
    node = TeleopBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()