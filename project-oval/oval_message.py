import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class OvalMessagePublisher(Node):
    def __init__(self):
        super().__init__('oval_message_publisher')
        self.publisher_ = self.create_publisher(String, '/oval_message', 10)

    def publish_message(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")

def main():
    rclpy.init()
    node = OvalMessagePublisher()

    try:
        # Example messages (you can modify these)
        messages = [
            "Welcome to the NC State OVAL Project!",
            "It's an autonomous small patrol vehicle built by Yoon Lab \n in the NC State University Department of Computer Science.",
            "Students are testing the car around the Oval!",
            "Stay tuned for our next milestone!",
            "Cole Malinchock \n Jack Elia \n Pratik Thapa \n Rosemary Bumgardner \n Sophie Noble \n Harper Martin \n Ryan Atack \n Dinesh Karnati \n Tyler Arnold \n Suchir Madap \n Jimin Yu \n Dhruva UP \n Dr. Man-Ki Yoon"
        ]

        while True:
            for text in messages:
                node.publish_message(text)
                time.sleep(5)  # send one message every 5 seconds
            

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()