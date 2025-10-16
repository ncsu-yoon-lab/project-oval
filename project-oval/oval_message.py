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
            "Project OVAL @ RIoT 2025!\n\nIt's an autonomous small patrol vehicle built by\n\nYoon Lab @ NC State",
            "Dept. of Computer Science \n\n Dept. of Mechanical and Aerospace Engineering \n\n Dept. of Electrical and Computer Engineering",
            "Cole Malinchock \n Jack Elia \n Pratik Thapa \n Rosemary Bumgardner \n Sophie Noble \n Harper Martin \n Suchir Madap \n Dinesh Karnati \n Tyler Arnold \n Ryan Atack \n Jimin Yu \n Ashwikaa Balasubramaniyan \n Dhruva UP \n Yuheng Zhu",
            "Students are testing the car around the Oval!\n\nWe are looking for collaboration (and sponsorships)!\n\nStay tuned for our next milestone!"
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
