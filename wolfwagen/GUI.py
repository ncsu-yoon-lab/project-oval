import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import threading
import tkinter as tk


def speed_callback(data):
    global speed
    speed = data.data


def main(args=None):
    global turn_direction , speed
    rclpy.init(args=args)
    node = Node("GUI_Node")
    print("GUI Node")

    sub_speed = node.create_subscription(Int64, 'speed', speed_callback, 1)
    sub_speed = node.create_subscription(Int64, 'direction', direction_callback, 1)
    sub_speed = node.create_subscription(Int64, 'battery', battery_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    while rclpy.ok():
        
        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()