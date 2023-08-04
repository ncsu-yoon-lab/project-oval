#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
import threading


astar_turn_val = None
ld_turn_sig = 0

def astar_turn_callback(data):
    global astar_turn_val
    astar_turn_val = data.data


def ld_turn_callback(data):
    global ld_turn_sig
    ld_turn_sig = data.data



def main(args=None):
    print("Driver node")
    rclpy.init(args=args)
    node = Node("Drive_node")


    pub_astar_turnsig = node.create_publisher(Int64, 'astar_turn_sig', 1)
    pub_ld_turnval = node.create_publisher(Int64MultiArray, 'ld_turn_val', 1)
    sub_astar_turnval = node.create_subscription(Int64MultiArray, 'astar_turn_val', astar_turn_callback, 1)
    sub_ld_turnsig = node.create_subscription(Int64, 'ld_turn_sig', ld_turn_callback, 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    while rclpy.ok():
        
        if astar_turn_val is not None:
            print("signal received from AStar")
            a = Int64MultiArray()
            a.data = astar_turn_val
            pub_ld_turnval.publish(a)
            print("signal sent to LD")
            b = Int64()
            b.data = int(0)
            pub_astar_turnsig.publish(b)
        if ld_turn_sig != 0:
            print("signal received from LD")
            print(ld_turn_sig)
            b = Int64()
            b.data = int(ld_turn_sig)
            pub_astar_turnsig.publish(b)
            print("signal sent to AStar")


        rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
