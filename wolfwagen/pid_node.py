#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import example_interfaces.msg as eim
import std_msgs.msg as sm
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import threading

# sigmoid transform constants
# > e = EULER
# > sigmoid will have domain (-inf, inf) and range (-SIGMOID_ASYMPTOTE, SIGMOID_ASYMPTOTE)
# > sigmoid will accelerate towards asymptotes directly proportionally to SIGMOID_EXP_SCALAR 
EULER = 2.71828
SIGMOID_ASYMPTOTE = 100
SIGMOID_EXP_SCALAR = 0.05

# PID constants
# how to tune: https://pidexplained.com/how-to-tune-a-pid-controller/
Kp = 0.2
Ki = 0
Kd = 0

# PID frequency
FREQ = 20

# delta time
dt = 1 / float(FREQ)

# callback globals
err = 0.0
d_err = 0.0
i_err = 0.0

def sigmoid_normalize(x: float) -> float:
    sigmoid = ((2 * SIGMOID_ASYMPTOTE) / (1 + (EULER ** (-x * SIGMOID_EXP_SCALAR)))) - SIGMOID_ASYMPTOTE
    return sigmoid

def lane_callback(msg: sm.Int64) -> None:

    # expect lane to pass error between trajectory center line and lane average center line.
    # > negative if the lane average center line is to the left of the trajectory center line (have to turn left),
    # > positive if to the right (have to turn right)
    
    global err , d_err , i_err , dt
    d_err = (msg.data - err) / dt
    i_err += msg.data
    err = msg.data
    

def main(args=None) -> None:

    global err , d_err , i_err , dt , Kp , Ki , Kd

    # initialize node and all related ros2 constructs
    rclpy.init(args=args)
    node = Node("pid_node")
    
    node.create_subscription(
        sm.Int64,
        "lane",
        lane_callback,
        10
    )

    publisher = node.create_publisher(
        sm.Int64,
        "pid_steering",
        10
    )
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(FREQ, node.get_clock())

    while rclpy.ok():
        # u = Kp * err + Ki * i_err + Kd * d_err
        p = Kp * err
        i = Ki * i_err
        d = Kd * d_err
        u = p + i + d

        # normalize to smooth values and keep published values in bounds
        u = sigmoid_normalize(u)
        print("THIS IS U: " , u)

        # floor normalized value
        m = sm.Int64()
        m.data = int(u)

        # publish
        publisher.publish(m)

        # maintain rate
        rate.sleep()
        

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
