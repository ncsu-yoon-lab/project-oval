#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
import struct
import can
import threading
import time

throttle = 0
steer = 0
manual_mode = True

PWM_MAX = 2000
PWM_CENTER = 1500
PWM_MIN = 1000

INPUT_MAX = 100
INPUT_CENTER = 0
INPUT_MIN = -100

PWM_COEFFICIENT = (PWM_MAX - PWM_CENTER) / INPUT_MAX

def mode_callback(data):
    """
    Param data: Bool if it is currently in manual mode
    """
    global manual_mode
    manual_mode = data.data

def manual_throttle_callback(data):
    """
    Param data: Int64 of the manual throttle [INPUT_MIN, INPUT_MAX]
    """
    if(manual_mode):
        global throttle
        throttle = data.data

def manual_steering_callback(data):
    """
    Param data: Int64 of the manual steering [INPUT_MIN, INPUT_MAX]
    """
    if(manual_mode):
        global steer
        steer = data.data

def auto_throttle_callback(data):
    """
    Param data: Int64 of the auto throttle [INPUT_MIN, INPUT_MAX]
    """
    if(not manual_mode):
        global throttle
        throttle = data.data

def auto_steering_callback(data):
    """
    Param data: Int64 of the auto steering [INPUT_MIN, INPUT_MAX]
    """
    if(not manual_mode):
        global steer
        steer = data.data

def pwm_converter(throttle, steer):
    """
    Param throttle: int of throttle [INPUT_MIN, INPUT_MAX]
    Param steer: int of steer [INPUT_MIN, INPUT_MAX]

    Return motor1_pwm: int of pwm for motor1 [PWM_MIN, PWM_MAX]
    Return motor2_pwm: int of pwm for motor2 [PWM_MIN, PWM_MAX]
    """

    # Set both motor PWMs to the same based on the throttle
    # Converts the [-100, 100] range to [1000, 2000] range
    motor1_pwm = motor2_pwm = (throttle * PWM_COEFFICIENT) + PWM_CENTER

    # Adjust the PWMs based on the steering
    #TODO

    return motor1_pwm, motor2_pwm


def main(args=None):
    
    rclpy.init(args=args)
    node = Node("driver")

    mode_sub = node.create_subscription(Bool, "/xbox_controller/mode", mode_callback, 10)
    manual_throttle_sub = node.create_subscription(Int64, "/xbox_controller/throttle", manual_throttle_callback, 10)
    manual_steering_sub = node.create_subscription(Int64, "/xbox_controller/steering", manual_steering_callback, 10)

    auto_throttle_sub = node.create_subscription(Int64, "/lane_follower/throttle", auto_throttle_callback, 10)
    auto_steering_sub = node.create_subscription(Int64, "/lane_follower/steering", auto_steering_callback, 10)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            motor1_pwm, motor2_pwm = pwm_converter(throttle, steering)
        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()