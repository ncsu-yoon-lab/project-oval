#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from std_msgs.msg import Bool
import sys
import threading
import time
from pyvesc import VESC
import atexit            

MAX_INPUT = 100.0

stop_signal = False

class DriverNode(Node):
    def __init__(self):
        super().__init__("driver")
        self.throttle = 0
        self.steer = 0
        self.serial_port = '/dev/ttyTHS0'

        # Setup VESC
        self.motor = VESC(self.serial_port)
        atexit.register(self.cleanup)
        self.right_id = 115

        self.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)
        self.create_subscription(Bool, 'lidar_od_signal', self.brake_callback, 10)

    def steer_callback(self, msg):
        self.steer = msg.data
    
    def brake_callback(self, msg):
        global stop_signal
        stop_signal = msg.data
        
        if stop_signal:
            print(f"Stop Signal: {stop_signal}")


    def throttle_callback(self, msg):
        if (msg.data > MAX_INPUT):
            self.throttle = MAX_INPUT
        elif (msg.data < -MAX_INPUT):
            self.throttle = -MAX_INPUT
        else:
            self.throttle = msg.data

    def arcade_drive(self, throttle, steer):
        throttle *= -1.0
        steer *= -1.0
        if stop_signal and throttle > 0:
            throttle = 0
        maximum = max(abs(steer), abs(throttle))
        total, difference = throttle + steer, throttle - steer

        if throttle >= 0:
            if steer >= 0:  # I quadrant
                throttle_left = maximum
                throttle_right = difference
            else:            # II quadrant
                throttle_left = total
                throttle_right = maximum
        else:
            if steer >= 0:  # IV quadrant
                throttle_left = total
                throttle_right = -maximum
            else:            # III quadrant
                throttle_left = -maximum
                throttle_right = difference

        return throttle_left, throttle_right

    def send_speeds(self):
        try:
            left_throttle, right_throttle = self.arcade_drive(self.throttle, self.steer)
            
            # Convert to duty cycle (-1.0 to 1.0)
            left_duty = left_throttle / MAX_INPUT
            right_duty = right_throttle / MAX_INPUT
            
            # Send duty cycle to motors
            self.motor.set_duty_cycle(left_duty)
            self.motor.set_duty_cycle(right_duty, can_id=self.right_id)
            
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

    def cleanup(self):
        """Stop motors and close connection"""
        self.motor.set_duty_cycle(0)
        self.motor.set_duty_cycle(0, can_id=self.right_id)
        self.motor.stop_heartbeat()

def main():
    rclpy.init()
    node = DriverNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.send_speeds()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Serials Closed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()