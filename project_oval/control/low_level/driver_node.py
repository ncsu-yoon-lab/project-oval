#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray, Bool, Int64, String, Float64MultiArray

from std_msgs.msg import Float64

import sys
import threading
import time
from pyvesc import VESC
import atexit
import time    

import pdb

MAX_INPUT = 100.0

class DriverNode(Node):
    def __init__(self):
        super().__init__("driver")
        self.mode = "Manual"
        self.manual_throttle = 0
        self.manual_steer = 0
        self.auto_throttle = 0
        self.auto_steer = 0
        self.serial_port = '/dev/ttyTHS1'
        self.stop_signal = False

        self.pp_left_omega = 0.0
        self.pp_right_omega = 0.0

        # Setup VESC
        self.motor = VESC(self.serial_port)
        print("Motor setup")
        atexit.register(self.cleanup)
        self.left_id = 115

        self.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)
        self.create_subscription(Bool, '/xbox_controller/mode', self.mode_callback, 10)
        self.rpm_pub = self.create_publisher(Int64MultiArray, '/motors/rpm', 10)
        
        ## Adding Gemini Controls
        self.create_subscription(Int64, '/gemini/throttle', self.auto_throttle_callback, 10)
        self.create_subscription(Int64, '/gemini/steering', self.auto_steer_callback, 10)

        ## Adding 
        self.telemetry_message = self.create_publisher(String, '/telemetry_message', 10)
        self.telemetry_command = self.create_subscription(String, "/telemetry_command", self.telemetry_command_callback, 5)

        ## Adding to handle pure pursuit
        self.pp_left_throttle = 0.0
        self.pp_right_throttle = 0.0
        self.pp_use_throttle = False

        # self.create_subscription(Float64, "/pure_pursuit/left_throttle", self.pp_left_throttle_callback, 10)
        # self.create_subscription(Float64, "/pure_pursuit/right_throttle", self.pp_right_throttle_callback, 10)
        # self.create_subscription(Bool, "/pure_pursuit/use_throttle", self.pp_use_throttle_callback, 10)
        ## 
        self.create_subscription(Float64MultiArray, "/pure_pursuit/omega", self.pp_omega_callback, 10)

    def steer_callback(self, msg):
        self.manual_steer = msg.data
    
    def brake_callback(self, msg):

        self.stop_signal = msg.data
        
        if self.stop_signal:
            print(f"Stop Signal: {self.stop_signal}")
    
    def mode_callback(self, msg):
        
        self.mode = "Manual" if msg.data else "Auto"

    def auto_steer_callback(self, msg):
        self.auto_steer = msg.data

    def telemetry_command_callback(self, msg):
        if (msg.data == "cmd:left"):
            self.auto_steer = 20
            self.auto_throttle = 20
            print("Received Left Command")
            time.sleep(1)
            self.auto_steer = 0
            self.auto_throttle = 0
        elif (msg.data == "cmd:right"):
            self.auto_steer = 20
            self.auto_throttle = 20
            print("Received Right Command")
            time.sleep(1)
            self.auto_steer = 0
            self.auto_throttle = 0
        else:
            self.auto_steer = 0
            self.auto_throttle = 0

    def auto_throttle_callback(self, msg):
        if (msg.data > MAX_INPUT):
            self.auto_throttle = MAX_INPUT
        elif (msg.data < -MAX_INPUT):
            self.auto_throttle = -MAX_INPUT
        else:
            self.auto_throttle = msg.data

    def throttle_callback(self, msg):
        if (msg.data > MAX_INPUT):
            self.manual_throttle = MAX_INPUT
        elif (msg.data < -MAX_INPUT):
            self.manual_throttle = -MAX_INPUT
        else:
            self.manual_throttle = msg.data
    
    # Pure pursuit control callbacks
    def pp_omega_callback(self, msg: Float64MultiArray):
        self.pp_left_omega = msg.data[0]
        self.pp_right_omega = msg.data[1]

    def arcade_drive(self, throttle, steer):
        throttle *= -1.0
        
        if self.stop_signal and throttle > 0:
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

    ## Adding to handle pure pursuit
    def pp_left_throttle_callback(self, msg: Float64):
        self.pp_left_throttle = msg.data

    def pp_right_throttle_callback(self, msg: Float64):
        self.pp_right_throttle = msg.data

    def pp_use_throttle_callback(self, msg: Bool):
        self.pp_use_throttle = msg.data
    ##
  

    def send_speeds(self):
        try:
            if self.mode == "Auto":
                ## Adding to handle pure pursuit
                # if self.pp_use_throttle:
                #     left_throttle = self.pp_left_throttle
                #     right_throttle = self.pp_right_throttle
                # else:
                #     left_throttle, right_throttle = self.arcade_drive(self.auto_throttle, self.auto_steer)
    
                # left_throttle, right_throttle = self.arcade_drive(self.auto_throttle, self.auto_steer)
                print(f"AUTO: Left RPM {self.pp_left_omega}, Right RPM {self.pp_right_omega}")
                self.motor.set_rpm(int(self.pp_right_omega))
                self.motor.set_rpm(int(self.pp_left_omega), can_id=self.left_id)
             
            else:
                left_throttle, right_throttle = self.arcade_drive(self.manual_throttle, self.manual_steer)

                print(f"Left Throttle: {left_throttle}, Right Throttle: {right_throttle}")
                
                ## Sending telemetry message
                tel_l_msg = String()
                tel_l_msg.data = f'Throttle_L:{left_throttle:.2f}'
                self.telemetry_message.publish(tel_l_msg)
                
                tel_r_msg = String()
                tel_r_msg.data = f'Throttle_R:{right_throttle:.2f}'
                self.telemetry_message.publish(tel_r_msg)
                
                # Convert to duty cycle (-1.0 to 1.0)
                left_duty = left_throttle / MAX_INPUT
                right_duty = right_throttle / MAX_INPUT

                right_duty *= -1
                
                # Send duty cycle to motors
                self.motor.set_duty_cycle(right_duty)
                self.motor.set_duty_cycle(left_duty, can_id=self.left_id)
            
            
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

            # rpm_msg = Int64MultiArray()
            # left_rpm = int(node.motor.get_rpm())
            # right_rpm = int(node.motor.get_rpm(can_id=node.left_id))
            # print(f"Left RPM: {left_rpm}, Right RPM: {right_rpm}")
            # rpm_msg.data = [left_rpm, right_rpm]
            # node.rpm_pub.publish(rpm_msg)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Keyboard interrupt: Serials Closed")
    except Exception as e:
        print("Exception caught: ", e)  
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()