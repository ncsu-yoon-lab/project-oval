#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial
import sys
import threading
import struct
import time

MAX_INPUT = 100.0

MAX_OUTPUT = 4095

class DriverNode(Node):
    def __init__(self):
        super().__init__("driver")
        self.throttle = 0
        self.steer = 0
        self.init_serial()
        self.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)

    def init_serial(self):
        try:
            self.ser_left = serial.Serial('/dev/ttyACM0', 115200, timeout=1, write_timeout=1)
            self.ser_right = serial.Serial('/dev/ttyACM1', 115200, timeout=1, write_timeout=1)
            print(f"Opened {self.ser_left.name}")
            print(f"Opened {self.ser_right.name}")
        except serial.SerialException as e:
            print(f"Error: {e}")
            sys.exit(1)

    def steer_callback(self, msg):
        self.steer = msg.data
        # print(f"Steer: {self.steer}")

    def throttle_callback(self, msg):
        if (msg.data > MAX_INPUT):
            self.throttle = MAX_INPUT
        elif (msg.data < -MAX_INPUT):
            self.throttle = -MAX_INPUT
        else:
            self.throttle = msg.data

        # print(f"Throttle: {self.throttle}")

    def arcade_drive(self, throttle, steer):
        maximum = max(abs(steer), abs(throttle))
        total, difference = throttle + steer, throttle - steer

        multiplier = throttle / MAX_INPUT

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
            start_marker = b'\xAA'  # Start marker (10101010 in binary)
            data_left = start_marker + struct.pack('<f', float(left_throttle))  # 4-byte float
            data_right = start_marker + struct.pack('<f', float(right_throttle))  # 4-byte float

            self.ser_left.write(data_left)
            # print(f"Sent Left: {self.throttle}")

            self.ser_right.write(data_right)
            # print(f"Sent Right: {self.throttle}")

            # return self.ser_left.readline().decode().strip() == "OK" and self.ser_right.readline().decode().strip() == "OK"
            return True
        except Exception as e:
            print(f"Error: {e}")
            return False

def main():
    rclpy.init()
    node = DriverNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.send_speeds()
    except KeyboardInterrupt:
        node.ser_right.close()
        node.ser_left.close()
        print("Serials Closed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()