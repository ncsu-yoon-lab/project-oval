#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ROS2 subscriber: messages to send to serial
        self.subscription = self.create_subscription(
            String,
            'telemetry_message',
            self.telemetry_callback,
            10)
        self.subscription  # prevent unused variable warning

        # ROS2 publisher: messages received from serial
        self.publisher = self.create_publisher(String, 'telemetry_command', 10)

        # Serial configuration
        serial_port = '/dev/ttyUSB0'
        baud_rate = 57600

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.5)
            self.get_logger().info(f'Opened serial port {serial_port} at {baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            self.ser = None
            return

        # Start reading thread
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

    def telemetry_callback(self, msg: String):
        # send message to the vehicle
        if self.ser and self.ser.is_open:
            data = msg.data.strip() + '\n'
            try:
                self.ser.write(data.encode('utf-8'))
                self.get_logger().info(f'Sent over serial: {msg.data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
        else:
            self.get_logger().warn('Serial port not open. Message not sent.')

    def read_serial(self):
        # received command from the vehicle. Handle the command
        while rclpy.ok() and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'Received from vehicle: {line}')

                        msg = String()
                        msg.data = line

                        #line can be "cmd:left, cmd:straight, cmd:right" 
                        #it is published to 'telemetry_command'
                        self.publisher.publish(msg)

                time.sleep(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(1)  # wait before retrying

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt. Exiting...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()