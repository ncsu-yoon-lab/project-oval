#!/usr/bin/env python3
'''
IMU Publisher Node - Read Gyro and Accelerometer data from MPU6050 and publish as ROS2 topic
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus
import time

# MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('cheap_imu_publisher')
        
        # Create publisher for IMU data
        self.imu_publisher = self.create_publisher(
            Float64MultiArray, 
            '/cheap_imu', 
            10
        )
        
        # Set up MPU6050
        # Based on the diagnostic results, use bus 7 for Jetson Orin
        self.bus = smbus.SMBus(7)  # Changed from bus 1 to bus 7
        self.device_address = 0x68  # MPU6050 device address
        
        # Initialize the MPU6050
        self.mpu_init()
        
        # Create timer that will call the publish_imu_data method at defined frequency
        self.timer_period = 0.02  # 50 Hz - matches high frequency data collection
        self.timer = self.create_timer(self.timer_period, self.publish_imu_data)
        
        # Log that we've started up
        self.get_logger().info('Cheap IMU Publisher Node initialized')
    
    def mpu_init(self):
        """Initialize MPU6050 settings"""
        try:
            # Write to sample rate register
            self.bus.write_byte_data(self.device_address, SMPLRT_DIV, 7)
            
            # Write to power management register
            self.bus.write_byte_data(self.device_address, PWR_MGMT_1, 1)
            
            # Write to Configuration register
            self.bus.write_byte_data(self.device_address, CONFIG, 0)
            
            # Write to Gyro configuration register
            self.bus.write_byte_data(self.device_address, GYRO_CONFIG, 24)
            
            # Write to interrupt enable register
            self.bus.write_byte_data(self.device_address, INT_ENABLE, 1)
            
            self.get_logger().info('MPU6050 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {str(e)}')
    
    def read_raw_data(self, addr):
        """Read raw data from MPU6050 register"""
        try:
            # Accelero and Gyro value are 16-bit
            high = self.bus.read_byte_data(self.device_address, addr)
            low = self.bus.read_byte_data(self.device_address, addr+1)
            
            # Concatenate higher and lower value
            value = ((high << 8) | low)
            
            # To get signed value from MPU6050
            if(value > 32768):
                value = value - 65536
            
            return value
        except Exception as e:
            self.get_logger().error(f'Error reading from MPU6050: {str(e)}')
            return 0
    
    def publish_imu_data(self):
        """Read IMU data and publish to ROS2 topic"""
        try:
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Full scale range +/- 250 degree/C as per sensitivity scale factor
            # Convert to physical units
            Ax = acc_x/16384.0  # Convert to g
            Ay = acc_y/16384.0
            Az = acc_z/16384.0
            
            Gx = gyro_x/131.0  # Convert to deg/s
            Gy = gyro_y/131.0
            Gz = gyro_z/131.0
            
            # Create message - ordering matters to match the data collector expectations
            # Order: [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z]
            imu_msg = Float64MultiArray()
            imu_msg.data = [Gx, Gy, Gz, Ax, Ay, Az]
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            
            # Log at debug level (less frequent)
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0
                
            if self.log_counter % 50 == 0:  # Log every ~1 second at 50Hz
                self.get_logger().debug(
                    f'Published IMU data: Gx={Gx:.2f} deg/s, Gy={Gy:.2f} deg/s, Gz={Gz:.2f} deg/s, '
                    f'Ax={Ax:.2f} g, Ay={Ay:.2f} g, Az={Az:.2f} g'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in publish_imu_data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_publisher = ImuPublisher()
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        print('\nShutting down IMU Publisher Node...')
    except Exception as e:
        print(f'Unexpected error: {str(e)}')
    finally:
        # Cleanup
        rclpy.shutdown()

if __name__ == '__main__':
    main()