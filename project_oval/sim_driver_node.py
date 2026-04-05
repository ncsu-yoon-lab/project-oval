import math
import time
from project_oval.sim_driver_lib import SimDriverLib
import rclpy
from std_msgs.msg import Int64MultiArray, Bool, Int64, String, Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from enum import Enum


# On sim, max input is 10
# MAX_INPUT = 30
'''
colcon build --packages-select project-oval --symlink-install
source install/setup.bash

Launch executable is in install/project-oval/share/project-oval/launch/
ros2 launch project-oval car_sim_launch.py
This is the driver for the simulated car. Ideally this should work like driver_node as much as possible.

For rviz testing:
ros2 run rviz2 rviz2
'''
WHEEL_RADIUS = 0.18
TRACK_WIDTH = 0.54


# Control modes
class DriveMode(Enum):
    XBOX = 0
    TELEOP = 1
    PURE_PURSUIT = 2

class SimDriverNode:
    def init(self, webots_node, properties):
        

        self.current_speed = 1.0
        self.__robot = webots_node.robot
        # names of the wheels in the simulation
        self.__motor_names = ["wheel1", "wheel2", "wheel3", "wheel4"]

        # This will contain connections to the wheel "motors" in the sim
        self.__motor_devices = [] 

        self.__time = 0.0

        # Brake command to prevent collisions and other potential uses (stopping for tours)
        self.__brake = False 
       
        # Contains methods for converting throttle to torque to be sent to motor
        self.driver_lib = SimDriverLib()

        # add motors to the motor devices
        for motor_name in self.__motor_names:
            motor = self.__robot.getDevice(motor_name)
            motor.setPosition(float('inf'))  # enable velocity control mode
            motor.setVelocity(0.0)
           
            #motor.setMaxTorque(max_motor_torque)
            self.__motor_devices.append(motor)

        # Start Class arguments from DriverNode
        self.mode = "Manual"
        self.manual_throttle = 0
        self.manual_steer = 0
        self.auto_throttle = 0
        self.auto_steer = 0
        self.stop_signal = False
        # End Class arguments from DriverNode

        # Pure pursuit params
        self.pp_left_omega = 0.0
        self.pp_right_omega = 0.0
        self.pp_use_throttle = False

        # Teleoperation mode:
        self.teleop_left_omega = 0.0
        self.teleop_right_omega = 0.0


        rclpy.init(args=None)
        self.__node = rclpy.create_node('sim_driver_node')

        # Subcribe to the xbox controller nodes
        self.__node.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.__node.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)
        self.__node.create_subscription(Bool, '/xbox_controller/mode', self.mode_callback, 10)

        # TODO: add support for gemini steering
        # self.create_subscription(Int64, '/gemini/throttle', self.auto_throttle_callback, 10)
        # self.create_subscription(Int64, '/gemini/steering', self.auto_steer_callback, 10)

        # pure puresuit subscribers
        self.__node.create_subscription(Float64MultiArray, "/pure_pursuit/omega", self.pp_omega_callback, 10)

        # Subscribe to brake signal
        self.__brake_sub = self.__node.create_subscription(Bool,"/brake", self.__brake_callback, 10)

        # Teleoperation mode
        self.__node.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Mode callback for changing modes (what is sending velocity commands)
        self.__node.create_subscription(String, "/driver/mode", self.mode_callback, 10)


    # Pure pursuit control callbacks
    def pp_omega_callback(self, msg: Float64MultiArray):
        self.pp_left_omega = msg.data[0]
        self.pp_right_omega = msg.data[1]

    # teleoperation mode callbacks
    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        self.teleop_left_omega = (linear - angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        self.teleop_right_omega = (linear + angular * TRACK_WIDTH / 2.0) / WHEEL_RADIUS

    # Start callbacks from DriverNode
    def steer_callback(self, msg):
        self.manual_steer = msg.data
    
    def brake_callback(self, msg):
        self.stop_signal = msg.data
        if self.stop_signal:
            print(f"Stop Signal: {self.stop_signal}")

    def mode_callback(self, msg):
        self.mode = DriveMode(int(msg.data))

    def auto_steer_callback(self, msg):
        self.auto_steer = msg.data

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
    
    # End callbacks from DriverNode

    # Start Helper functions from DriverNode
    def arcade_drive(self, throttle, steer):
        throttle *= -1.0
        #print("Arcade throttle: ", throttle)
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

    # End Helper function from DriverNode


    # Modified send_speeds function for the simulator
    def send_speeds(self):
        # For control code, for a differential drive robot it is better to use something like setRPM where PyVesc handles PID for us.
        try:
            left_drive_train = [self.__motor_devices[0], self.__motor_devices[2]]
            right_drive_train = [self.__motor_devices[1], self.__motor_devices[3]]

            if self.mode == DriveMode.TELEOP:
                for m in left_drive_train:
                    m.setVelocity(self.teleop_left_omega)
                for m in right_drive_train:
                    m.setVelocity(self.teleop_right_omega)

            elif self.mode == DriveMode.PURE_PURSUIT:
                for m in left_drive_train:
                    m.setVelocity(self.pp_left_omega)
                for m in right_drive_train:
                    m.setVelocity(self.pp_right_omega)

            return True
        except Exception as e:
            print(f"Error: {e}")
            return False 

    def __brake_callback(self, msg: Bool):
        self.__brake = msg.data
        self.__node.get_logger().info(f"Received brake={self.__brake}")


    def step(self):
        # process incoming ROS messages (including /brake)
        rclpy.spin_once(self.__node, timeout_sec=0.0)

        # send speeds to car
        self.send_speeds()