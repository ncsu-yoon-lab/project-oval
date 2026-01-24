import math

import rclpy
from std_msgs.msg import Bool

# import some of the methods from the driver node
from driver_node import arcade_drive, 

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


'''
This is the driver for the simulated car. Ideally this should work like driver_node as much as possible.
'''
class SimDriverNode:
    def init(self, webots_node, properties):

        self.current_speed = 1.0
        self.__robot = webots_node.robot
        # names of the wheels in the simulation
        self.__motor_names = ["wheel1", "wheel2", "wheel3", "wheel4"]

        # This will contain connections to the wheel "motors" in the sim
        self.__motor_devices = [] 

        self.__time = 0.0
        self.__brake = False   # updated from /brake

        # add motors to the motor devices
        for motor_name in self.__motor_names:
            motor = self.__robot.getDevice(motor_name)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.__motor_devices.append(motor)

        # Ensure the car begins at rest. Possible to remove this
        self.set_all_speed(0.0)

        # Start Class arguments from DriverNode
        self.mode = "Manual"
        self.manual_throttle = 0
        self.manual_steer = 0
        self.auto_throttle = 0
        self.auto_steer = 0
        self.stop_signal = False
        # End Class arguments from DriverNode

        rclpy.init(args=None)
        self.__node = rclpy.create_node('sim_driver_node')

        # Subcribe to the xbox controller nodes
        self.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)
        self.create_subscription(Bool, '/xbox_controller/mode', self.mode_callback, 10)

        # TODO: add support for gemini steering
        # self.create_subscription(Int64, '/gemini/throttle', self.auto_throttle_callback, 10)
        # self.create_subscription(Int64, '/gemini/steering', self.auto_steer_callback, 10)

        # Subscribe to brake signal
        self.__brake_sub = self.__node.create_subscription(
            Bool,
            "/brake",
            self.__brake_callback,
            10
        )

    '''
    The sim accepts velocity commands, but the actual motor driver uses PWM.
    This method converts the PWM signal to a velocity output for the sim to use.
    This is one area where the sim will likely differ from real life. Ideally, the 
    velocity sent to the motors will be similar to the velocity of the motors

    '''
    def duty_cycle_to_velocity(duty_cycle):
        # duty cycle: [-1, 1] where sign indicates direction and magnitude indicates is the PWM duty cycle


    def set_all_speed(self, speed: float):
        for motor in self.__motor_devices:
            motor.setVelocity(speed)

    
    # Start callbacks from DriverNode
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

    def __brake_callback(self, msg: Bool):
        self.__brake = msg.data
        self.__node.get_logger().info(f"Received brake={self.__brake}")


    def step(self):
        # process incoming ROS messages (including /brake)
        rclpy.spin_once(self.__node, timeout_sec=0.0)

        # otherwise move in squiggly oval
        self.drive_oval()
