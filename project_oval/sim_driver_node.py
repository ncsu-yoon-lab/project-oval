import math
import time

import rclpy
from std_msgs.msg import Int64MultiArray, Bool, Int64, String


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# On sim, max input is 10
MAX_INPUT = 30
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

        self.__max_velocity = 10 # m/s Can be tuned for car
        self.__max_angular_velocity = self.__max_velocity

        # TODO: correct formula: self.__max_angular_velocity = self.__max_velocity / WHEEL_RADIUS


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
        self.__node.create_subscription(Int64, '/xbox_controller/steer', self.steer_callback, 10)
        self.__node.create_subscription(Int64, '/xbox_controller/throttle', self.throttle_callback, 10)
        self.__node.create_subscription(Bool, '/xbox_controller/mode', self.mode_callback, 10)

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
    def duty_cycle_to_angular_velocity(self, duty_cycle):
        # duty cycle: [-1, 1] where sign indicates direction and magnitude indicates is the PWM duty cycle
        # For now, the conversion will just be using the duty cycle as a percentage of the max angular cycle
        # TODO: Potentially find a better conversion method (or we could ignore duty cycle entirely)
        angular_velocity = duty_cycle * self.__max_angular_velocity
        print(duty_cycle)
        print(self.__max_angular_velocity)
        return angular_velocity

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

    # Start Helper functions from DriverNode
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

    # End Helper function from DriverNode


    # Modified send_speeds function for the simulator
    def send_speeds(self):
        try:
            if self.mode == "Auto":
                left_throttle, right_throttle = self.arcade_drive(self.auto_throttle, self.auto_steer)
            else:
                left_throttle, right_throttle = self.arcade_drive(self.manual_throttle, self.manual_steer)

            #print(f"Left Throttle: {left_throttle}, Right Throttle: {right_throttle}")
            
            # Convert to duty cycle (-1.0 to 1.0)
            left_duty = left_throttle / MAX_INPUT
            right_duty = right_throttle / MAX_INPUT

            right_duty *= -1
            
            # wheel 1 and 3 represent the left drivetrain on car
            # wheel 2 and 4 represent right drivetrain on car.
            # We may need to modify sim car somehow if we want it to be more mechanically similar to Oval Car
            left_drive_train = [self.__motor_devices[0], self.__motor_devices[2]]
            right_drive_train = [self.__motor_devices[1], self.__motor_devices[3]]
            # Send duty cycle to motors
            left_omega = self.duty_cycle_to_angular_velocity(left_duty)
            right_omega = self.duty_cycle_to_angular_velocity(right_duty)
            print(left_omega)
            print(right_omega)
            for m in left_drive_train:
                m.setVelocity(left_omega)
            for m in right_drive_train:
                m.setVelocity(right_omega)
            
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
