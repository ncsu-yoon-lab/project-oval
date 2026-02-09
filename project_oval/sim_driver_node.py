import math
import time
from project_oval.sim_driver_lib import SimDriverLib
import rclpy
from std_msgs.msg import Int64MultiArray, Bool, Int64, String


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# On sim, max input is 10
MAX_INPUT = 30
'''
colcon build --packages-select project-oval --symlink-install
source install/setup.bash

Launch executable is in install/project-oval/share/project-oval/launch/
ros2 launch project-oval car_sim_launch.py
This is the driver for the simulated car. Ideally this should work like driver_node as much as possible.
'''
class SimDriverNode:
    def init(self, webots_node, properties):
        
        #Convert
        max_motor_torque=25450
        max_motor_torque = max_motor_torque / 1000

        self.current_speed = 1.0
        self.__robot = webots_node.robot
        # names of the wheels in the simulation
        self.__motor_names = ["wheel1", "wheel2", "wheel3", "wheel4"]

        # This will contain connections to the wheel "motors" in the sim
        self.__motor_devices = [] 

        self.__time = 0.0
        self.__brake = False   # updated from /brake


       
        # Contains methods for converting throttle to torque to be sent to motor
        self.driver_lib = SimDriverLib()

        # add motors to the motor devices
        for motor_name in self.__motor_names:
            motor = self.__robot.getDevice(motor_name)
           
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
        print("Arcade throttle: ", throttle)
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
                steer_cmd = self.auto_steer
                left_throttle, right_throttle = self.arcade_drive(self.auto_throttle, self.auto_steer)
            else:
                steer_cmd = self.manual_steer
                left_throttle, right_throttle = self.arcade_drive(self.manual_throttle, self.manual_steer)

            # --- Anti-deadlock for sim skid-steer ---
            # If we're steering but one side hits exactly 0, force a tiny counter-command
            # so the contact solver breaks symmetry and you get yaw.
            if abs(steer_cmd) > 1 and (right_throttle == 0 or left_throttle == 0):
                eps = 2  # in "throttle units" (tune 1-5)
                if right_throttle == 0:
                    right_throttle = -eps if left_throttle > 0 else eps
                if left_throttle == 0:
                    left_throttle = -eps if right_throttle > 0 else eps
        # # --------------------------------------
            print("-----")
            print("left throttle: ", left_throttle)
            print("right throttle: ", right_throttle)  


          
            # wheel 1 and 3 represent the left drivetrain on car
            # wheel 2 and 4 represent right drivetrain on car.
            # We may need to modify sim car somehow if we want it to be more mechanically similar to Oval Car
            left_drive_train = [self.__motor_devices[0], self.__motor_devices[2]]
            right_drive_train = [self.__motor_devices[1], self.__motor_devices[3]]

            left_speeds = [left_drive_train[0].getVelocity(), left_drive_train[1].getVelocity()]
            right_speeds = [right_drive_train[0].getVelocity(), right_drive_train[1].getVelocity()]

            left_torque = self.driver_lib.get_torque_input(left_throttle, left_speeds, lambda x: self.driver_lib.soft_pedal(x))
            per_motor_left = left_torque / 2

            right_torque = self.driver_lib.get_torque_input(right_throttle, right_speeds, lambda x: self.driver_lib.soft_pedal(x))
            per_motor_right = right_torque / 2

            if per_motor_right > 0 and per_motor_left > 0 or per_motor_right < 0 and per_motor_left < 0:
                per_motor_right /= 20
                per_motor_left  /= 20
            else:
                per_motor_left /= 5
                per_motor_right /= 5

            # if per_motor_left != 0 and per_motor_right == 0:
            #     per_motor_right = per_motor_left * 0.1
            
            # if per_motor_right != 0 and per_motor_left == 0:
            #     per_motor_left = per_motor_right * 0.1

            print("Right motor: ", per_motor_right)
            print("left motor: ", per_motor_left)
            print("-----")
            for m in left_drive_train:
                m.setForce(per_motor_left)
            for m in right_drive_train:
                m.setForce(per_motor_right)

            #print("---------")
            #print("max vel:", m.getMaxVelocity())
            #print("avail torque:", m.getAvailableTorque())
            #print("max torque:", m.getMaxTorque())
            #print("---------")      
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
