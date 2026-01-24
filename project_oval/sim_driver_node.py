import math

import rclpy
from std_msgs.msg import Bool

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    def init(self, webots_node, properties):

        self.current_speed = 1.0
        self.__robot = webots_node.robot

        self.__motor_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
        self.__motor_devices = []

        self.__time = 0.0
        self.__brake = False   # updated from /brake

        for motor_name in self.__motor_names:
            motor = self.__robot.getDevice(motor_name)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.__motor_devices.append(motor)

        self.set_all_speed(0.0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        # Subscribe to brake signal
        self.__brake_sub = self.__node.create_subscription(
            Bool,
            "/brake",
            self.__brake_callback,
            10
        )

    def set_all_speed(self, speed: float):
        for motor in self.__motor_devices:
            motor.setVelocity(speed)

    def __brake_callback(self, msg: Bool):
        self.__brake = msg.data
        self.__node.get_logger().info(f"Received brake={self.__brake}")

    def drive_oval(self, dt=0.032):
        # advance internal time
        self.__time += dt

        # Base forward speed (keep this modest so the circle is small)
        forward_speed = 4.0

        # Constant turn bias: this makes it follow a circle instead of a line
        turn_bias = 2.0     # bigger = tighter circle, smaller = larger circle

        # Small wobble on top of the bias so it "winds" a bit
        wiggle_amp = 0.7    # how strong the wiggle is
        wiggle_freq = 1.0   # how fast it wiggles

        wiggle = wiggle_amp * math.sin(self.__time * wiggle_freq)

        # Effective left/right speeds:
        #  - forward_speed pushes it forward
        #  - (turn_bias + wiggle) bends it into a circle with a small oscillation
        turn_component = turn_bias + wiggle

        left_speed = forward_speed - turn_component
        right_speed = forward_speed + turn_component

        # Apply to wheels: indices 0,2 = left; 1,3 = right
        for i, motor in enumerate(self.__motor_devices):
            if i % 2 == 0:   # left side
                motor.setVelocity(left_speed)
            else:            # right side
                motor.setVelocity(right_speed)


    def step(self):
        # process incoming ROS messages (including /brake)
        rclpy.spin_once(self.__node, timeout_sec=0.0)

        if self.__brake:
            # stop immediately
            self.set_all_speed(0.0)
            return

        # otherwise move in squiggly oval
        self.drive_oval()
