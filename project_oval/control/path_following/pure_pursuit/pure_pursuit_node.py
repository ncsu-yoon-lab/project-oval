import rclpy
from rclpy.node import Node

from .pure_pursuit import PurePursuit
from .pure_pursuit import PathPoint2D

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# https://docs.ros.org/en/humble/p/tf_transformations/
from tf_transformations import euler_from_quaternion

# import the pure pursuit algorithm.
from .pure_pursuit import PurePursuit

# Import PID controller for pure pursuit.
from .pure_pursuit_pid import PurePursuitPid

# This node is written to work with the simulation. I do not yet know how subscription names will differ when run on the car.
# We will figure that out, should not require major refactoring.

# distance from center of wheel to, robot center line

# THIS IS THE PHYSICAL CAR DIMENSION FOR CENTER LINE DISTANCE
############
# center_line_distance = 9.5 # in
# center_line_distance = center_line_distance * 0.0254 # meters
################

# THIS IS THE SIM CAR DIMENSION FOR CENTER LINE DISTANCE
##############
center_line_distance = 0.06  # meters
#############



class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        # TODO: Added GPS and IMU to sim robot, subscribe to those and use them to provide state estimation to pure pursuit algorithm
        # lookahead will be modifiable parameters.
        self.declare_parameters(namespace="", parameters=[("lookahead_distance", 1.0), ("operating_velocity", 5.0),],)

        # State parameters for pure pursuit
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None

        self.left_wheel_omega = None
        self.right_wheel_omega = None

        self.pure_pursuit = None
        self.pid_left = None
        self.pid_right = None

        self.path = None
        self.path_points = None

        # For dynamic path changing
        self.path_changed = False

        # this is for dt (change in time) for 
        self.last_time = self.get_clock().now()

        self.init_controller()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()

    def init_controller(self):
        lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        operating_velocity = float(self.get_parameter("operating_velocity").value)
        self.pure_pursuit = PurePursuit(lookahead_distance, operating_velocity)

        # optionally add paramters for he kp, ki, and kd constants in PID
        self.right_pid = PurePursuitPid()
        self.left_pid = PurePursuitPid()
    

    def init_publishers(self):

        self.pp_left_throttle_pub = self.create_publisher(Float64, "/pure_pursuit/left_throttle", 10)
        self.pp_right_throttle_pub = self.create_publisher(Float64, "/pure_pursuit/right_throttle", 10)
        self.pp_use_throttle_pub = self.create_publisher(Bool, "/pure_pursuit/use_throttle", 10)
    

    def init_subscribers(self):
        # GPS pose and speed data
        self.create_subscription(Odometry,"/gps", self._gps_callback, 10)
        self.create_subscription(Float64, "/gps/speed", self._speed_callback, 10)
        
        # IMU data
        self.create_subscription(Imu, "/imu", self._imu_callback, 10)

        # callback for path
        self.create_subscription(Path, "/pure_pursuit/path", self.path_callback, 10)

        # measured angular velocity
        self.create_subscription(Float64, "/left_wheel/angular_velocity", self.left_wheel_vel_callback, 10)
        self.create_subscription(Float64, "/right_wheel/angular_velocity", self.right_wheel_vel_callback, 10)
    
    # Callback for gps data
    def gps_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # Callback for speed data
    def speed_callback(self, msg: Float64):
        self.v = msg.data

    # Callback for imu data
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]

        # Data recieved from sim GPS is a quaternion, need to convert it to aquire yaw.
        # https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_quaternion
        _, self.omega_measured, self.yaw = euler_from_quaternion(quat)
    
    # callbacks for measured angular velocity
    def left_wheel_vel_callback(self, msg: Float64):
        self.left_wheel_vel = msg.data

    def right_wheel_vel_callback(self, msg: Float64):
        self.right_wheel_vel = msg.data

    def path_callback(self, msg):
        self.path = msg
        self.points = []

        for pose_stamped in self.path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.points.append((x, y))
        
        self.path_changed = True

    def init_timers(self):
        # 50 hz control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)


    def control_loop(self):
        # havent written the pure pursuit algorithm to handle changes in these values during runtime, will update soon.
        # lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        # operating_velocity = float(self.get_parameter("operating_velocity").value)
        if self.pure_pursuit is None:
            print("[WARNING] Pure Pursuit Algorithm was not initialized")
            return
        
        # for this scenario, we could optionally shut down publishers and subscribers when pure pursuit is not being used
        if self.path is None or self.path_points is None:
            print("[WARNING] Pure Pursuit has not been provided a path")
            return
        
        odom_data = [self.x, self.y, self.yaw, self.v]

        if None in odom_data:
            print("[WARNING] Pure pursuit has not recieved odom data from sensors")
            return

        # if the path has been added/changed, all the init path method on the path points
        if self.path_changed:
            self.pure_pursuit.init_path(self.path_points)
        
        # For future implementations, we can improve accuracy of path following by:
        # TODO: check timestamps to make sure ODOM data is all recieved within a certain timeframe
        # this is because the gps may update at a faster rate than the IMU. For initial impl. this should 
        # not be a huge problem.
        robot_pose = PathPoint2D(self.x, self.y)
        desired_velocity = self.pure_pursuit.update_state(robot_pose, self.yaw, self.velocity)

        velocity_linear_desired = desired_velocity[0]
        omega_desired = desired_velocity[1]

        # TODO: I need to split this into two angular velocitys for the left and right drivetrain
        # This will require wheel encoders on the car, need to check if we have those.
        # Calculate dt, it should roughly equal the 1/frequency of the controol loop timer
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        omega_right_desired = (velocity_linear_desired + center_line_distance * omega_desired) / WHEEL_RADIUS
        omega_left_desired  = (velocity_linear_desired - center_line_distance * omega_desired) / WHEEL_RADIUS

        error_right = omega_right_desired - self.right_wheel_vel # rad/s
        error_left  = omega_left_desired  - self.left_wheel_vel # rad/s

        throttle_right = self.right_pid.update(error_right, dt)
        throttle_left  = self.left_pid.update(error_left, dt)

        # Publish throttle data to 
        self.pp_use_throttle_pub.publish(Bool(data=True))
        self.pp_left_throttle_pub.publish(Float64(data=throttle_left))
        self.pp_right_throttle_pub.publish(Float64(data=throttle_right))
        


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()