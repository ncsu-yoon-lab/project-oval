import rclpy
from rclpy.node import Node

from .pure_pursuit import PurePursuit
from .pure_pursuit import PathPoint2D

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Bool

from tf_transformations import euler_from_quaternion

from .pure_pursuit import PurePursuit
from .pure_pursuit_pid import PurePursuitPid


center_line_distance = 0.06  # meters


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.declare_parameters(namespace="", parameters=[("lookahead_distance", 1.0), ("operating_velocity", 5.0),],)

        self.x = None
        self.y = None
        self.yaw = None
        self.v = None

        self.left_wheel_omega = None
        self.right_wheel_omega = None

        self.pure_pursuit = None
        self.left_pid = None
        self.right_pid = None

        self.path = None
        self.path_points = None

        self.path_changed = False

        
        self.left_wheel_vel = None
        
        self.right_wheel_vel = None

        self.last_time = self.get_clock().now()

        self.init_controller()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()

    def init_controller(self):
        lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        operating_velocity = float(self.get_parameter("operating_velocity").value)
        self.pure_pursuit = PurePursuit(lookahead_distance, operating_velocity)

        self.right_pid = PurePursuitPid()
        self.left_pid = PurePursuitPid()
    

    def init_publishers(self):

        self.pp_left_throttle_pub = self.create_publisher(Float64, "/pure_pursuit/left_throttle", 10)
        self.pp_right_throttle_pub = self.create_publisher(Float64, "/pure_pursuit/right_throttle", 10)
        self.pp_use_throttle_pub = self.create_publisher(Bool, "/pure_pursuit/use_throttle", 10)
    

    def init_subscribers(self):
        
        self.create_subscription(Odometry,"/gps", self.gps_callback, 10)
        
        self.create_subscription(Float64, "/gps/speed", self.speed_callback, 10)
        
        
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.create_subscription(Path, "/pure_pursuit/path", self.path_callback, 10)

        self.create_subscription(Float64, "/left_wheel/angular_velocity", self.left_wheel_vel_callback, 10)
        self.create_subscription(Float64, "/right_wheel/angular_velocity", self.right_wheel_vel_callback, 10)
    
    def gps_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def speed_callback(self, msg: Float64):
        self.v = msg.data

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, self.omega_measured, self.yaw = euler_from_quaternion(quat)
    
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
        
        
        self.path_points = self.points

        self.path_changed = True

    def init_timers(self):
        self.control_timer = self.create_timer(0.02, self.control_loop)


    def control_loop(self):
        if self.pure_pursuit is None:
            print("[WARNING] Pure Pursuit Algorithm was not initialized")
            return
        
        if self.path is None or self.path_points is None:
            print("[WARNING] Pure Pursuit has not been provided a path")
            return
        
        odom_data = [self.x, self.y, self.yaw, self.v]

        if None in odom_data:
            print("[WARNING] Pure pursuit has not recieved odom data from sensors")
            return

        
        wheel_data = [self.left_wheel_vel, self.right_wheel_vel]
        
        if None in wheel_data:
            print("[WARNING] Pure pursuit has not recieved wheel velocity data")
            return

        if self.path_changed:
            self.pure_pursuit.init_path(self.path_points)
    
            self.path_changed = False
        
        robot_pose = PathPoint2D(self.x, self.y)
        
        desired_velocity = self.pure_pursuit.update_state(robot_pose, self.yaw, self.v)

        velocity_linear_desired = desired_velocity[0]
        omega_desired = desired_velocity[1]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        
        omega_right_desired = (velocity_linear_desired + center_line_distance * omega_desired)
        
        omega_left_desired  = (velocity_linear_desired - center_line_distance * omega_desired)

        error_right = omega_right_desired - self.right_wheel_vel
        error_left  = omega_left_desired  - self.left_wheel_vel

        throttle_right = self.right_pid.update(error_right, dt)
        throttle_left  = self.left_pid.update(error_left, dt)

        self.pp_use_throttle_pub.publish(Bool(data=True))
        self.pp_left_throttle_pub.publish(Float64(data=throttle_left))
        self.pp_right_throttle_pub.publish(Float64(data=throttle_right))
        


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()