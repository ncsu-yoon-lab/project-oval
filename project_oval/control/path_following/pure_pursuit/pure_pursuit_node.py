import rclpy
from rclpy.node import Node

from .pure_pursuit import PurePursuit

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# https://docs.ros.org/en/humble/p/tf_transformations/
from tf_transformations import euler_from_quaternion

# This node is written to work with the simulation. I do not yet know how subscription names will differ when run on the car.
# We will figure that out, should not require major refactoring.

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        # TODO: Added GPS and IMU to sim robot, subscribe to those and use them to provide state estimation to pure pursuit algorithm
        # lookahead will be modifiable parameters.
        self.declare_parameters(namespace="", parameters=[("lookahead_distance", 1.0),],)

        # State parameters for pure pursuit
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None


        self._init_controller()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

    def _init_controller(self):
        pass

    def _init_publishers(self):
        pass
    

    def _init_subscribers(self):
        # GPS pose and speed data
        self.create_subscription(Odometry,"/gps", self._gps_callback, 10)
        self.create_subscription(Float64, "/gps/speed", self._speed_callback, 10)

        # IMU data
        self.create_subscription(Imu, "/imu", self._imu_callback, 10)
    
    # Callback for gps data
    def _gps_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # Callback for speed data
    def _speed_callback(self, msg: Float64):
        self.v = msg.data

    # Callback for imu data
    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]

        # Data recieved from sim GPS is a quaternion, need to convert it to aquire yaw.
        # https://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_quaternion
        _, _, self.yaw = euler_from_quaternion(quat)

    def _init_timers(self):
        pass

    def _path_callback(self, msg):
        pass

    def _state_callback(self, msg):
        pass

    def _control_loop(self):
        pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()