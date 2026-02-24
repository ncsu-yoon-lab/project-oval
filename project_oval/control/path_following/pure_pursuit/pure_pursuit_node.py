import rclpy
from rclpy.node import Node

from .pure_pursuit import PurePursuit


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        # TODO: Added GPS and IMU to sim robot, subscribe to those and use them to provide state estimation to pure pursuit algorithm
        # lookahead will be modifiable parameters.
        self.declare_parameters(namespace="", parameters=[("lookahead_distance", 1.0),],)

        self._init_controller()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

    def _init_controller(self):
        pass

    def _init_publishers(self):
        pass

    def _init_subscribers(self):
        pass

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