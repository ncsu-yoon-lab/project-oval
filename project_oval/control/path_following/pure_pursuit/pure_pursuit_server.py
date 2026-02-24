import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# Action server for high level management of pure pursuit algorithm.
# Hopefully, will allow us to monitor progress and start, pause, and stop pure pursuit in real time.
class PurePursuitServer(Node):
    def __init__(self):
        super().__init__("pure_pursuit_server")

        self._init_action_server()
        self._init_internal_state()

    def _init_action_server(self):
        pass

    def _init_internal_state(self):
        pass

    async def _execute_callback(self, goal_handle):
        pass

    def _goal_callback(self, goal_request):
        pass

    def _cancel_callback(self, goal_handle):
        pass

    def _publish_feedback(self, goal_handle):
        pass

    def _set_succeeded(self, goal_handle):
        pass

    def _set_aborted(self, goal_handle):
        pass


# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitServer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()