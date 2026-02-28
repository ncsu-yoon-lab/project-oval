"""
ROS2 bridge for the local voice assistant.
Reuses the same motor control logic and ROS2 topics as the Gemini chatbot.
Publishes to /gemini/throttle and /gemini/steering so driver_node.py works unchanged.
"""

import asyncio
import threading
from typing import Dict, List

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class WolfwagenController:
    """
    Manages ROS2 communications for vehicle motor control.
    Publishes to the same topics as the Gemini chatbot integration.
    """

    def __init__(self):
        self._lock = threading.Lock()

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("wolfwagen_local_llm_controller")
        self._throttle_pub = self._node.create_publisher(
            Int64, "/gemini/throttle", 1
        )
        self._steering_pub = self._node.create_publisher(
            Int64, "/gemini/steering", 1
        )

        self._ros_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True
        )
        self._ros_thread.start()
        self._node.get_logger().info("Local LLM controller ROS2 node started.")

    def _publish(self, throttle: int, steering: int):
        """Publish motor commands in a thread-safe manner."""
        with self._lock:
            t_msg = Int64()
            s_msg = Int64()
            t_msg.data = throttle
            s_msg.data = steering
            self._throttle_pub.publish(t_msg)
            self._steering_pub.publish(s_msg)
            self._node.get_logger().info(
                f"Published: throttle={throttle}, steering={steering}"
            )

    async def turn_left(self) -> Dict:
        self._publish(throttle=0, steering=-20)
        await asyncio.sleep(2.0)
        self._publish(throttle=0, steering=0)
        return {"status": "success", "summary": "Completed left turn."}

    async def turn_right(self) -> Dict:
        self._publish(throttle=0, steering=20)
        await asyncio.sleep(2.0)
        self._publish(throttle=0, steering=0)
        return {"status": "success", "summary": "Completed right turn."}

    async def go_straight(self) -> Dict:
        self._publish(throttle=-20, steering=0)
        await asyncio.sleep(2.0)
        self._publish(throttle=0, steering=0)
        return {"status": "success", "summary": "Went straight for 2 seconds."}

    async def go_back(self) -> Dict:
        self._publish(throttle=20, steering=0)
        await asyncio.sleep(2.0)
        self._publish(throttle=0, steering=0)
        return {"status": "success", "summary": "Went back for 2 seconds."}

    async def execute_sequence(self, steps: List[Dict]) -> Dict:
        action_map = {
            "turn_left": self.turn_left,
            "turn_right": self.turn_right,
            "go_straight": self.go_straight,
            "go_back": self.go_back,
        }
        results = []
        for step in steps:
            action_name = step.get("action")
            if action_name not in action_map:
                results.append({"status": "error", "summary": f"Unknown: {action_name}"})
                continue
            repeats = max(1, int(step.get("repeat", 1)))
            for _ in range(repeats):
                res = await action_map[action_name]()
                results.append(res)

        return {"status": "success", "summary": "Sequence executed.", "details": results}

    def shutdown(self):
        self._node.get_logger().info("Shutting down local LLM controller.")
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._ros_thread.join(timeout=2.0)
        print("[Controller] ROS2 shutdown complete.")


def get_tool_handlers(controller: WolfwagenController) -> Dict:
    """Return a mapping of tool names to async handler functions."""
    return {
        "turn_left": controller.turn_left,
        "turn_right": controller.turn_right,
        "go_straight": controller.go_straight,
        "go_back": controller.go_back,
        "execute_sequence": controller.execute_sequence,
    }
