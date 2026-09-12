import asyncio
from typing import Dict, List, Optional
import queue
import threading

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

# --- The Self-Contained Robot Controller ---

class WolfwagenController:
    """
    Manages all ROS 2 communications for the Wolfwagen vehicle.
    This class initializes a ROS 2 node, spins it in a separate thread,
    and provides async methods to control the robot's movement.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._ros_thread = None
        self._node = None
        
        # Initialize rclpy and create the node
        rclpy.init()
        self._node = Node('wolfwagen_controller')
        self._throttle_publisher = self._node.create_publisher(Int64, '/gemini/throttle', 1)
        self._steering_publisher = self._node.create_publisher(Int64, '/gemini/steering', 1)

        # Start the ROS 2 event loop in a daemon thread
        self._ros_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._ros_thread.start()
        self._node.get_logger().info('ROS 2 spin thread started.')

    def _publish_command(self, throttle: float, steering: float):
        """Internal method to publish a Twist message in a thread-safe manner."""
        with self._lock:
            throttle_msg = Int64()
            steering_msg = Int64()
            throttle_msg.data = throttle  
            steering_msg.data = steering  
            self._throttle_publisher.publish(throttle_msg)
            self._steering_publisher.publish(steering_msg)
            self._node.get_logger().info(f'Publishing: throttle={throttle}, steering={steering}')

    # --- Public Async Methods (The Tool Actions) ---

    async def turn_left(self) -> Dict[str, any]:
        """Publishes a command to turn the vehicle left."""
        
        self._publish_command(throttle=0, steering=-20)
        await asyncio.sleep(2.0)
        self._publish_command(throttle=0, steering=0) # Stop
        
        return {"status": "success", "summary": "Completed a 90-degree left turn."}

    async def turn_right(self) -> Dict[str, any]:
        """Publishes a command to turn the vehicle right."""

        self._publish_command(throttle=0, steering=20)
        await asyncio.sleep(2.0)
        self._publish_command(throttle=0, steering=0) # Stop
        
        return {"status": "success", "summary": "Completed a 90-degree right turn."}
    
    async def go_straight(self) -> Dict[str, any]:
        """Publishes a command to go straight"""
        
        self._publish_command(throttle=17.5, steering=0)
        await asyncio.sleep(2.0)
        self._publish_command(throttle=0, steering=0) # Stop
        
        return {"status": "success", "summary": "Going straight for 2 seconds."}

    async def go_back(self) -> Dict[str, any]:
        """Publishes a command to go back"""
        
        self._publish_command(throttle=-17.5, steering=0)
        await asyncio.sleep(2.0)
        self._publish_command(throttle=0, steering=0) # Stop
        
        return {"status": "success", "summary": "Going back for 2 seconds."}
    
    async def execute_sequence(self, steps: List[Dict[str, any]]) -> Dict[str, any]:
        """Executes a sequence of primitive actions sequentially."""
        results: List[Dict[str, any]] = []
        action_map = {
            "turn_left": self.turn_left,
            "turn_right": self.turn_right,
            "go_straight": self.go_straight,
            "go_back": self.go_back,
        }

        for step in steps:
            action_name = step.get("action")
            if action_name not in action_map:
                results.append({
                    "status": "error",
                    "summary": f"Unknown action: {action_name}",
                })
                continue

            repeats = step.get("repeat", 1)
            if not isinstance(repeats, int) or repeats < 1:
                repeats = 1

            for _ in range(repeats):
                handler = action_map[action_name]
                res = await handler()
                results.append(res)

        return {
            "status": "success",
            "summary": "Sequence executed.",
            "details": results,
        }


    def shutdown(self):
        """Cleanly shuts down the ROS 2 node and rclpy."""
        self._node.get_logger().info('Shutdown signal received.')
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._ros_thread.join(timeout=2.0)
        print("[Controller] ROS 2 shutdown complete.")

# --- Gemini Live Tool Integration ---

# Tool function declarations remain the same
def get_function_declarations() -> List[dict]:
        return [
            {
                "name": "turn_left",
                "description": "Turn the Wolfwagen vehicle to the left.",
                "parameters": {
                    "type": "object",
                    "properties": {},
                },
            },
            {
                "name": "turn_right",
                "description": "Turn the Wolfwagen vehicle to the right.",
                "parameters": {
                    "type": "object",
                    "properties": {},
                },
            },
            {
                "name": "go_straight",
                "description": "Apply throttle and go straight ahead.",
                "parameters": {
                    "type": "object",
                    "properties": {},
                },
            },
            {
                "name": "go_back",
                "description": "Apply throttle and go back.",
                "parameters": {
                    "type": "object",
                    "properties": {},
                },
            },
            {
                "name": "execute_sequence",
                "description": "Execute a sequence of primitive maneuvers (e.g., two left turns for a U-turn).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "steps": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "action": {
                                        "type": "string",
                                        "enum": ["turn_left", "turn_right", "go_straight", "go_back"],
                                        "description": "Primitive maneuver to execute"
                                    },
                                    "repeat": {
                                        "type": "integer",
                                        "minimum": 1,
                                        "default": 1,
                                        "description": "How many times to repeat the action"
                                    }
                                },
                                "required": ["action"]
                            },
                            "description": "Ordered list of steps to execute"
                        }
                    },
                    "required": ["steps"]
                },
            },
        ]
    

# This function links the tool names to the controller's methods
def get_handlers(controller_instance: WolfwagenController) -> Dict:
    return {
        "turn_left": controller_instance.turn_left,
        "turn_right": controller_instance.turn_right,
        "go_straight": controller_instance.go_straight,
        "go_back": controller_instance.go_back,
        "execute_sequence": controller_instance.execute_sequence,
    }
