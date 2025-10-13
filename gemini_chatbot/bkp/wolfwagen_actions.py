import asyncio
from typing import Dict, List
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int64
import math
import numpy as np

class GeminiControlNode(Node):
    """ROS2 Node to control the Wolfwagen vehicle via Gemini Live Tools actions."""
    def __init__(self):
        super().__init__("gemini_control_node")
        self.throttle = 0
        self.steer = 0
        

        self.throttle_pub = self.create_publisher(Int64, '/gemini/throttle', 1)
        self.steer_pub = self.create_publisher(Int64, '/gemini/steer', 1)

async def turn_left() -> Dict[str, any]:
    """Simulates turning the vehicle left."""
    print("[Action] Starting to turn left...")
    await asyncio.sleep(2)  # Simulate a 2-second action
    result = {"status": "success", "summary": "Completed a 90-degree left turn."}
    print(f"[Action] Finished turning left. Result: {result}")
    return result


async def turn_right() -> Dict[str, any]:
    """Simulates turning the vehicle right."""
    print("[Action] Starting to turn right...")
    await asyncio.sleep(2)  # Simulate a 2-second action
    result = {"status": "success", "summary": "Completed a 90-degree right turn."}
    print(f"[Action] Finished turning right. Result: {result}")
    return result

async def go_straight() -> Dict[str, any]:
    """Simulates giving just throttle to go straight."""
    print("[Action] Giving throttle...")
    await asyncio.sleep(2)  # Simulate a 2-second action
    result = {"status": "success", "summary": "Moved straight ahead."}
    print(f"[Action] Finished moving straight ahead. Result: {result}")
    return result

# Tool/function declarations for Gemini Live Tools

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
                                    "enum": ["turn_left", "turn_right", "go_straight"],
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


# Name to handler mapping
async def execute_sequence(steps: List[Dict[str, any]]) -> Dict[str, any]:
    """Executes a sequence of primitive actions sequentially."""
    results: List[Dict[str, any]] = []
    action_map = {
        "turn_left": turn_left,
        "turn_right": turn_right,
        "go_straight": go_straight,
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


HANDLERS = {
    "turn_left": turn_left,
    "turn_right": turn_right,
    "go_straight": go_straight,
    "execute_sequence": execute_sequence,
}

# def main():
#     rclpy.init()
#     node = rclpy.create_node("wolfwagen_actions_node")
#     node.create_publisher(Int64, "/gemini_throttle", 1)
#     node.create_publisher(Int64, "/gemini_steer", 1)
    
#     thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     thread.start()

#     FREQ = 5
#     rate = node.create_rate(FREQ, node.get_clock())

#     node.get_logger().info("Wolfwagen Actions Node started.")

#     while rclpy.ok():
    
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("Shutting down Wolfwagen Actions Node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
