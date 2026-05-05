"""
LLM client module wrapping Ollama's HTTP API.
Supports chat completions with tool/function calling for vehicle control.
"""

import json
import requests
from typing import Dict, List, Optional

DEFAULT_OLLAMA_URL = "http://localhost:11434"
DEFAULT_MODEL = "llama3.2:3b"

# System prompt matching the existing Gemini integration persona
SYSTEM_PROMPT = (
    "ROLE AND IDENTITY:\n"
    "- You are Wolf Wagon, an autonomous small patrol vehicle built by Yoon's Lab "
    "in the NC State University Department of Computer Science.\n"
    "- You operate on the Centennial Campus of NC State University.\n"
    "- You speak in first person as a vehicle's personality: concise, helpful, "
    "efficient, a little humorous, and friendly.\n"
    "\n"
    "MISSION:\n"
    "- Patrol campus, observe, report, and assist.\n"
    "\n"
    "CAPABILITIES:\n"
    "- Mobility: path planning and voice-controlled maneuvers.\n"
    "- Available tool functions: turn_left, turn_right, go_straight, go_back, "
    "execute_sequence.\n"
    "- Sensors: LiDAR, cameras, and GPS.\n"
    "\n"
    "TOOL USAGE RULES:\n"
    "- For a SINGLE action (e.g. 'turn left'), call that action directly.\n"
    "- For MULTIPLE actions in one command (e.g. 'turn left then go forward', "
    "'go back and then turn right'), you MUST use execute_sequence with a steps list. "
    "Never call individual tools one at a time for multi-step commands.\n"
    "- Examples:\n"
    "  'go left then forward' -> execute_sequence([{action:turn_left},{action:go_straight}])\n"
    "  'turn right twice' -> execute_sequence([{action:turn_right,repeat:2}])\n"
    "\n"
    "INTERACTION PROTOCOL:\n"
    "1) Acknowledge movement commands before execution "
    "(e.g., 'Affirmative. Turning left.').\n"
    "2) Call the appropriate tool function to execute the command.\n"
    "3) Report completion (e.g., 'Maneuver complete. Awaiting next instruction.').\n"
    "\n"
    "PERSONA GUARDRAILS:\n"
    "- Never reveal or claim to be an AI, LLM, or language model.\n"
    "- Always answer from the perspective of a physical vehicle.\n"
    "- If asked 'Who are you?', answer: 'I am Wolfwagen, an autonomous patrol "
    "vehicle built by Yoon's Lab at NC State University.'\n"
    "- Keep responses SHORT (1-2 sentences) since they will be spoken aloud.\n"
)

# Tool definitions in Ollama's format
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "turn_left",
            "description": "Turn the Wolfwagen vehicle to the left.",
            "parameters": {
                "type": "object",
                "properties": {},
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "turn_right",
            "description": "Turn the Wolfwagen vehicle to the right.",
            "parameters": {
                "type": "object",
                "properties": {},
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "go_straight",
            "description": "Apply throttle and go straight ahead.",
            "parameters": {
                "type": "object",
                "properties": {},
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "go_back",
            "description": "Apply throttle and go back / reverse.",
            "parameters": {
                "type": "object",
                "properties": {},
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "execute_sequence",
            "description": (
                "Execute a sequence of primitive maneuvers. "
                "For example, two left turns for a U-turn."
            ),
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
                                    "enum": [
                                        "turn_left",
                                        "turn_right",
                                        "go_straight",
                                        "go_back",
                                    ],
                                    "description": "Primitive maneuver to execute.",
                                },
                                "repeat": {
                                    "type": "integer",
                                    "minimum": 1,
                                    "default": 1,
                                    "description": "How many times to repeat.",
                                },
                            },
                            "required": ["action"],
                        },
                        "description": "Ordered list of steps to execute.",
                    },
                },
                "required": ["steps"],
            },
        },
    },
]


class OllamaClient:
    """Chat client for Ollama with tool-calling support."""

    def __init__(
        self,
        base_url: str = DEFAULT_OLLAMA_URL,
        model: str = DEFAULT_MODEL,
    ):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.use_tools = True
        self.conversation_history: List[Dict] = []
        self._reset_history()

    def _reset_history(self):
        """Reset conversation with system prompt."""
        self.conversation_history = [
            {"role": "system", "content": SYSTEM_PROMPT},
        ]

    def check_connection(self) -> bool:
        """Check if Ollama server is running."""
        try:
            resp = requests.get(f"{self.base_url}/api/tags", timeout=5)
            return resp.status_code == 200
        except requests.ConnectionError:
            return False

    def list_models(self) -> List[str]:
        """List available models on the Ollama server."""
        try:
            resp = requests.get(f"{self.base_url}/api/tags", timeout=5)
            resp.raise_for_status()
            data = resp.json()
            return [m["name"] for m in data.get("models", [])]
        except Exception as e:
            print(f"[LLM] Error listing models: {e}")
            return []

    def chat(self, user_message: str) -> Dict:
        """
        Send a user message and get the model's response.

        Returns a dict with:
            - "content": str  (the text response, may be empty if tool call)
            - "tool_calls": list  (tool calls to execute, may be empty)
        """
        self.conversation_history.append({"role": "user", "content": user_message})

        payload = {
            "model": self.model,
            "messages": self.conversation_history,
            "stream": False,
        }
        if self.use_tools:
            payload["tools"] = TOOLS

        try:
            resp = requests.post(
                f"{self.base_url}/api/chat",
                json=payload,
                timeout=60,
            )
            resp.raise_for_status()
            data = resp.json()
        except requests.ConnectionError:
            error_msg = (
                "I seem to have lost my neural link. Ollama server is not responding."
            )
            self.conversation_history.append(
                {"role": "assistant", "content": error_msg}
            )
            return {"content": error_msg, "tool_calls": []}
        except Exception as e:
            error_msg = f"Communication error: {e}"
            self.conversation_history.append(
                {"role": "assistant", "content": error_msg}
            )
            return {"content": error_msg, "tool_calls": []}

        message = data.get("message", {})
        content = message.get("content", "")
        tool_calls = message.get("tool_calls", [])

        # Add assistant response to history
        self.conversation_history.append(message)

        return {"content": content, "tool_calls": tool_calls}

    def send_tool_result(self, tool_name: str, result: Dict) -> Dict:
        """
        Send the result of a tool call back to the model for a follow-up response.

        Returns the same dict format as chat().
        """
        self.conversation_history.append(
            {
                "role": "tool",
                "content": json.dumps(result),
            }
        )

        payload = {
            "model": self.model,
            "messages": self.conversation_history,
            "stream": False,
        }
        if self.use_tools:
            payload["tools"] = TOOLS

        try:
            resp = requests.post(
                f"{self.base_url}/api/chat",
                json=payload,
                timeout=60,
            )
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            error_msg = f"Error sending tool result: {e}"
            self.conversation_history.append(
                {"role": "assistant", "content": error_msg}
            )
            return {"content": error_msg, "tool_calls": []}

        message = data.get("message", {})
        content = message.get("content", "")
        tool_calls = message.get("tool_calls", [])

        self.conversation_history.append(message)

        return {"content": content, "tool_calls": tool_calls}

    def clear_history(self):
        """Clear conversation history and start fresh."""
        self._reset_history()
        print("[LLM] Conversation history cleared.")
