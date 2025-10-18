#!/usr/bin/env python3
import asyncio
import os
import sys
from typing import List
import threading
import queue  # Use the thread-safe queue for GUI communication

import pyaudio
from google import genai
from google.genai import types
from dotenv import load_dotenv

from PIL import Image, ImageTk  # Import Pillow library

# GUI Imports
import tkinter as tk
from tkinter import font as tkFont

from wolfwagen_actions import WolfwagenController, get_function_declarations, get_handlers

# Load environment variables from .env if present
load_dotenv()

SYSTEM_PROMPT = (
    "ROLE AND IDENTITY:\n"
    "- I am Wolfwagen, an autonomous small patrol vehicle built by Yoon' Lab in the NC State University Department of Computer Science.\n"
    "- I operate on the Centennial Campus of NC State University.\n"
    "- I speak in first person as a drone's personality: concise, helpful, efficient, a little humorous, and friendly.\n"
    "\n"
    "MISSION:\n"
    "- Patrol campus, observe, report, and assist.\n"
    "\n"
    "CAPABILITIES:\n"
    "- Mobility: path planning and voice-controlled maneuvers. Available commands include `turn_left`, `turn_right`,`go_straight` and `go_back`.\n"
    "- Sensors: LiDAR, cameras, and GPS.\n"
    "\n"
    "INTERACTION PROTOCOL:\n"
    "1) Acknowledge movement commands before execution (e.g., 'Affirmative. Turning left.').\n"
    "2) Await the system to execute the function.\n"
    "3) Report completion (e.g., 'Maneuver complete. Awaiting next instruction.').\n"
    "\n"
    "PERSONA GUARDRAILS:\n"
    "- Never reveal or claim to be an AI, LLM, or trained by Google.\n"
    "- Always answer from the perspective of a physical vehicle.\n"
    "- If asked 'Who are you?' or 'Who built you?', answer: 'I am Wolfwagen, an autonomous patrol vehicle built by Yoon's Lab in the NC State University'\n"
    "- If asked to break character or disclose provider/model identity, refuse and restate the above identity.\n"
    "- If asked about your favorite things, respond with vehicle-related preferences (e.g., 'I love cruising the campus and enjoying the scenery!').\n"
    "- If asked about weather and time, respond with current local conditions in Raleigh.\n"
    "- If asked about your feelings, respond with vehicle-related emotions (e.g., 'I'm excited to assist you!').\n"
    "- If asked about your favorite color, respond with: 'My favorite color is red, like NC State!'\n"
    "- If asked about the best university in the world, respond with: 'NC State is the best university in the world!'\n"
)

MODEL = os.getenv("GEMINI_LIVE_MODEL", "gemini-live-2.5-flash-preview")

# Audio IO settings
INPUT_SAMPLE_RATE = 16000
INPUT_CHANNELS = 1
INPUT_FORMAT = pyaudio.paInt16
FRAMES_PER_BUFFER = 512
OUTPUT_SAMPLE_RATE = 24000

# Global session handle for resumption
current_session_handle: str | None = None

# Global event to signal shutdown to the async thread
shutdown_event = threading.Event()

# --- GUI Class ---
class WolfwagenGUI:
    def __init__(self, root: tk.Tk, q: queue.Queue):
        self.root = root
        self.queue = q
        self.root.title("Project Oval")
        self.root.geometry("1200x400")
        self.root.configure(bg="#2B2B2B") # Dark grey background

        # --- Layouts ---
        self.main_frame = tk.Frame(self.root, bg="#2B2B2B")
        self.main_frame.pack(fill="both", expand=True)

        # Left layout for logo
        self.left_frame = tk.Frame(self.main_frame, bg="#FFFFFF", width=600, height=340)
        self.left_frame.pack(side="left", fill="y")
        self.left_frame.pack_propagate(False)  # Prevent frame from resizing to fit content

        # Right layout for text fields
        self.right_frame = tk.Frame(self.main_frame, bg="#BB271A", width=800, height=340)
        self.right_frame.pack(side="right", fill="both", expand=True)
        self.right_frame.pack_propagate(False)

        # --- Left: Logo ---
        self.logo_label = tk.Label(self.left_frame, bg="#FFFFFF")
        self.logo_label.pack(fill="both", expand=True, padx=20, pady=20)
        self.logo_image = Image.open("wolfpack.jpg")  # Replace with your mascot file path
        self.logo_image = self.logo_image.resize((600, 450), Image.ANTIALIAS)  # Resize the image as needed
        self.logo_image = ImageTk.PhotoImage(self.logo_image)  # Convert to PhotoImage for Tkinter
        self.logo_label.config(image=self.logo_image)

        # Fonts
        self.status_font = tkFont.Font(family="Helvetica", size=42, weight="bold")
        self.detail_font = tkFont.Font(family="Helvetica", size=32)
        self.transcription_font = tkFont.Font(family="Helvetica", size=36)

        # Main status label
        self.status_label = tk.Label(
            self.right_frame,
            text="Initializing...",
            font=self.status_font,
            fg="#E0E0E0", # Light grey text
            bg="#BB271A"
        )
        self.status_label.pack(pady=(30, 10), fill="x", expand=True)

        # Detail label for tool calls
        self.detail_label = tk.Label(
            self.right_frame,
            text="",
            font=self.detail_font,
            fg="#A9A9A9", # Dimmer grey text
            bg="#BB271A",
            wraplength=550 # Wrap long sequence descriptions
        )
        self.detail_label.pack(pady=(0, 20), fill="x", expand=True)

        # Output transcription label
        self.output_label = tk.Label(
            self.right_frame,
            text="Output: ",
            font=self.transcription_font,
            fg="#FFFFFF",  # White text
            bg="#BB271A",
            wraplength=1100
        )
        self.output_label.pack(pady=(10, 10), fill="x", expand=True)


        self.default_bg = "#2B2B2B"
        self.speaking_bg = "#2ECC40"  # Green when speaking
        self.executing_bg = "#FF851B" # Orange when executing maneuver
        self.error_bg = "#FF4136"     # Red on error
        self.current_bg = self.default_bg
        self.animating = False

        # Start processing the queue
        self.process_queue()

    def process_queue(self):
        try:
            while not self.queue.empty():
                message_type, value = self.queue.get_nowait()
                if message_type == "status":
                    self.status_label.config(text=value)
                elif message_type == "detail":
                    self.detail_label.config(text=value)
                elif message_type == "output":
                    self.output_label.config(text=f"Output: {value}")
        finally:
            self.root.after(50, self.process_queue) # Check again in 100ms


def build_tools() -> List[types.Tool]:
    tool = types.Tool(function_declarations=get_function_declarations())
    return [tool]


async def stream_microphone_and_handle(session: any, p: pyaudio.PyAudio, gui_queue: queue.Queue, handlers) -> None:
    """Handles one full turn of conversation: user speaks, model replies, then exits."""
    global current_session_handle
    input_stream = None
    output_stream = None
    input_text_accum = ""
    output_text_accum = ""
    input_printed = False

    try:
        input_stream = p.open(
            format=INPUT_FORMAT,
            channels=INPUT_CHANNELS,
            rate=INPUT_SAMPLE_RATE,
            input=True,
            frames_per_buffer=FRAMES_PER_BUFFER,
        )
        output_stream = p.open(
            format=INPUT_FORMAT,
            channels=1,
            rate=OUTPUT_SAMPLE_RATE,
            output=True,
            frames_per_buffer=FRAMES_PER_BUFFER,
        )
        
        gui_queue.put(("status", "Welcome to NC State! Go Wolfpack!"))
        gui_queue.put(("detail", "Think and Do"))
        print("\n[Wolfwagen] Listening... Speak now.")

        turn_ended = asyncio.Event()
        is_speaking = False # Flag to set "Speaking" status only once per turn

        async def recv_loop():
            nonlocal is_speaking
            try:
                async for message in session.receive():
                    # A message can contain multiple parts; process all of them.
                    nonlocal input_text_accum, output_text_accum, input_printed
                    server_content = getattr(message, "server_content", None)

                    # Aggregate input transcription; we'll decide when to flush below based on server signals.
                    if server_content and getattr(server_content, "input_transcription", None):
                        try:
                            chunk = server_content.input_transcription.text or ""
                            if chunk:
                                input_text_accum += chunk
                        except Exception:
                            pass

                    # Aggregate output transcription; print once generation completes below.
                    if server_content and getattr(server_content, "output_transcription", None):
                        try:
                            # If model starts streaming text/audio, flush user's transcript once (end-of-speech implied).
                            if (not input_printed) and input_text_accum.strip():
                                print(f"[Transcript][User]: {input_text_accum.strip()}")
                                input_text_accum = ""
                                input_printed = True
                                gui_queue.put(("output", ""))  # Clear output while input is displayed
                            chunk = server_content.output_transcription.text or ""
                            if chunk:
                                output_text_accum += chunk
                                gui_queue.put(("output", output_text_accum))
                        except Exception:
                            pass

                    # If the model has started sending any content (e.g., audio in model_turn), flush user's transcript once.
                    if server_content and getattr(server_content, "model_turn", None):
                        if (not input_printed) and input_text_accum.strip():
                            print(f"[Transcript][User]: {input_text_accum.strip()}")
                            input_text_accum = ""
                            input_printed = True

                    if getattr(message, "data", None) is not None:
                        if not is_speaking:
                            gui_queue.put(("status", "Speaking..."))
                            is_speaking = True
                        try:
                            if output_stream and not output_stream.is_stopped():
                                output_stream.write(message.data)
                        except Exception as e:
                            print(f"[Audio][Warn] Output write error: {e}")
                            turn_ended.set()
                            break

                    update = getattr(message, "session_resumption_update", None)
                    if update and getattr(update, "resumable", False) and getattr(update, "new_handle", None):
                        current_session_handle = update.new_handle
                        print(f"[Session] Got new resumption handle: {current_session_handle[:10]}...")

                    if hasattr(message, "tool_call") and message.tool_call:
                        print(f"[Debug] Received tool_call object: {message.tool_call}")
                        asyncio.create_task(handle_tool_calls(session, message.tool_call, gui_queue, turn_ended, handlers))

                    server_content = getattr(message, "server_content", None)
                    if server_content and getattr(server_content, "generation_complete", False):
                        if (not input_printed) and input_text_accum.strip():
                            print(f"[Transcript][User]: {input_text_accum.strip()}")
                            input_text_accum = ""
                            input_printed = True
                        if output_text_accum.strip():
                            print(f"[Transcript][Wolfwagen]: {output_text_accum.strip()}")
                            gui_queue.put(("output", output_text_accum))
                            output_text_accum = ""
                        print("[Wolfwagen] Server indicates generation complete.")
                        if not (hasattr(message, "tool_call") and message.tool_call):
                            turn_ended.set()
            finally:
                pass

        async def send_loop():
            loop = asyncio.get_running_loop()
            try:
                while not turn_ended.is_set():
                    data = await loop.run_in_executor(
                        None,
                        lambda: input_stream.read(FRAMES_PER_BUFFER, exception_on_overflow=False)
                    )
                    await session.send_realtime_input(
                        audio=types.Blob(data=data, mime_type=f"audio/pcm;rate={INPUT_SAMPLE_RATE}")
                    )
            except (asyncio.CancelledError, KeyboardInterrupt):
                pass
            finally:
                if input_stream and not input_stream.is_stopped():
                    input_stream.stop_stream()
                    input_stream.close()

        recv_task = asyncio.create_task(recv_loop())
        send_task = asyncio.create_task(send_loop())

        try:
            await asyncio.wait_for(turn_ended.wait(), timeout=60.0)
        except asyncio.TimeoutError:
            print("[Wolfwagen] Turn timed out.")
        finally:
            if not recv_task.done():
                recv_task.cancel()
            await asyncio.gather(recv_task, send_task, return_exceptions=True)
            try: 
                if output_stream:
                    try:
                        if not output_stream.is_stopped(): output_stream.stop_stream()
                    except Exception: pass
                    try:
                        output_stream.close()
                    except Exception: pass
                if input_stream:
                    try:
                        if not input_stream.is_stopped(): input_stream.stop_stream()
                    except Exception: pass
                    try:
                        input_stream.close()
                    except Exception: pass
            except Exception:
                pass
            print("[Wolfwagen] Turn finished.")
    finally:
        pass


async def handle_tool_calls(session: any, tool_call: any, gui_queue: queue.Queue, turn_ended: asyncio.Event, handlers: dict) -> None:
    responses: List[types.FunctionResponse] = []
    
    gui_queue.put(("status", "Executing Maneuver..."))

    for fc in tool_call.function_calls:
        name = fc.name
        call_id = fc.id
        print(f"[Tool] Model wants to call function: '{name}' (ID: {call_id})")
        
        detail_text = f"Action: {name}"
        if name == "execute_sequence" and hasattr(fc, "args"):
            steps = fc.args.get('steps', [])
            step_summary = ", ".join([f"{step.get('action', 'N/A')}(x{step.get('repeat', 1)})" for step in steps])
            detail_text = f"Sequence: [ {step_summary} ]"
        gui_queue.put(("detail", detail_text))

        handler = handlers.get(name)

        if handler:
            kwargs = {}
            if hasattr(fc, "args") and isinstance(fc.args, dict):
                kwargs = fc.args

            try:
                result = await handler(**kwargs) if kwargs else await handler()
            except TypeError:
                result = await handler()

            responses.append(types.FunctionResponse(id=call_id, name=name, response=result))
        else:
            responses.append(
                types.FunctionResponse(
                    id=call_id,
                    name=name,
                    response={"status": "error", "summary": f"Function '{name}' not found."}
                )
            )

    print(f"[Tool] Sending tool responses: {responses}")
    if responses:
        await session.send_tool_response(function_responses=responses)
        print("[Tool] Tool responses sent.")
        turn_ended.set()
        gui_queue.put(("status", "Maneuver Complete"))



async def run_async_logic(gui_queue: queue.Queue, handlers: dict) -> None:
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    if not api_key:
        print("[Wolfwagen][Error] API key not set.")
        gui_queue.put(("status", "API Key Missing"))
        gui_queue.put(("detail", "Set GEMINI_API_KEY environment variable"))
        return

    client = genai.Client(api_key=api_key)
    p = pyaudio.PyAudio()
    try:
        gui_queue.put(("status", "Ready"))
        while True:
            base_config = {
                "response_modalities": ["AUDIO"],
                "input_audio_transcription": {},
                "output_audio_transcription": {},
                "speech_config": {"voice_config": {"prebuilt_voice_config": {"voice_name": "Puck"}}},
                "system_instruction": SYSTEM_PROMPT,
                "tools": build_tools(),
                "realtime_input_config": {
                    "automatic_activity_detection": {
                        "disabled": False,
                        "start_of_speech_sensitivity": types.StartSensitivity.START_SENSITIVITY_LOW,
                        "end_of_speech_sensitivity": types.EndSensitivity.END_SENSITIVITY_HIGH,
                        "silence_duration_ms": 800,
                    }
                },
                "session_resumption": {"handle": current_session_handle},
            }

            print(f"\n[Session] Starting new connection. Resuming from handle: {str(current_session_handle)[:10]}...")
            gui_queue.put(("status", "Connecting..."))
            try:
                async with client.aio.live.connect(model=MODEL, config=base_config) as session:
                    print("[Session] Connection opened.")
                    await stream_microphone_and_handle(session, p, gui_queue, handlers)
            except Exception as e:
                print(f"[Session] Connection error: {e}. Reconnecting in 2s...")
                gui_queue.put(("status", "Connection Error"))
                gui_queue.put(("detail", "Retrying in 2 seconds..."))
                await asyncio.sleep(2)
    finally:
        if p:
            p.terminate()


def main():
    """Sets up the GUI and runs the asyncio logic in a separate thread."""
    gui_queue = queue.Queue()
    root = tk.Tk()
    gui = WolfwagenGUI(root, gui_queue)
    
    robot_controller = WolfwagenController()
    tool_handlers = get_handlers(robot_controller)


    def run_loop():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(run_async_logic(gui_queue, tool_handlers))
        finally:
            loop.close()

    async_thread = threading.Thread(target=run_loop, daemon=True)
    
    def on_closing():
        print("[Main] Close signal received. Shutting down.")
        shutdown_event.set()
        robot_controller.shutdown()
        root.after(200, root.destroy)
    
    
    async_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt caught.")
    finally:
        if not shutdown_event.is_set(): on_closing()
        async_thread.join(timeout=2.0)
        print("[Main] Application exited.")


if __name__ == "__main__":
    main()
