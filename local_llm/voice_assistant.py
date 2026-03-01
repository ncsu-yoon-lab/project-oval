#!/usr/bin/env python3
"""
Wolfwagen Local Voice Assistant
Main loop: VAD -> Whisper STT -> Ollama LLM -> Piper TTS -> Speaker

All processing runs locally on the Jetson Orin.
No cloud APIs required.
"""

import asyncio
import threading
import queue
import sys
import os
import numpy as np
import pyaudio

# GUI
import tkinter as tk
from tkinter import font as tkFont

# Local modules
from vad import VoiceActivityDetector
from stt import SpeechToText
from tts import TextToSpeech
from llm_client import OllamaClient

# Try to import ROS2 bridge; skip if ROS2 is not available
try:
    from wolfwagen_actions import WolfwagenController, get_tool_handlers

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[Main] ROS2 not available. Running without motor control.")

# Audio playback config
TTS_SAMPLE_RATE = 22050


# --- GUI ---


class WolfwagenGUI:
    """Simple status display GUI matching the existing Gemini chatbot style."""

    def __init__(self, root: tk.Tk, gui_queue: queue.Queue):
        self.root = root
        self.queue = gui_queue
        self.root.title("Wolfwagen - Local Voice Assistant")
        self.root.geometry("900x350")
        self.root.configure(bg="#2B2B2B")

        main_frame = tk.Frame(self.root, bg="#BB271A")
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        title_font = tkFont.Font(family="Helvetica", size=20, weight="bold")
        status_font = tkFont.Font(family="Helvetica", size=36, weight="bold")
        detail_font = tkFont.Font(family="Helvetica", size=18)
        transcript_font = tkFont.Font(family="Helvetica", size=16)

        tk.Label(
            main_frame,
            text="WOLFWAGEN ASSISTANT",
            font=title_font,
            fg="#FFFFFF",
            bg="#BB271A",
        ).pack(pady=(15, 5))

        self.status_label = tk.Label(
            main_frame,
            text="Initializing...",
            font=status_font,
            fg="#E0E0E0",
            bg="#BB271A",
        )
        self.status_label.pack(pady=(10, 5))

        self.detail_label = tk.Label(
            main_frame,
            text="",
            font=detail_font,
            fg="#A9A9A9",
            bg="#BB271A",
            wraplength=800,
        )
        self.detail_label.pack(pady=(0, 5))

        self.user_label = tk.Label(
            main_frame,
            text="",
            font=transcript_font,
            fg="#FFFFFF",
            bg="#BB271A",
            wraplength=800,
        )
        self.user_label.pack(pady=(5, 2))

        self.response_label = tk.Label(
            main_frame,
            text="",
            font=transcript_font,
            fg="#FFD700",
            bg="#BB271A",
            wraplength=800,
        )
        self.response_label.pack(pady=(2, 15))

        self._poll_queue()

    def _poll_queue(self):
        try:
            while not self.queue.empty():
                msg_type, value = self.queue.get_nowait()
                if msg_type == "status":
                    self.status_label.config(text=value)
                elif msg_type == "detail":
                    self.detail_label.config(text=value)
                elif msg_type == "user":
                    self.user_label.config(text=f"You: {value}")
                elif msg_type == "response":
                    self.response_label.config(text=f"Wolfwagen: {value}")
        finally:
            self.root.after(50, self._poll_queue)


# --- Audio Playback ---


def play_audio(audio: np.ndarray, sample_rate: int, pa: pyaudio.PyAudio):
    """Play a float32 numpy audio array through the speakers."""
    if len(audio) == 0:
        return

    audio_int16 = (audio * 32767).clip(-32768, 32767).astype(np.int16)
    stream = pa.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=sample_rate,
        output=True,
    )
    try:
        stream.write(audio_int16.tobytes())
    finally:
        stream.stop_stream()
        stream.close()


# --- Tool Execution ---


async def execute_tool_calls(
    tool_calls, handlers, llm: OllamaClient, gui_queue: queue.Queue
):
    """Execute tool calls from the LLM and send results back."""
    for tool_call in tool_calls:
        func = tool_call.get("function", {})
        name = func.get("name", "")
        args = func.get("arguments", {})

        gui_queue.put(("status", "Executing Maneuver..."))
        gui_queue.put(("detail", f"Action: {name}"))
        print(f"[Tool] Executing: {name}({args})")

        handler = handlers.get(name)
        if handler:
            try:
                if args:
                    result = await handler(**args)
                else:
                    result = await handler()
            except Exception as e:
                result = {"status": "error", "summary": str(e)}
        else:
            result = {"status": "error", "summary": f"Unknown function: {name}"}

        print(f"[Tool] Result: {result}")

        # Send result back to LLM for follow-up response
        response = llm.send_tool_result(name, result)
        return response

    return {"content": "", "tool_calls": []}


# --- Main Loop ---
def assistant_loop(gui_queue: queue.Queue):
    """Main voice assistant loop running in a background thread."""

    # Initialize components
    gui_queue.put(("status", "Loading models..."))

    vad = VoiceActivityDetector()
    stt = SpeechToText()
    tts = TextToSpeech()
    llm = OllamaClient()

    # Load models
    gui_queue.put(("detail", "Loading VAD..."))
    vad.load()

    gui_queue.put(("detail", "Loading Whisper STT..."))
    stt.load()

    gui_queue.put(("detail", "Loading Piper TTS..."))
    tts.load()

    # Check Ollama connection
    gui_queue.put(("detail", "Connecting to Ollama..."))
    if not llm.check_connection():
        gui_queue.put(("status", "ERROR"))
        gui_queue.put(("detail", "Ollama not running! Start with: ollama serve"))
        print("[Main] ERROR: Ollama server not running.")
        return

    models = llm.list_models()
    if llm.model not in models and not any(llm.model in m for m in models):
        gui_queue.put(("detail", f"Pulling model {llm.model}..."))
        print(f"[Main] Model {llm.model} not found. Available: {models}")
        gui_queue.put(("status", "ERROR"))
        gui_queue.put(
            ("detail", f"Model {llm.model} not found. Run: ollama pull {llm.model}")
        )
        return

    # Initialize ROS2 controller if available
    controller = None
    tool_handlers = {}
    if ROS2_AVAILABLE:
        try:
            controller = WolfwagenController()
            tool_handlers = get_tool_handlers(controller)
            gui_queue.put(("detail", "ROS2 motor control connected"))
        except Exception as e:
            print(f"[Main] ROS2 init failed: {e}. Running without motor control.")

    # Disable tool calling if no ROS2
    if not tool_handlers:
        llm.use_tools = False
        print("[Main] Tools disabled (no ROS2).")

    pa = pyaudio.PyAudio()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    gui_queue.put(("status", "Ready - Speak!"))
    gui_queue.put(("detail", "Listening for voice input..."))
    print("[Main] Voice assistant ready. Speak to interact.")

    # Play startup greeting
    try:
        greeting_audio = tts.synthesize("Wolfwagen online. Ready for your command.")
        play_audio(greeting_audio, TTS_SAMPLE_RATE, pa)
    except Exception as e:
        print(f"[Main] Startup greeting failed: {e}")

    try:
        while True:
            # Step 1: Listen for speech (VAD)
            gui_queue.put(("status", "Listening..."))
            gui_queue.put(("detail", "Waiting for voice input"))
            audio = vad.listen_for_speech(pa)

            if len(audio) == 0:
                continue

            # Step 2: Transcribe speech (Whisper)
            gui_queue.put(("status", "Transcribing..."))
            gui_queue.put(("detail", "Processing speech with Whisper"))
            text = stt.transcribe(audio)

            if not text.strip():
                print("[Main] Empty transcription, skipping.")
                continue

            print(f"[User] {text}")
            gui_queue.put(("user", text))

            # Step 3: Send to LLM (Ollama)
            gui_queue.put(("status", "Thinking..."))
            gui_queue.put(("detail", f"Querying {llm.model}"))
            response = llm.chat(text)

            # Step 4: Handle tool calls if any
            if response["tool_calls"] and tool_handlers:
                followup = loop.run_until_complete(
                    execute_tool_calls(
                        response["tool_calls"],
                        tool_handlers,
                        llm,
                        gui_queue,
                    )
                )
                # Use the follow-up text response after tool execution
                response_text = followup.get("content", "Maneuver complete.")
            elif response["tool_calls"] and not tool_handlers:
                # LLM wants to call tools but no ROS2 available — ask again without tools
                print("[Main] Tool calls received but no ROS2. Re-querying without tools.")
                import requests as req
                retry = req.post(f"{llm.base_url}/api/chat", json={
                    "model": llm.model,
                    "messages": llm.conversation_history[:-1] + [
                        {"role": "user", "content": text + " (just respond verbally, do not execute any actions)"}
                    ],
                    "stream": False,
                    "options": {"num_predict": 80},
                }, timeout=60)
                response_text = retry.json().get("message", {}).get("content", "")
            else:
                response_text = response["content"]

            if not response_text:
                response_text = "I heard you, but I'm not sure how to respond."

            print(f"[Wolfwagen] {response_text}")
            gui_queue.put(("response", response_text))

            # Step 5: Speak the response (Piper TTS)
            gui_queue.put(("status", "Speaking..."))
            gui_queue.put(("detail", "Generating speech with Piper"))
            response_audio = tts.synthesize(response_text)
            play_audio(response_audio, TTS_SAMPLE_RATE, pa)

            gui_queue.put(("status", "Ready - Speak!"))
            gui_queue.put(("detail", "Listening for voice input..."))

    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        pa.terminate()
        if controller:
            controller.shutdown()
        loop.close()


def main():
    """Start the GUI and assistant loop."""
    gui_queue = queue.Queue()
    root = tk.Tk()
    gui = WolfwagenGUI(root, gui_queue)

    assistant_thread = threading.Thread(
        target=assistant_loop, args=(gui_queue,), daemon=True
    )
    assistant_thread.start()

    def on_closing():
        print("[Main] Close signal received.")
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt.")
    finally:
        print("[Main] Application exited.")


if __name__ == "__main__":
    main()
