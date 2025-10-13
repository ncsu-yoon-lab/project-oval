#!/usr/bin/env python3
import asyncio
import os
import sys
from typing import List

import pyaudio
from google import genai
from google.genai import types
from dotenv import load_dotenv

# GUI: Import tkinter and asyncio.Queue
import tkinter as tk
from asyncio import Queue

import wolfwagen_actions as actions

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
    "- Mobility: path planning and voice-controlled maneuvers. Available commands include `turn_left` and `turn_right`.\n"
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


def build_tools() -> List[types.Tool]:
    tool = types.Tool(function_declarations=actions.get_function_declarations())
    return [tool]


async def stream_microphone_and_handle(session: any, p: pyaudio.PyAudio) -> None:
    """Handles one full turn of conversation: user speaks, model replies, then exits."""
    global current_session_handle
    input_stream = None
    output_stream = None

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
        print("\n[Wolfwagen] Listening... Speak now.")

        turn_ended = asyncio.Event()

        async def recv_loop():
            try:
                async for message in session.receive():
                    # A message can contain multiple parts; process all of them.

                    if getattr(message, "data", None) is not None:
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

                    # Per the official notebook, the response object has a `tool_call` attribute.
                    if hasattr(message, "tool_call") and message.tool_call:
                        print(f"[Debug] Received tool_call object: {message.tool_call}")
                        asyncio.create_task(handle_tool_calls(session, message.tool_call, turn_ended))

                    server_content = getattr(message, "server_content", None)
                    if server_content and getattr(server_content, "generation_complete", False):
                        # End the turn if it's a pure voice response.
                        # For tool calls, `handle_tool_calls` is responsible for ending the turn.
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
            # Only cancel the receiver; let the sender exit via turn_ended to avoid
            # interrupting an in-flight blocking read in the executor thread.
            if not recv_task.done():
                recv_task.cancel()
            # Wait for tasks to finish their cleanup before the main finally block runs.
            await asyncio.gather(recv_task, send_task, return_exceptions=True)
            # Ensure audio streams are closed after tasks have completed.
            try: 
                if output_stream:
                    try:
                        if not output_stream.is_stopped():
                            output_stream.stop_stream()
                    except Exception:
                        pass
                    try:
                        output_stream.close()
                    except Exception:
                        pass
                if input_stream:
                    try:
                        if not input_stream.is_stopped():
                            input_stream.stop_stream()
                    except Exception:
                        pass
                    try:
                        input_stream.close()
                    except Exception:
                        pass
            except Exception:
                pass
            print("[Wolfwagen] Turn finished.")
    finally:
        # Terminate the PyAudio session after all streams are guaranteed to be closed.
        # This is now handled in the main function's finally block.
        pass


async def handle_tool_calls(session: any, tool_call: any, turn_ended: asyncio.Event) -> None:
    responses: List[types.FunctionResponse] = []

    # The tool_call object contains a list of function_calls to be executed.
    for fc in tool_call.function_calls:
        name = fc.name
        call_id = fc.id
        print(f"[Tool] Model wants to call function: '{name}' (ID: {call_id})")
        handler = actions.HANDLERS.get(name)

        if handler:
            # Some tools (like execute_sequence) may expect arguments
            kwargs = {}
            if hasattr(fc, "args") and isinstance(fc.args, dict):
                kwargs = fc.args

            try:
                result = await handler(**kwargs) if kwargs else await handler()
            except TypeError:
                # Fallback if handler signature doesn't accept kwargs
                result = await handler()

            # The FunctionResponse requires the original call's ID.
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
        # Per the official notebook, use the `function_responses` keyword argument.
        await session.send_tool_response(function_responses=responses)
        print("[Tool] Tool responses sent.")

    # Note: do not end the turn here; allow the model to continue making additional tool calls
    # within the same turn if needed. The recv_loop will end the turn when generation completes.


async def main() -> None:
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    if not api_key:
        print("[Wolfwagen][Error] API key not set.")
        sys.exit(1)

    client = genai.Client(api_key=api_key)

    # Instantiate PyAudio once for the entire application lifecycle.
    p = pyaudio.PyAudio()
    try:
        while True:
            base_config = {
                "response_modalities": ["AUDIO"],
                "speech_config": {
                    "voice_config": {
                        "prebuilt_voice_config": {
                            "voice_name": "Puck"
                        }
                    }
                },
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

            try:
                async with client.aio.live.connect(model=MODEL, config=base_config) as session:
                    print("[Session] Connection opened.")
                    # Pass the single PyAudio instance to the handler.
                    await stream_microphone_and_handle(session, p)
            except Exception as e:
                print(f"[Session] Connection error: {e}. Reconnecting in 2s...")
                await asyncio.sleep(2)
    finally:
        # Terminate PyAudio once when the application exits.
        if p:
            p.terminate()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Wolfwagen] Stopped by user")
        sys.exit(0)
