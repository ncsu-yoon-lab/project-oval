#!/usr/bin/env python3
import asyncio
import math
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

try:
    from wolfwagen_actions import (
        WolfwagenController,
        get_function_declarations,
        get_handlers,
    )

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[Main] ROS2 not available. Running without motor control.")

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

MODEL = os.getenv("GEMINI_LIVE_MODEL", "gemini-2.5-flash-native-audio-latest")

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


# --- Voice Orb ---
class VoiceOrb:
    """Morphing multi-layer blob. Layers are painted back→front (outermost first)."""

    # NC State red gradient: deep dark → NC red → salmon → white
    LAYER_COLORS = [
        "#1A0000",
        "#3D0000",
        "#770000",
        "#AA0000",
        "#CC0000",
        "#EE3311",
        "#FF9977",
        "#FFFFFF",
    ]
    # (radial_scale, amplitude_factor, phase_offset)
    LAYER_CFG = [
        (2.00, 0.28, 0.0),
        (1.70, 0.42, 0.9),
        (1.42, 0.60, 1.8),
        (1.16, 0.80, 2.7),
        (0.90, 1.00, 0.4),
        (0.64, 0.68, 1.3),
        (0.38, 0.38, 2.2),
        (0.18, 0.16, 0.8),
    ]

    def __init__(self, canvas: tk.Canvas, cx: int, cy: int, base_r: float = 88.0):
        self.canvas = canvas
        self.cx, self.cy = cx, cy
        self.base_r = base_r
        self.phase = 0.0
        self.amplitude = 5.0
        self.target_amp = 5.0
        self.speed = 0.022
        self.target_speed = 0.022
        # Pulse beat (active only when speaking)
        self.pulse_phase = 0.0
        self.pulse_amp = 0.0
        self.target_pulse_amp = 0.0
        self.pulse_speed = 0.0
        self.target_pulse_speed = 0.0

        # Static radar rings drawn first so blobs paint over them at center
        for r in range(200, 0, -40):
            canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r, outline="#1C1C1C", fill="", width=1
            )

        self.layers = [
            canvas.create_polygon([0, 0, 1, 1], smooth=True, fill=c, outline="")
            for c in self.LAYER_COLORS
        ]
        self._animate()

    def set_state(self, state: str):
        if state == "speaking":
            self.target_amp = 16.0
            self.target_speed = 0.06
            self.target_pulse_amp = 24.0  # strong outward beat
            self.target_pulse_speed = 0.20  # ~2 beats/sec at 30fps
        elif state == "listening":
            self.target_amp = 14.0
            self.target_speed = 0.040
            self.target_pulse_amp = 0.0
            self.target_pulse_speed = 0.0
        else:
            self.target_amp = 5.0
            self.target_speed = 0.020
            self.target_pulse_amp = 0.0
            self.target_pulse_speed = 0.0

    def _pts(self, r: float, amp: float, p_off: float, n: int = 80):
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            radius = r + amp * (
                math.sin(self.phase + p_off + a * 3) * 0.50
                + math.sin(self.phase * 1.3 + p_off + a * 5) * 0.30
                + math.sin(self.phase * 0.7 + p_off + a * 2) * 0.20
            )
            pts.extend([self.cx + radius * math.cos(a), self.cy + radius * math.sin(a)])
        return pts

    def _animate(self):
        self.amplitude += (self.target_amp - self.amplitude) * 0.08
        self.speed += (self.target_speed - self.speed) * 0.05
        self.pulse_amp += (self.target_pulse_amp - self.pulse_amp) * 0.07
        self.pulse_speed += (self.target_pulse_speed - self.pulse_speed) * 0.05
        self.phase += self.speed
        self.pulse_phase += self.pulse_speed

        # Pulse beat expands the base radius rhythmically
        beat = self.pulse_amp * abs(math.sin(self.pulse_phase))
        effective_r = self.base_r + beat

        for layer, (sf, af, po) in zip(self.layers, self.LAYER_CFG):
            self.canvas.coords(
                layer, self._pts(effective_r * sf, self.amplitude * af, po)
            )
        self.canvas.after(30, self._animate)


# --- GUI Class ---
class WolfwagenGUI:
    # ── NC State palette ──────────────────────────────────────────────────────
    C_BG = "#0A0A0A"  # near-black
    C_PANEL = "#111111"  # slightly lighter panel
    C_SURFACE = "#161616"  # card surface
    C_RED = "#CC0000"  # Reynolds Red
    C_WHITE = "#F5F5F5"
    C_TEXT = "#E8E8E8"
    C_DIM = "#5A5A5A"
    C_BORDER = "#242424"

    def __init__(self, root: tk.Tk, q: queue.Queue):
        self.root = root
        self.queue = q
        self.root.title("Project OVAL — Wolfwagen")
        self.root.geometry("1440x810")
        self.root.configure(bg=self.C_BG)
        self.root.resizable(True, True)

        self._build_header()
        self._build_body()
        self.process_queue()

    # ── Header ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=self.C_PANEL, height=58)
        hdr.pack(fill="x", side="top")
        hdr.pack_propagate(False)

        f_title = tkFont.Font(family="Courier", size=18, weight="bold")
        f_sub = tkFont.Font(family="Courier", size=9)
        f_badge = tkFont.Font(family="Courier", size=14, weight="bold")

        left = tk.Frame(hdr, bg=self.C_PANEL)
        left.pack(side="left", padx=24, pady=10)
        tk.Label(
            left, text="PROJECT OVAL", font=f_title, fg=self.C_WHITE, bg=self.C_PANEL
        ).pack(anchor="w")
        tk.Label(
            left,
            text="AUTONOMOUS PATROL VEHICLE  ·  YOON'S LAB",
            font=f_sub,
            fg=self.C_DIM,
            bg=self.C_PANEL,
        ).pack(anchor="w")

        right = tk.Frame(hdr, bg=self.C_PANEL)
        right.pack(side="right", padx=24, pady=10)
        tk.Label(
            right, text="WOLFWAGEN", font=f_badge, fg=self.C_RED, bg=self.C_PANEL
        ).pack(anchor="e")
        tk.Label(
            right,
            text="NC STATE UNIVERSITY  ·  CENTENNIAL CAMPUS",
            font=f_sub,
            fg=self.C_DIM,
            bg=self.C_PANEL,
        ).pack(anchor="e")

        # NC State red hairline under header
        tk.Frame(self.root, bg=self.C_RED, height=2).pack(fill="x", side="top")

    # ── Body ──────────────────────────────────────────────────────────────────
    def _build_body(self):
        body = tk.Frame(self.root, bg=self.C_BG)
        body.pack(fill="both", expand=True)

        # Column frames
        left = tk.Frame(body, bg=self.C_PANEL, width=280)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        tk.Frame(body, bg=self.C_BORDER, width=1).pack(side="left", fill="y")

        center = tk.Frame(body, bg=self.C_BG, width=460)
        center.pack(side="left", fill="y")
        center.pack_propagate(False)

        tk.Frame(body, bg=self.C_BORDER, width=1).pack(side="left", fill="y")

        right = tk.Frame(body, bg=self.C_BG)
        right.pack(side="left", fill="both", expand=True)

        self._build_left_col(left)
        self._build_center_col(center)
        self._build_right_col(right)

    # ── Left column: logo + vehicle data ─────────────────────────────────────
    def _build_left_col(self, panel):
        # Logo in white box
        self.logo_label = tk.Label(panel, bg=self.C_PANEL)
        self.logo_label.pack(pady=(18, 0))
        try:
            _here = os.path.dirname(os.path.abspath(__file__))
            img = Image.open(os.path.join(_here, "nc_state_logo.png"))
            img = img.resize((244, 244), Image.LANCZOS)
            self.logo_image = ImageTk.PhotoImage(img)
            self.logo_label.config(image=self.logo_image)
        except Exception:
            f_fb = tkFont.Font(family="Courier", size=13, weight="bold")
            self.logo_label.config(text="WOLFPACK", font=f_fb, fg=self.C_RED)

        # Thin red divider
        tk.Frame(panel, bg=self.C_RED, height=2).pack(fill="x", padx=16, pady=14)

        # Vehicle data readout
        f_key = tkFont.Font(family="Courier", size=8)
        f_val = tkFont.Font(family="Courier", size=10, weight="bold")

        rows = [
            ("UNIT", "WOLFWAGEN-01"),
            ("CAMPUS", "CENTENNIAL"),
            ("MODE", "PATROL"),
        ]
        data = tk.Frame(panel, bg=self.C_PANEL)
        data.pack(fill="x", padx=16)
        for key, val in rows:
            row = tk.Frame(data, bg=self.C_PANEL)
            row.pack(fill="x", pady=4)
            tk.Label(
                row,
                text=key,
                font=f_key,
                fg=self.C_DIM,
                bg=self.C_PANEL,
                width=8,
                anchor="w",
            ).pack(side="left")
            tk.Frame(row, bg=self.C_BORDER, width=1).pack(side="left", fill="y", padx=6)
            tk.Label(
                row, text=val, font=f_val, fg=self.C_WHITE, bg=self.C_PANEL, anchor="w"
            ).pack(side="left")

        # ── Radar + description (fills remaining space) ───────────────────────
        tk.Frame(panel, bg=self.C_BORDER, height=1).pack(
            fill="x", padx=16, pady=(18, 0)
        )

        bottom = tk.Frame(panel, bg=self.C_PANEL)
        bottom.pack(fill="both", expand=True, padx=16, pady=(14, 20))

        f_desc_lbl = tkFont.Font(family="Courier", size=9, weight="bold")
        f_desc = tkFont.Font(family="Courier", size=11)

        tk.Label(
            bottom, text="ABOUT", font=f_desc_lbl, fg=self.C_DIM, bg=self.C_PANEL
        ).pack(anchor="w", pady=(0, 10))

        description = (
            "Wolfwagen is an autonomous\n"
            "patrol vehicle developed by\n"
            "Yoon's Lab at NC State\n"
            "University.\n\n"
            "It navigates Centennial\n"
            "Campus using computer\n"
            "vision, GPS, and a\n"
            "Gemini-powered voice\n"
            "interface."
        )
        tk.Label(
            bottom,
            text=description,
            font=f_desc,
            fg="#707070",
            bg=self.C_PANEL,
            justify="left",
            anchor="nw",
        ).pack(anchor="nw")

    # ── Center column: voice orb ──────────────────────────────────────────────
    def _build_center_col(self, panel):
        orb_canvas = tk.Canvas(
            panel, bg=self.C_BG, highlightthickness=0, width=460, height=720
        )
        orb_canvas.pack(fill="both", expand=True)
        self.voice_orb = VoiceOrb(orb_canvas, cx=230, cy=360, base_r=88)

    # ── Right column: status cards + text ─────────────────────────────────────
    def _build_right_col(self, panel):
        f_lbl = tkFont.Font(family="Courier", size=9, weight="bold")

        def card(parent, top_accent=None, fixed_h=None):
            """Surface card with optional top accent bar."""
            outer = tk.Frame(parent, bg=self.C_SURFACE)
            if fixed_h:
                outer.config(height=fixed_h)
                outer.pack_propagate(False)
            if top_accent:
                tk.Frame(outer, bg=top_accent, height=2).pack(fill="x")
            inner = tk.Frame(outer, bg=self.C_SURFACE)
            inner.pack(fill="both", expand=True, padx=22, pady=14)
            return outer, inner

        # ── VEHICLE STATUS ────────────────────────────────────────────────────
        sc, si = card(panel, top_accent=self.C_RED, fixed_h=104)
        sc.pack(fill="x", padx=22, pady=(20, 8))

        tk.Label(
            si, text="VEHICLE STATUS", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE
        ).pack(anchor="w")

        dot_row = tk.Frame(si, bg=self.C_SURFACE)
        dot_row.pack(anchor="w", pady=(5, 0))

        self._dot_cv = tk.Canvas(
            dot_row, width=14, height=14, bg=self.C_SURFACE, highlightthickness=0
        )
        self._dot_cv.pack(side="left", padx=(0, 10))
        self._dot = self._dot_cv.create_oval(1, 1, 13, 13, fill=self.C_RED, outline="")

        f_status = tkFont.Font(family="Helvetica", size=26, weight="bold")
        self.status_label = tk.Label(
            dot_row,
            text="Initializing...",
            font=f_status,
            fg=self.C_TEXT,
            bg=self.C_SURFACE,
        )
        self.status_label.pack(side="left")

        # ── ACTIVE COMMAND ────────────────────────────────────────────────────
        ac, ai = card(panel, fixed_h=76)
        ac.pack(fill="x", padx=22, pady=(0, 8))

        tk.Label(
            ai, text="ACTIVE COMMAND", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE
        ).pack(anchor="w")

        f_detail = tkFont.Font(family="Courier", size=13)
        self.detail_label = tk.Label(
            ai,
            text="—",
            font=f_detail,
            fg="#888888",
            bg=self.C_SURFACE,
            wraplength=640,
            justify="left",
        )
        self.detail_label.pack(anchor="w", pady=(3, 0))

        # ── TRANSMISSION ──────────────────────────────────────────────────────
        tc, ti = card(panel, top_accent=self.C_RED)
        tc.pack(fill="both", expand=True, padx=22, pady=(0, 22))

        tk.Label(
            ti, text="TRANSMISSION", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE
        ).pack(anchor="nw")

        f_response = tkFont.Font(family="Helvetica", size=21, weight="bold")
        self.output_label = tk.Label(
            ti,
            text="",
            font=f_response,
            fg=self.C_TEXT,
            bg=self.C_SURFACE,
            wraplength=640,
            justify="center",
            anchor="center",
        )
        self.output_label.pack(fill="both", expand=True, pady=(7, 0))

    # ── Queue processing ──────────────────────────────────────────────────────
    def process_queue(self):
        try:
            while not self.queue.empty():
                message_type, value = self.queue.get_nowait()
                if message_type == "status":
                    self.status_label.config(text=value)
                    t = value.lower()
                    if "speak" in t:
                        self.voice_orb.set_state("speaking")
                    elif any(
                        w in t for w in ("listen", "welcome", "ready", "go wolfpack")
                    ):
                        self.voice_orb.set_state("listening")
                    else:
                        self.voice_orb.set_state("idle")
                elif message_type == "detail":
                    self.detail_label.config(text=value if value else "—")
                elif message_type == "output":
                    self.output_label.config(text=value)
        finally:
            self.root.after(50, self.process_queue)  # Check again in 100ms


def build_tools() -> List[types.Tool]:
    if ROS2_AVAILABLE:
        tool = types.Tool(function_declarations=get_function_declarations())
        return [tool]
    return []


async def stream_microphone_and_handle(
    session: any, p: pyaudio.PyAudio, gui_queue: queue.Queue, handlers
) -> None:
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
        is_speaking = False  # Flag to set "Speaking" status only once per turn

        async def recv_loop():
            nonlocal is_speaking
            try:
                async for message in session.receive():
                    # A message can contain multiple parts; process all of them.
                    nonlocal input_text_accum, output_text_accum, input_printed
                    server_content = getattr(message, "server_content", None)

                    # Aggregate input transcription; we'll decide when to flush below based on server signals.
                    if server_content and getattr(
                        server_content, "input_transcription", None
                    ):
                        try:
                            chunk = server_content.input_transcription.text or ""
                            if chunk:
                                input_text_accum += chunk
                        except Exception:
                            pass

                    # Aggregate output transcription; print once generation completes below.
                    if server_content and getattr(
                        server_content, "output_transcription", None
                    ):
                        try:
                            # If model starts streaming text/audio, flush user's transcript once (end-of-speech implied).
                            if (not input_printed) and input_text_accum.strip():
                                print(f"[Transcript][User]: {input_text_accum.strip()}")
                                input_text_accum = ""
                                input_printed = True
                                gui_queue.put(
                                    ("output", "")
                                )  # Clear output while input is displayed
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
                    if (
                        update
                        and getattr(update, "resumable", False)
                        and getattr(update, "new_handle", None)
                    ):
                        current_session_handle = update.new_handle
                        print(
                            f"[Session] Got new resumption handle: {current_session_handle[:10]}..."
                        )

                    if hasattr(message, "tool_call") and message.tool_call:
                        print(f"[Debug] Received tool_call object: {message.tool_call}")
                        asyncio.create_task(
                            handle_tool_calls(
                                session,
                                message.tool_call,
                                gui_queue,
                                turn_ended,
                                handlers,
                            )
                        )

                    server_content = getattr(message, "server_content", None)
                    if server_content and getattr(
                        server_content, "generation_complete", False
                    ):
                        if (not input_printed) and input_text_accum.strip():
                            print(f"[Transcript][User]: {input_text_accum.strip()}")
                            input_text_accum = ""
                            input_printed = True
                        if output_text_accum.strip():
                            print(
                                f"[Transcript][Wolfwagen]: {output_text_accum.strip()}"
                            )
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
                        lambda: input_stream.read(
                            FRAMES_PER_BUFFER, exception_on_overflow=False
                        ),
                    )
                    await session.send_realtime_input(
                        audio=types.Blob(
                            data=data, mime_type=f"audio/pcm;rate={INPUT_SAMPLE_RATE}"
                        )
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
        pass


async def handle_tool_calls(
    session: any,
    tool_call: any,
    gui_queue: queue.Queue,
    turn_ended: asyncio.Event,
    handlers: dict,
) -> None:
    responses: List[types.FunctionResponse] = []

    gui_queue.put(("status", "Executing Maneuver..."))

    for fc in tool_call.function_calls:
        name = fc.name
        call_id = fc.id
        print(f"[Tool] Model wants to call function: '{name}' (ID: {call_id})")

        detail_text = f"Action: {name}"
        if name == "execute_sequence" and hasattr(fc, "args"):
            steps = fc.args.get("steps", [])
            step_summary = ", ".join(
                [
                    f"{step.get('action', 'N/A')}(x{step.get('repeat', 1)})"
                    for step in steps
                ]
            )
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

            responses.append(
                types.FunctionResponse(id=call_id, name=name, response=result)
            )
        else:
            responses.append(
                types.FunctionResponse(
                    id=call_id,
                    name=name,
                    response={
                        "status": "error",
                        "summary": f"Function '{name}' not found.",
                    },
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
                "speech_config": {
                    "voice_config": {"prebuilt_voice_config": {"voice_name": "Puck"}}
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

            print(
                f"\n[Session] Starting new connection. Resuming from handle: {str(current_session_handle)[:10]}..."
            )
            gui_queue.put(("status", "Connecting..."))
            try:
                async with client.aio.live.connect(
                    model=MODEL, config=base_config
                ) as session:
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

    robot_controller = None
    tool_handlers = {}
    if ROS2_AVAILABLE:
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
        if robot_controller:
            robot_controller.shutdown()
        root.after(200, root.destroy)

    async_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[Main] KeyboardInterrupt caught.")
    finally:
        if not shutdown_event.is_set():
            on_closing()
        async_thread.join(timeout=2.0)
        print("[Main] Application exited.")


if __name__ == "__main__":
    main()
