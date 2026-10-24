#!/usr/bin/env python3
"""Test harness for Gemini Robotics-ER 2 Streaming Preview (Live API only).

Runs a battery of capability tests on one camera frame (or image), or an interactive
mode that streams camera frames continuously while you type questions.

    python3 robotics_er2_test.py                      # battery on /dev/video0
    python3 robotics_er2_test.py --camera 4           # ZED 2i (left eye is cropped out automatically)
    python3 robotics_er2_test.py --image wolfpack.jpg --point "the wolf's eyes"
    python3 robotics_er2_test.py --chat --fps 1       # stream frames, type prompts

Robot tools are simulated: calls are printed and answered with a fake success, nothing is
published to ROS, so the car never moves.

Model quirks found while writing this:
  * Text-only output; AUDIO response modality is rejected (1007).
  * Even with TEXT, replies arrive in server_content.output_transcription, not model_turn.
  * Spatial answers use [y, x] / [ymin, xmin, ymax, xmax] normalized to 0-1000.
  * A realtime video frame sent right before a realtime text prompt may not be ingested
    yet, and the model then answers blind (confidently describing things that aren't
    there). The battery sends image + prompt in one turn instead, and every reply reports
    how many image tokens the model actually counted.
"""
import argparse
import asyncio
import json
import os
import re
import sys
import time

import cv2
from dotenv import load_dotenv
from google import genai
from google.genai import types

HERE = os.path.dirname(os.path.abspath(__file__))
load_dotenv(os.path.join(HERE, ".env"))

MODEL = os.getenv("GEMINI_ER_MODEL", "gemini-robotics-er-2-streaming-preview")
MAX_SIDE = 768
TURN_TIMEOUT_S = 60.0

ROBOT_TOOLS = [
    types.Tool(
        function_declarations=[
            {"name": name, "description": desc, "parameters": {"type": "object", "properties": {}}}
            for name, desc in [
                ("go_straight", "Drive the robot forward for 2 seconds."),
                ("go_back", "Drive the robot backward for 2 seconds."),
                ("turn_left", "Turn the robot 90 degrees to the left."),
                ("turn_right", "Turn the robot 90 degrees to the right."),
            ]
        ]
    )
]


# --- Frames ---------------------------------------------------------------------------
def is_zed(index: int) -> bool:
    try:
        with open(f"/sys/class/video4linux/video{index}/name") as f:
            return "ZED" in f.read()
    except OSError:
        return False


def fit(frame):
    h, w = frame.shape[:2]
    scale = MAX_SIDE / max(h, w)
    return cv2.resize(frame, (int(w * scale), int(h * scale))) if scale < 1 else frame


class Camera:
    def __init__(self, index: int):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            sys.exit(f"Could not open /dev/video{index}")
        # Without the ZED SDK the ZED is a side-by-side stereo UVC camera
        self.stereo = is_zed(index)
        for _ in range(5):  # let auto-exposure settle
            self.cap.read()

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        if self.stereo:
            frame = frame[:, : frame.shape[1] // 2]
        return fit(frame)

    def close(self):
        self.cap.release()


def jpeg(frame) -> bytes:
    return cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


# --- Model I/O ------------------------------------------------------------------------
async def receive_turn(session, simulate_tools: bool = True):
    """Collects one full reply, answering tool calls with a fake success until the model is done.

    Returns (text, tool calls, seconds to first output, total seconds, image tokens in the prompt).
    """
    text, calls, image_tokens = "", [], None
    t0, first = time.monotonic(), None
    while True:
        pending = None
        async for msg in session.receive():
            sc = msg.server_content
            chunk = ""
            if sc and sc.output_transcription and sc.output_transcription.text:
                chunk = sc.output_transcription.text
            elif sc and sc.model_turn:
                chunk = "".join(p.text for p in sc.model_turn.parts if p.text and not p.thought)
            if chunk or msg.tool_call:
                first = first if first is not None else time.monotonic() - t0
            text += chunk
            usage = msg.usage_metadata
            if usage and usage.prompt_tokens_details:
                image_tokens = sum(
                    d.token_count or 0 for d in usage.prompt_tokens_details if d.modality == types.MediaModality.IMAGE
                )
            if msg.tool_call:
                pending = msg.tool_call
                for fc in msg.tool_call.function_calls:
                    calls.append(f"{fc.name}({fc.args or {}})")
                    print(f"  -> tool call: {fc.name}({fc.args or {}})  [simulated, robot not moved]")
                # The turn stays open until the tool response arrives, so stop reading here
                break
        if pending and simulate_tools:
            await session.send_tool_response(
                function_responses=[
                    types.FunctionResponse(id=fc.id, name=fc.name, response={"status": "success"})
                    for fc in pending.function_calls
                ]
            )
            continue
        return text.strip(), calls, first, time.monotonic() - t0, image_tokens


def parse_json(text: str):
    match = re.search(r"\[.*\]", text, re.DOTALL)
    if not match:
        return None
    try:
        return json.loads(match.group(0))
    except json.JSONDecodeError:
        return None


def annotate(frame, items, path):
    """Draws points / boxes (normalized 0-1000, y first) and saves the image."""
    img = frame.copy()
    h, w = img.shape[:2]
    for item in items or []:
        if not isinstance(item, dict):
            continue
        label = str(item.get("label", ""))
        if "point" in item:
            y, x = item["point"]
            px, py = int(x / 1000 * w), int(y / 1000 * h)
            cv2.circle(img, (px, py), 7, (0, 255, 0), 2)
            cv2.putText(img, label, (px + 9, py - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        elif "box_2d" in item:
            y0, x0, y1, x1 = item["box_2d"]
            p0, p1 = (int(x0 / 1000 * w), int(y0 / 1000 * h)), (int(x1 / 1000 * w), int(y1 / 1000 * h))
            cv2.rectangle(img, p0, p1, (0, 200, 255), 2)
            cv2.putText(img, label, (p0[0] + 3, p0[1] + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    cv2.imwrite(path, img)
    return path


# --- Battery --------------------------------------------------------------------------
def battery(point_target: str):
    return [
        ("describe", False, "Briefly describe what you see, in 2 sentences."),
        (
            "point",
            False,
            f"Point to {point_target}. Answer ONLY with JSON: "
            '[{"point": [y, x], "label": <name>}], coordinates normalized to 0-1000.',
        ),
        (
            "boxes",
            False,
            "Detect up to 5 prominent objects. Answer ONLY with JSON: "
            '[{"box_2d": [ymin, xmin, ymax, xmax], "label": <name>}], normalized to 0-1000.',
        ),
        (
            "tools",
            True,
            "You are the brain of a small wheeled robot looking through this camera. "
            "Turn left, then drive forward, then say what you did.",
        ),
    ]


async def run_battery(client, frame, args):
    os.makedirs(args.out, exist_ok=True)
    cv2.imwrite(os.path.join(args.out, "frame.jpg"), frame)
    print(f"Model: {MODEL}  frame: {frame.shape[1]}x{frame.shape[0]}  output: {args.out}/\n")
    results = []
    for name, with_tools, prompt in battery(args.point):
        config = {"response_modalities": ["TEXT"]}
        if with_tools:
            config["tools"] = ROBOT_TOOLS
        print(f"== {name} ==")
        try:
            # A fresh session per test so earlier answers don't leak into later ones
            async with client.aio.live.connect(model=MODEL, config=config) as session:
                # Image and prompt in one turn, so the model can't answer before it has the image
                await session.send_client_content(
                    turns={
                        "role": "user",
                        "parts": [{"inline_data": {"mime_type": "image/jpeg", "data": jpeg(frame)}}, {"text": prompt}],
                    },
                    turn_complete=True,
                )
                text, calls, first, total, image_tokens = await asyncio.wait_for(receive_turn(session), TURN_TIMEOUT_S)
        except Exception as e:
            print(f"  FAIL {type(e).__name__}: {e}\n")
            results.append((name, "FAIL"))
            continue
        print(f"  first response {first:.2f}s, total {total:.2f}s" if first is not None else f"  no output ({total:.2f}s)")
        blind = not image_tokens
        print(f"  image tokens seen: {image_tokens}" + ("  <-- WARNING: answered without the image" if blind else ""))
        if text:
            print("  " + text.replace("\n", "\n  "))
        items = parse_json(text) if name in ("point", "boxes") else None
        if items:
            print("  annotated:", annotate(frame, items, os.path.join(args.out, f"{name}.jpg")))
        ok = bool(items) if name in ("point", "boxes") else bool(calls) if with_tools else bool(text)
        results.append((name, "BLIND" if blind else "PASS" if ok else "CHECK"))
        print()
    print("Summary: " + "  ".join(f"{n}={r}" for n, r in results))
    print("PASS only means the reply had the right shape: check the annotated images before trusting it.")


# --- Interactive streaming --------------------------------------------------------------
async def run_chat(client, camera, args):
    os.makedirs(args.out, exist_ok=True)
    config = {"response_modalities": ["TEXT"], "tools": ROBOT_TOOLS}
    latest = {"frame": camera.read()}
    print(f"Model: {MODEL}  streaming frames at {args.fps} fps. Type a prompt, or 'quit'.")
    print('Tip: ask for JSON points/boxes (e.g. "point to the door as JSON") to get an annotated image.\n')
    async with client.aio.live.connect(model=MODEL, config=config) as session:

        async def stream_frames():
            while True:
                latest["frame"] = await asyncio.to_thread(camera.read)
                await session.send_realtime_input(video=types.Blob(data=jpeg(latest["frame"]), mime_type="image/jpeg"))
                await asyncio.sleep(1.0 / args.fps)

        streamer = asyncio.create_task(stream_frames())
        try:
            n = 0
            while True:
                prompt = (await asyncio.to_thread(input, "you> ")).strip()
                if prompt.lower() in ("quit", "exit", "q"):
                    break
                if not prompt:
                    continue
                frame = latest["frame"]  # what the model most recently saw
                await session.send_realtime_input(text=prompt)
                text, _, first, total, image_tokens = await asyncio.wait_for(receive_turn(session), TURN_TIMEOUT_S)
                timing = f"{first:.2f}s to first response, {total:.2f}s total" if first is not None else "no output"
                print(f"er2> {text}\n     ({timing}, image tokens in context: {image_tokens})")
                items = parse_json(text)
                if items:
                    n += 1
                    print("     annotated:", annotate(frame, items, os.path.join(args.out, f"chat_{n}.jpg")))
                print()
        finally:
            streamer.cancel()


async def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0, help="/dev/videoN index (0 = USB HD camera, 4 = ZED 2i)")
    parser.add_argument("--image", help="use this image instead of a camera")
    parser.add_argument("--point", default="the most prominent objects", help="what the pointing test should point at")
    parser.add_argument("--chat", action="store_true", help="interactive mode: stream frames and type prompts")
    parser.add_argument("--fps", type=float, default=1.0, help="frames per second sent in --chat mode")
    parser.add_argument("--out", default="er2_output", help="directory for frames and annotated results")
    args = parser.parse_args()

    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    if not api_key:
        sys.exit("Set GEMINI_API_KEY (or put it in tools/gemini_chatbot/.env)")
    client = genai.Client(api_key=api_key)

    if args.chat:
        if args.image:
            sys.exit("--chat streams from a camera; drop --image")
        camera = Camera(args.camera)
        try:
            await run_chat(client, camera, args)
        finally:
            camera.close()
        return

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            sys.exit(f"Could not read {args.image}")
        frame = fit(frame)
    else:
        camera = Camera(args.camera)
        frame = camera.read()
        camera.close()
    await run_battery(client, frame, args)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
