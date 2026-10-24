# Wolfwagen Symposium Demo

`symposium_demo.py` is a voice-controlled demo of the Wolfwagen. You talk to it through the mic, it answers through the speaker, and spoken movement commands ("move forward", "turn left twice") drive the car. It uses Gemini Live (`gemini-2.5-flash-native-audio-latest`) and publishes to `/gemini/throttle` and `/gemini/steering`, which `driver_node` follows in **Auto** mode.

## Quickstart

**One-time setup** (on the Jetson):

```bash
cd ~/ros2_ws/src/project-oval/tools/gemini_chatbot
python3 -m pip install -r requirements.txt pillow
echo "GEMINI_API_KEY=<your key>" > .env      # git-ignored
```

**Every run** (two terminals, ROS sourced in both: `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`):

1. Start the car stack (joy, Xbox controller, motor driver):
   ```bash
   ros2 launch project_oval_control manual_control.launch.py
   ```
   Check that the curses screen shows `Joy: OK`. If it shows `LOST`, pair the controller.
2. Press **A** on the controller to switch to **Auto**. The screen shows `Manual: False`.
3. Start the demo:
   ```bash
   cd ~/ros2_ws/src/project-oval/tools/gemini_chatbot
   python3 symposium_demo.py
   ```
4. Wait for **"Welcome to NC State! Go Wolfpack!"**, then speak.

To stop, **close the window**. Ctrl+C doesn't work, because rclpy catches the signal.

## Controls

| Input | Action |
|---|---|
| Voice: "move forward / back", "turn left / right" | One 2-second maneuver |
| Voice: "move back and then turn left", "turn right twice" | A sequence of maneuvers |
| **RB** | **Reset**: stops the motors and starts a fresh listening session |
| **A** | Toggle Manual / Auto |
| **RT / LT** (Manual) | Drive forward / backward |
| **Right stick** (Manual) | Steer |
| **RB held** (Manual) | Turbo |

If a maneuver or the connection fails, the screen shows **"Timed out"** and **"Press RB to Reset"**. Press RB and it listens again. After 60 s of silence it reconnects by itself.

## Safety notes

- **RB is the only stop.** There is no spoken "stop", and RB is ignored while the robot is speaking (it takes effect when the speech ends).
- **Keep the controller connected.** In Manual, losing it zeroes throttle within 1 s. In **Auto**, voice commands still move the car with no RB to stop it.
- **Voice only moves the car in Auto.** In Manual, or if `driver_node` isn't running, it still *says* the maneuver succeeded.
- Repeats aren't capped ("turn left ten times" = 20 s of motion), so keep requests short.
- Nearby conversation can start a turn, so run it away from crowd noise when you can.

## Debugging

```bash
WOLFWAGEN_DEBUG=1 python3 symposium_demo.py 2>&1 | tee demo.log
```

This timestamps every line and logs transcripts, tool calls, GUI state changes, mic levels (`peak_rms`, where 0 means the mic is silent) and each turn's outcome (`ok` / `failed` / `reset`). To watch what actually reaches the car:

```bash
ros2 topic echo /gemini/throttle
```

| Symptom | Likely cause |
|---|---|
| "ROS2 not available. Running without motor control" | ROS isn't sourced in that terminal |
| Model talks but the car doesn't move | Not in Auto, or the `manual_control` launch isn't running |
| `Connection Error` / `1011 ... quota` | Gemini API key is out of quota |
| No reaction to speech | Check `peak_rms` in debug mode, and the default input device (`pactl info`) |

## Other files

| File | Purpose |
|---|---|
| `wolfwagen_actions.py` | ROS node behind the voice tools (publishes `/gemini/*`, listens for RB on `/joy`) |
| `robotics_er2_test.py` | Standalone test of Gemini Robotics-ER 2 (camera, pointing, boxes, tool calls; never moves the car) |
| `wolfwagen_live_demo.py`, `wolfwagen_live_demo_2.py` | Older versions of the demo |
