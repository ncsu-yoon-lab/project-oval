# Project OVAL

**Project On-campus Vehicle Autonomous Launch (OVAL)** is a research and development project aimed at designing an autonomous vehicle capable of navigating NC State University's campus. The project provides a platform for students to test autonomous vehicle algorithms in a real-world environment. Additionally, it opens possibilities for campus applications such as package delivery, security patrols, plant monitoring, or other autonomous tasks.

The system architecture includes:

* **Onboard Computer (Jetson Orin)** – Runs the ROS2 packages in this repo for perception, control, telemetry, and navigation.
* **GUI Computer** – Provides a graphical interface for monitoring and control.
* **AWS EC2 Web Server** – Optional remote server for logging, telemetry, or additional processing.

💡 *Suggested addition:* A system diagram showing the connections between Jetson, sensors, GUI, and AWS could help new users visualize the setup.

---

## Repository Layout

This repo is a ROS2 `colcon` workspace `src/` checkout: every top-level `project_oval_*` folder is an independent `ament_python` package, discovered automatically when you run `colcon build` from the workspace root. There is no single monolithic package anymore — functionality is split by subsystem so each package can be built, tested, and depended on independently.

```
project_oval/                       # this repo (colcon workspace src/ checkout)
├── project_oval_control/           # driving & actuation
├── project_oval_perception/        # camera / ML perception
├── project_oval_telemetry/         # sensors & comms
├── project_oval_navigation/        # graph-based global navigation
├── project_oval_bringup/           # launch files, URDF, Webots world
├── PyVESC/                         # vendored VESC motor-controller protocol library
├── archive/                        # retired subsystems, excluded from the build (COLCON_IGNORE)
├── tools/                          # standalone scripts that aren't ROS packages
├── README.md
├── requirements.txt                # pip dependencies for all packages below (see Building)
├── ros2_command_guide.md           # colcon/ros2 command cheat sheet
└── version_control_guide.md        # day-to-day git command cheat sheet
```

Build everything with:

```bash
python3 -m pip install -r src/project_oval/requirements.txt   # once per machine — colcon doesn't do this for you
cd ~/ros2_ws   # workspace root, one level above src/
colcon build
source install/setup.bash
```

Build a single package while iterating:

```bash
colcon build --packages-select project_oval_control --symlink-install
```

`ros2 run`/`ros2 launch` won't find these packages until you `source install/setup.bash` in your current shell (a fresh terminal needs this every time, or add it to `.bashrc`) — see [`ros2_command_guide.md`](ros2_command_guide.md) for that and other frequently-used `colcon`/`ros2` commands.

### `project_oval_control` — driving & actuation

Turns steering/throttle/lane-following decisions into motor commands, and reads the Xbox controller for manual/teleop driving. Depends on the vendored `PyVESC/` library to talk to the VESC motor controllers over serial, plus `gps_msgs`/`nmea_msgs` for GPS-based pursuit.

| Run with | Purpose |
|---|---|
| `ros2 run project_oval_control driver` | Sends throttle/steering commands to the VESC motor controllers. |
| `ros2 run project_oval_control lane_follower` | PID lane-centering control from a segmented edge-distance signal. |
| `ros2 run project_oval_control xbox_controller` | Reads Xbox controller input (via `joy_node`) and republishes it as drive commands. |

Also included (not standalone nodes): `sim_driver_lib.py`/`sim_driver_node.py` (the `SimDriverNode` Webots plugin, wired up via `project_oval_bringup/urdf/my_robot.urdf` for simulation), `PIDController.py` (shared PID implementation), and `pursuitNode.py`/`test_lib.py` (older GPS-pursuit and manual test scripts, kept for reference — not registered as console scripts).

### `project_oval_perception` — camera & ML perception

Runs the computer-vision and ML models on camera input: sidewalk/lane segmentation and object detection.

| Run with | Purpose |
|---|---|
| `ros2 run project_oval_perception segmentation` | ONNX Runtime lane/sidewalk segmentation on the ZED camera feed. |
| `ros2 run project_oval_perception sidewalk_segmentation` | Transformer-based (Segformer) sidewalk segmentation using the bundled `sidewalk_segmentation_model/` weights. |
| `ros2 run project_oval_perception obstacle_detector` | LaserScan-based obstacle detection; publishes a brake/stop signal. |
| `ros2 run project_oval_perception yolo_od` | YOLO object detection on the camera feed. |
| `ros2 run project_oval_perception zed` | Publishes raw ZED camera images as a ROS2 topic. |

`lane_detector.py`/`simple_lane_detection.py` are shared detection library code (also imported by `project_oval_control`'s `lane_follower_node`). `docs/onnx_runtime_install_guide.md` has the ONNX Runtime GPU install steps for Jetson/ARM64.

> ⚠️ `segmentation_node.py`'s default model path and `test.py` still point at a contributor's personal machine (`/home/wolfwagen1/...`) and reference an ONNX model (`dino_segmentation_2.onnx`) that isn't checked into this repo. Point `model_path` at a real model (e.g. the bundled `sidewalk_segmentation_model/`) before relying on the `segmentation` executable.

### `project_oval_telemetry` — sensors & communications

Reads and republishes sensor/GPS data, bridges to external systems (a serial radio link, Google Cloud Pub/Sub), and displays status.

| Run with | Purpose |
|---|---|
| `ros2 run project_oval_telemetry imu` | Logs ZED IMU data to CSV. |
| `ros2 run project_oval_telemetry fake_rtk` | Publishes a synthetic RTK/GPS fix for testing without hardware. |
| `ros2 run project_oval_telemetry zed_heading` | Computes vehicle heading from magnetometer/IMU data. |
| `ros2 run project_oval_telemetry telemetry_ros2` | Serial bridge node — relays ROS2 messages to/from a radio-connected Raspberry Pi. |
| `ros2 run project_oval_telemetry pub_sub_google` | Publishes telemetry to a Google Cloud Pub/Sub topic. |
| `ros2 run project_oval_telemetry oval_message` | Publishes rotating status/sponsor messages to `/oval_message`. |
| `ros2 run project_oval_telemetry message_display` | Tkinter GUI that displays whatever `oval_message` is currently publishing. |
| `ros2 run project_oval_telemetry map_test` | Manual test script: parses NMEA GPS fixes and renders them on a Folium map. |

`export_imu_csv.py`/`export_rosbag_video.py` are offline analysis scripts for pulling IMU CSVs and depth video out of a recorded rosbag — run directly with `python3`, not as ROS nodes.

### `project_oval_navigation` — graph-based global navigation

A self-contained global-navigation stack: waypoint-graph planning over campus, EKF/RTK localization, pure-pursuit path following, and a browser dashboard for launching/monitoring the other nodes. See `project_oval_navigation/README.md` for the full waypoint/graph format and startup sequence.

| Run with | Purpose |
|---|---|
| `ros2 run project_oval_navigation global_planner_node` | Plans a route through the waypoint graph (`maps/graph_nodes.csv`, `maps/graph_edges.csv`). |
| `ros2 run project_oval_navigation global_set_reference` | Captures a lat/lon reference origin from the current RTK GPS + ZED pose. |
| `ros2 run project_oval_navigation pure_pursuit_node` | Pure-pursuit path following against the planned route. |
| `ros2 run project_oval_navigation oval_web_ui` | Local web dashboard for starting/stopping the nodes above (and the manual driver/Xbox controller from `project_oval_control`). |
| `ros2 launch project_oval_navigation sensors_bringup.launch.py` | Brings up the sensor stack needed for localization. |
| `ros2 launch project_oval_navigation localization.launch.py` | Runs the `robot_localization` EKF/navsat filters. |
| `ros2 launch project_oval_navigation control_nodes.launch.py` | Launches driver + xbox_controller (from `project_oval_control`) + pure pursuit together. |
| `ros2 launch project_oval_navigation rtk_control.launch.py` | Full localization + control stack for RTK-based driving. |

This package depends on `project_oval_control` at run time (for the `driver` and `xbox_controller` executables used by its launch files and web UI).

### `project_oval_bringup` — launch, URDF, simulation world

No nodes of its own — orchestrates the others.

* `launch/car_sim_launch.py` — launches the Webots simulator with `worlds/test_world.wbt` and attaches `project_oval_control`'s `SimDriverNode` via `urdf/my_robot.urdf`.
* `urdf/my_robot.urdf` — robot description used by the Webots driver plugin.
* `worlds/test_world.wbt` — the Webots simulation world.

```bash
ros2 launch project_oval_bringup car_sim_launch.py
```

### Other top-level folders

* **`PyVESC/`** — vendored copy of [LiamBindle/PyVESC](https://github.com/LiamBindle/PyVESC) (VESC motor controller serial protocol). Not a ROS package; `project_oval_control` imports it as a regular Python dependency.
* **`archive/`** — retired code kept for reference, marked with `archive/COLCON_IGNORE` so colcon skips it during a build. Currently holds `project-oval_legacy_sidewalk/`, the earlier LIDAR + sidewalk-following subsystem.
* **`tools/`** — scripts that aren't ROS packages and aren't built by colcon, e.g. `tools/gemini_chatbot/` (voice/chat control demo scripts, run directly with `python3`).

---

## Startup Sequence for Onboard Computer (Jetson Orin)

The Jetson Orin runs several ROS2 nodes to interface with the vehicle hardware. The primary ones are:

| Node | Package | Purpose |
|---|---|---|
| `joy_node` | `joy` (external) | Reads XBox controller inputs |
| `swiftnav_ros2_driver` | `swiftnav_ros2_driver` (external) | Interfaces with the Piksi Multi GNSS |
| `zed_camera` | `zed_wrapper` (external) | Streams stereo camera data |
| `xbox_controller` | `project_oval_control` | Turns joystick input into drive commands |
| `driver` | `project_oval_control` | Sends commands to the motors |

---

### **Joy Node**

Outputs joystick inputs from the XBox controller.

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
```

---

### **Lidar Node**

Outputs Lidar Pointcloud.

```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```

To viz:

```bash
rviz2 ## change to velodyne frame
```

---

### **Message Display**

Show the Team and other messages.

```bash
ros2 run project_oval_telemetry message_display   # shows the GUI
ros2 run project_oval_telemetry oval_message       # publishes messages for the GUI to display
```

---

### **Gemini Voice Controller**

Control the robot using voice commands. This is a standalone script (`tools/`), not a ROS package.

```bash
python3 tools/gemini_chatbot/wolfwagen_live_demo.py ## To show GUI and listen
```

---

### **Radio Raspberry Pi Control**

Bridges ROS2 messages to a radio-connected Raspberry Pi over serial.

```bash
ros2 run project_oval_telemetry telemetry_ros2 ## To connect to Radio
```

---

### **ZED Camera Node**

Outputs ZED camera topics. Documentation: [Stereolabs ZED ROS2 Node](https://www.stereolabs.com/docs/ros/zed-node).

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

#### ZED2i Setup

1. **Check CUDA installation**

```bash
nvidia-smi
```

2. **Install ZED SDK**
   Follow the [official ZED SDK installation instructions](https://www.stereolabs.com/developers/release/latest/) for NVIDIA Jetson. This may take \~30 minutes.

3. **Set up the environment**
   Follow the [ROS2 environment setup instructions](https://www.stereolabs.com/docs/ros2).

💡 *Suggested addition:* Include instructions for testing the ZED camera stream after setup.

---

### **RTK Piksi Multi Setup**

Setup instructions for the SwiftNav Piksi Multi ROS2 package.

#### Step 1 – Installation

Follow the [SwiftNav ROS2 package instructions](https://github.com/swift-nav/swiftnav-ros2).

#### Step 2 – TCP Connection Setup

If TCP connection issues occur, set up the Jetson Ethernet interface manually:

```bash
sudo ip addr add 192.168.0.100/24 dev eno1
ping 192.168.0.222
```

#### Step 3 – Optional USB Setup

USB may be preferable to avoid Ethernet setup. Some RS232-to-USB adapters may not work on the Jetson due to missing drivers.

Check for the PL2303 driver:

```bash
zcat /proc/config.gz | grep -i 'pl2303'
```

If it returns `#CONFIG_USB_SERIAL_PL2303 is not set`, use an FTDI or CH340/CH341 adapter.

#### Step 4 – Run the Node

```bash
ros2 launch swiftnav_ros2_driver start.py
```

💡 *Suggested addition:* Add a small troubleshooting section for common GNSS issues (e.g., no fix, bad TCP connection).

---

### **PyVESC Library Setup**

PyVESC is already vendored in this repo at `PyVESC/` (unmodified copy of [LiamBindle/PyVESC](https://github.com/LiamBindle/PyVESC) v1.0.5) — you don't need to clone it separately.

#### Step 1 – Install it in editable mode

```bash
cd ~/ros2_ws/src/project_oval
python3 -m pip install -e PyVESC/
python3 -m pip install pyserial
```

#### Step 2 – Build and test the driver node

```bash
cd ~/ros2_ws
colcon build --packages-select project_oval_control --symlink-install
source install/setup.bash
ros2 run project_oval_control driver
```

💡 *Suggested addition:* Include instructions for ROS2 integration with PyVESC and how to verify motor actuation.

---

# GitHub Contribution Guidelines

This repository follows a lightweight but strict Git workflow to keep development organized, reviewable, and stable—especially important for autonomous vehicle development where software, simulation, and hardware intersect.

Please read and follow these rules before contributing. For everyday git commands (branching, pulling, pushing), see [`version_control_guide.md`](version_control_guide.md).

---

## 1. Branch Lifespan

**Branches should be short-lived.**

- If you create a new branch, it should be **merged or closed within 7 days**.
- If work is ongoing past a week:
  - Open a pull request with incremental progress, or
  - Clearly note why the branch is staying open.

This helps prevent abandoned branches and keeps development moving.

---

## 2. Pull Requests for All Changes

**All changes must go through pull requests.**

- Any successful or functional change—no matter how small—should be submitted as a pull request.
- Draft pull requests are encouraged for incomplete or in-progress work.
- This applies even if you are the only one working on the branch.

Pull requests are used to track changes to `main`, discuss design decisions, and preserve project history.

---

## 3. One Purpose per Branch

**Each branch should have a single, clear purpose.**

- Avoid mixing unrelated changes in the same branch.
- Examples of unrelated changes:
  - Sensor drivers + visualization tweaks
  - Control logic + documentation cleanup

Keeping branches focused makes reviews, testing, and debugging significantly easier.

---

## 5. Pull Request Descriptions

**Every pull request must include:**

- **What changed**
- **Why it changed**
- **How it was tested**
  - Simulation
  - Hardware
  - Log playback
  - Or explicitly state if it was not tested

Clear PR descriptions save time and reduce misunderstandings.

---

## 6. No Direct Pushes to `main`

**Direct pushes to `main` are not allowed.**

- All changes must be merged through pull requests.
- This rule applies to everyone, regardless of role.

This ensures visibility, review, and traceability of all changes.

---

## 7. Keep `main` Runnable

**The `main` branch must always build and run.**

- Do not merge changes that break:
  - Builds
  - Simulation
  - Deployment pipelines
- If a change introduces a breaking issue, it must be fixed before merging.

`main` should always represent a stable, usable state of the project.

---

## 8. Commit Hygiene

**Commits should be clear and intentional.**

- Use descriptive commit messages.
- Avoid vague messages like:
  - `wip`
  - `fix`
  - `stuff`
- Each commit should represent a coherent, logical change.

Good commit history makes debugging, reverting, and collaboration far easier.

---

## Final Notes

These rules exist to:
- Prevent branch sprawl
- Keep `main` stable
- Make collaboration and review painless
- Save time during testing, deployment, and field work

If something doesn't fit neatly into these rules, communicate early via a pull request or discussion.

### **Additional Recommendations for README**

1. **System Requirements:** List Jetson OS version, ROS2 version, Python version, and minimum hardware requirements.
2. **Startup Script:** Consider creating a single script to launch all nodes in the correct order.
3. **Troubleshooting Section:** Include common error messages for ROS2, ZED, Piksi Multi, and PyVESC.
4. **Data Logging:** Provide instructions for recording sensor or GNSS data for research purposes.
5. **References:** Include links to relevant ROS2, ZED, Piksi, and PyVESC documentation for quick reference.
