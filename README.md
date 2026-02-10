# Project OVAL

**Project On-campus Vehicle Autonomous Launch (OVAL)** is a research and development project aimed at designing an autonomous vehicle capable of navigating NC State University's campus. The project provides a platform for students to test autonomous vehicle algorithms in a real-world environment. Additionally, it opens possibilities for campus applications such as package delivery, security patrols, plant monitoring, or other autonomous tasks.

The system architecture includes:

* **Onboard Computer (Jetson Orin)** – Runs ROS2 nodes for perception, control, and navigation.
* **GUI Computer** – Provides a graphical interface for monitoring and control.
* **AWS EC2 Web Server** – Optional remote server for logging, telemetry, or additional processing.

💡 *Suggested addition:* A system diagram showing the connections between Jetson, sensors, GUI, and AWS could help new users visualize the setup.

---

## Startup Sequence for Onboard Computer (Jetson Orin)

The Jetson Orin runs several ROS2 nodes to interface with the vehicle hardware. The primary nodes are:

* `joy_node` – Reads XBox controller inputs
* `swiftnav_ros2_driver` – Interfaces with the Piksi Multi GNSS
* `zed_camera` – Streams stereo camera data
* `xbox_controller_node` – Handles additional controller features
* `motor_actuation_node` – Sends commands to the motors

💡 *Suggested addition:* Include a table summarizing each node, its purpose, and required hardware.

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

---

### **Message Display**

Show the Team and other messages

```bash
python message_display.py ## To show GUI
python oval_message.py ## To send messages for GUI
```

---

---

### **Gemini Voice Controller**

Control the robot using voice commands

```bash
python wolfwagen_live_demo.py ## To show GUI and listen
```


---

### **Radio Raspberry Pi Control**

Show the Team and other messages

```bash
python telemetry_ros2.py ## To connect to Radio
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

#### Step 1 – Navigate to Project Directory

```bash
cd ~/ros2_ws/src/project-oval
```

#### Step 2 – Clone PyVESC

```bash
git clone https://github.com/LiamBindle/PyVESC.git
cd PyVESC
```

#### Step 3 – Install Dependencies

```bash
python -m pip install -e .
python -m pip install pyserial
```

#### Step 4 – Test Driver Node

```bash
cd ~/ros2_ws/src/project-oval/project-oval
python driver_node.py
```

💡 *Suggested addition:* Include instructions for ROS2 integration with PyVESC and how to verify motor actuation.

---

# GitHub Contribution Guidelines

This repository follows a lightweight but strict Git workflow to keep development organized, reviewable, and stable—especially important for autonomous vehicle development where software, simulation, and hardware intersect.

Please read and follow these rules before contributing.

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

If something doesn’t fit neatly into these rules, communicate early via a pull request or discussion.

### **Additional Recommendations for README**

1. **System Requirements:** List Jetson OS version, ROS2 version, Python version, and minimum hardware requirements.
2. **Startup Script:** Consider creating a single script to launch all nodes in the correct order.
3. **Troubleshooting Section:** Include common error messages for ROS2, ZED, Piksi Multi, and PyVESC.
4. **Data Logging:** Provide instructions for recording sensor or GNSS data for research purposes.
5. **References:** Include links to relevant ROS2, ZED, Piksi, and PyVESC documentation for quick reference.
