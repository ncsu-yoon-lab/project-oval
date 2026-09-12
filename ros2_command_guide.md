# ROS2 Command Guide

Common `colcon`/`ros2` commands for working in this workspace, with examples using this repo's own packages (`project_oval_control`, `project_oval_perception`, `project_oval_telemetry`, `project_oval_navigation`, `project_oval_bringup`).

## Installing Python dependencies (once per machine)

`colcon build` only builds/copies your Python code — it does **not** install the pip packages listed in each package's `setup.py` (`numpy`, `torch`, `crccheck`, etc.). Skipping this step is why `ros2 run` can fail with `ModuleNotFoundError` even after a successful build. Install them into the same Python that has `rclpy` (system Python, unless your venv was made with `--system-site-packages`):

```bash
python3 -m pip install -r ~/ros2_ws/src/project_oval/requirements.txt
```

## Building

Run these from the **workspace root** (one level above `src/`, e.g. `~/ros2_ws` or `~/Documents/gitprojects/project_oval_ws`) — not from inside `src/project_oval`.

| Command | What it does |
|---|---|
| `colcon build` | Builds every package found under `src/`. |
| `colcon build --packages-select project_oval_control` | Builds only that one package (fast iteration). |
| `colcon build --packages-up-to project_oval_navigation` | Builds that package and everything it depends on. |
| `colcon build --symlink-install` | Symlinks Python files into `install/` instead of copying them — edit-and-rerun without rebuilding. Use this while developing. |
| `colcon build --packages-select project_oval_control --symlink-install` | Combine the two above: fast, and edits take effect immediately. |
| `colcon test` | Runs each package's tests (`ament_flake8`, `ament_pep257`, `pytest`, etc., as declared in `package.xml`). |
| `colcon test-result --verbose` | Shows the output of the last `colcon test` run. |

## Sourcing the workspace (do this before `ros2 run`/`ros2 launch`)

`ros2 run`/`ros2 launch` only know about packages whose `install/` has been **sourced into your current shell**. Building does not do this automatically, and it only affects the shell you built in.

```bash
source /opt/ros/<distro>/setup.bash              # the base ROS2 install (once per shell, usually in .bashrc)
source ~/ros2_ws/install/setup.bash              # this workspace's packages (after every fresh terminal / after building)
```

**`Package 'project_oval_control' not found` almost always means this step was skipped** — check with:

```bash
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep project_oval
```

If nothing prints, the workspace isn't sourced in this shell. Note you can have *multiple* workspaces sourced at once (each `source .../install/setup.bash` appends to `AMENT_PREFIX_PATH`) — if your `.bashrc` auto-sources a different workspace (e.g. `turtlebot3_ws`), you still need to separately source this one before running anything from it.

## Running nodes and launch files

| Command | What it does |
|---|---|
| `ros2 run <package> <executable>` | Runs one node. E.g. `ros2 run project_oval_control driver`. |
| `ros2 run <package> <executable> --ros-args -p <param>:=<value>` | Runs a node overriding a parameter. E.g. `ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0`. |
| `ros2 launch <package> <file>.launch.py` | Runs a launch file (starts multiple nodes together). E.g. `ros2 launch project_oval_navigation rtk_control.launch.py`. |
| `ros2 launch <package> <file>.launch.py <arg>:=<value>` | Passes a launch argument. E.g. `ros2 launch project_oval_navigation localization.launch.py localization_config:=/path/to/file.yaml`. |

## Inspecting the running system

| Command | What it does |
|---|---|
| `ros2 node list` | Lists all currently running nodes. |
| `ros2 node info /driver_node` | Shows a node's publishers, subscribers, services, and parameters. |
| `ros2 topic list` | Lists all active topics. |
| `ros2 topic echo /oval_message` | Prints messages published on a topic, live. |
| `ros2 topic hz /zed/zed_node/imu/data` | Measures how fast a topic is publishing. |
| `ros2 topic info /gemini/steering -v` | Shows a topic's message type and who's publishing/subscribing. |
| `ros2 topic pub /oval_message std_msgs/String "{data: 'test'}"` | Manually publishes one message to a topic — useful for testing a subscriber without the real publisher running. |
| `ros2 interface show std_msgs/msg/String` | Shows the fields of a message type. |
| `ros2 param list /driver_node` | Lists a node's parameters. |
| `ros2 param get /driver_node <name>` / `ros2 param set /driver_node <name> <value>` | Reads/writes a node parameter at runtime. |
| `ros2 service list` | Lists available services. |
| `ros2 pkg list` | Lists every ROS2 package visible in the sourced environment. |
| `ros2 pkg prefix project_oval_control` | Shows where a package was installed to (useful to confirm which workspace it's coming from if you have several sourced). |
| `ros2 pkg executables project_oval_control` | Lists the runnable executables a package declares. |
| `rqt_graph` | GUI showing the live graph of nodes and topics — good for sanity-checking wiring after a launch. |
| `rviz2` | 3D visualization tool (point clouds, TF frames, camera feeds, etc.). |
| `ros2 doctor` | Checks your ROS2 environment for common misconfigurations. |

## Recording and replaying data

| Command | What it does |
|---|---|
| `ros2 bag record -a` | Records every topic to a bag file in the current directory. |
| `ros2 bag record /oval_message /gemini/steering` | Records only the listed topics. |
| `ros2 bag play <bag_dir>` | Replays a recorded bag, republishing its topics as if live. |
| `ros2 bag info <bag_dir>` | Shows what topics/message counts/duration a bag contains. |

## Project-specific examples

```bash
# Bring up manual driving (Xbox controller -> motors)
ros2 run project_oval_control xbox_controller
ros2 run project_oval_control driver

# Bring up the full RTK localization + control stack
ros2 launch project_oval_navigation rtk_control.launch.py

# Watch GPS fixes and steering commands while debugging pursuit
ros2 topic echo /odometry/filtered
ros2 topic echo /gemini/steering

# Simulation
ros2 launch project_oval_bringup car_sim_launch.py
```
