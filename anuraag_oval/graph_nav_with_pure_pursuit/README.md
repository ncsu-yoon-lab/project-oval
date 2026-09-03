# Graph Nav With Pure Pursuit

## Terminal Workflow

Open a new terminal for each long-running command. New terminals should source `/home/wolfwagen/.bashrc` automatically. If a terminal is already open, run:

```bash
source /home/wolfwagen/.bashrc
```

## One-Word Commands

These functions are defined in `/home/wolfwagen/.bashrc`.

| Terminal | Command | What it runs |
|---|---|---|
| 1 | `swift` | `ros2 launch swiftnav_ros2_driver start.py` |
| 2 | `zed` | `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i` |
| 3 | `joy` | `ros2 run joy joy_node` |
| 4 | `xbox` | activates the Oval venv, then runs `xbox_controller_node_copy.py` |
| 5 | `driver` | activates the Oval venv, then runs `driver_node.py` |
| 6 | `localize` | `ros2 launch graph_nav_with_pure_pursuit localization.launch.py` |
| 7 | `transform` | publishes `zed_camera_link -> swiftnav-gnss` static TF |
| 8 | `ref` | `ros2 run graph_nav_with_pure_pursuit global_set_reference` |
| 8 | `plan` | `ros2 run graph_nav_with_pure_pursuit global_planner_node` |
| 8 | `pursuit` | `ros2 run graph_nav_with_pure_pursuit pure_pursuit_node` |
| 9 | `throttle` | `ros2 topic echo /gemini/throttle` |
| 10 | `steering` | `ros2 topic echo /gemini/steering` |

Extra checks:

```bash
filtered
fix
```

## Full Order

1. Terminal 1:

```bash
swift
```

2. Terminal 2:

```bash
zed
```

3. Terminal 3:

```bash
joy
```

4. Terminal 4:

```bash
xbox
```

5. Terminal 5:

```bash
driver
```

6. Terminal 6:

```bash
localize
```

7. Terminal 7:

```bash
transform
```

8. Terminal 8, run these in sequence:

```bash
ref
plan
pursuit
```

9. Terminal 9:

```bash
throttle
```

10. Terminal 10:

```bash
steering
```

## Build

After code changes:

```bash
cd /home/wolfwagen/ros2_ws
colcon build --base-paths /home/wolfwagen/ros2_ws/src/project-oval/anuraag_oval/graph_nav_with_pure_pursuit --packages-select graph_nav_with_pure_pursuit
source /home/wolfwagen/ros2_ws/install/setup.bash
```
