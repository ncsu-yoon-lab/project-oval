# Project Oval

Project On-campus Vehicle Autonomous Launch (OVAL) is a project ran with the intentions of desigining an autonomous vehicle that is capable of traveling throughout NC State's campus. With this framework, it can provide research opportunities for students to test on the vehicle as it travels throughout the campus. In addition, it provides the opportunity for the University to adopt it for package delivery, security, plant monitor, or any other autonomous application needed around campus.

Startup requires setting up the ROS2 environment on the on board computer, setting up the GUI on a separate computer, and setting up the EC2 AWS web server on a separate computer or the same one as the GUI

## Startup Sequence For On Board Computer (Jetson)

List of all the nodes needed:
- Joy Node
- Swift GNSS Node
- Zed Node
- XBox Controller Node
- Motor Actuation Node
- RTK Node
- Pure Pursuit Node
- Server Node
- Capture Images Node

Below are the instructions for starting each node

### Joy Node
This node is used to output the joystick outputs from the XBox controller.

```shell
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
```

### Zed Node
This node is used to output the Zed topics which can be found [here](https://www.stereolabs.com/docs/ros/zed-node)

```shell
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### Swift GNSS Node
This node is used to output the Swift GNSS topics. Specifically it outputs /fixgps which contains velocity of the vehicle, position, and yaw.

```shell
ros2 launch swiftnav_ros2_driver start.py
```

### XBOX Controller Node
This node subscribes to the Joy Node messages to output the correct actions depending on what buttons are pushed and movements in the joystick.

```shell
python node_xbox_controller.py
```

### Motor Actuation Node
This node subscribes to the XBox Controller Node to receive manual throttle and steering and if a button is pushed to switch modes. If the mode is switched, it switches to pure pursuit mode where it subscribes to its throttle and steering calculated from the RTK Node.

```shell
python node_motor_act.py
```

### Pure Pursuit Node
This node subscribes to the RTK Node to know its latitude, longitude, yaw, and speed. It also subscribes to the Server Node to receive the waypoints that it must travel to. With this information, it calculates from its current position to the next waypoint, what steering is needed. It also maintains a constant speed by adjusting the throttle (ie increase throttle if going up hill and decrease throttle if going down hill)

```shell
python node_pure_pursuit.py
```

### Server Node
This node receives waypoints from the server and publishes them to the Pure Pursuit Node. It also subscribes to the RTK Node to receive its current position to then send that back over the server.

```shell
python node_server.py
```

### RTK Node
This node subscribes to the Swift GNSS Node to receive the current position, yaw, and speed.

```shell
python node_rtk.py
```

### Camera Node
**This node is only needed if you need photos to be recorded while driving around.** It subscribes to the Zed Node to receive images from the camera and captures them and saves them to a file to help with training.

```shell
python node_capture_images.py
```

## Startup Sequence For GUI and Web Server

- Activate the Kivy Virtual Environment to run the GUI
- [Instructions](https://kivy.org/doc/stable-2.0.0/gettingstarted/installation.html) for setting up kivy virtual environment

### GUI
Change to the GUI directory.

```shell
python main.py
```

### EC2 AWS Webserver
1. Login to the webserver account
2. Begin the instance of the server
3. The server should start, if not follow the below commands

```shell
cd code
```
```shell
python Server.py
```

## Common Issues

### CAN Issues
If node_motor_act.py outputs the following exception:
```shell
failed to transmit [Errno 105] No buffer space available
```
Solution:
```shell
sudo modprobe can_raw
sudo modprobe can
sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 250000
sudo ip link set up can0
sudo busybox devmem 0x0c303018 w 0x458
sudo busybox devmem 0x0c303010 w 0x400
```

## PyVESC Library Setup
Setup process for PyVESC

# Step 1 - Move directories
Move the project-oval root directory
```shell
cd ~/ros2_ws/src/project-oval
```

# Step 2 - Clone PyVESC
[Most current GitHub](https://github.com/LiamBindle/PyVESC/tree/master) 
```shell
git clone https://github.com/LiamBindle/PyVESC/tree/master
cd PyVESC
```

# Step 3 - Install dependencies
```shell
python -m pip install -e .
python -m pip install pyserial
```

# Step 4 - Test driver node
```shell
cd ~/ros2_ws/src/project-oval/project-oval
python driver_node.py
```