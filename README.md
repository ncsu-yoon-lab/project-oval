# Project Oval

Project On-campus Vehicle Autonomous Launch (OVAL) is a project ran with the intentions of desigining an autonomous vehicle that is capable of traveling throughout NC State's campus. With this framework, it can provide research opportunities for students to test on the vehicle as it travels throughout the campus. In addition, it provides the opportunity for the University to adopt it for package delivery, security, weed monitor, or any other autonomous application needed around campus.

Startup requires setting up the ROS2 environment on the on board computer, setting up the GUI on a separate computer, and setting up the EC2 AWS web server on a separate computer or the same one as the GUI

# Startup Sequence For On Board Computer (Jetson)

List of all the nodes needed:
- Joy Node
- Swift GNSS Node
- Zed Node
- XBOX Controller Node
- Motor Actuation Node
- RTK Node
- Pure Pursuit Node
- Server Node
- Capture Images Node

Below are the instructions for starting each node

## Joy Node
```shell
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
```

## XBOX Controller Node
```shell
python node_xbox_controller.py
```

## Zed Node
```shell
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

## Motor Actuation Node
```shell
python node_motor_act.py
```

## RTK Node
```shell
python node_rtk.py
```

## Pure Pursuit Node
```shell
python node_pure_pursuit.py
```

## Server Node
```shell
python node_server.py
```

## Camera Node
```shell
python node_capture_images.py
```

# Startup Sequence For GUI and Web Server

- Activate the Kivy Virtual Environment to run the GUI
- [Instructions](https://kivy.org/doc/stable-2.0.0/gettingstarted/installation.html) for setting up kivy virtual environment

## GUI
```shell
python main.py
```

## EC2 AWS Webserver
1. Login to the webserver account
2. Begin the instance of the server
3. The server should start, if not follow the below commands

```shell
cd code
```
```shell
python Server.py
```

# Common Issues

## CAN Issues
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

