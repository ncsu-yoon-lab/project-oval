# Wolfwagen

## Enabling CAN
```shell
sudo modprobe can_raw
sudo modprobe can
sudo modprobe mttcan
sudo ip link set can0 up type can bitrate 250000
sudo ip link set up can0
sudo busybox devmem 0x0c303018 w 0x458
sudo busybox devmem 0x0c303010 w 0x400
```
See [this](https://forums.developer.nvidia.com/t/jetson-orin-can-bus-access/221728/3)

## ZED 2i node
```shell
ros2 launch zed_wrapper zed2i.launch.py
```

## joy node
```shell
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
```
If autorepeat_rate>0, joy topics will be published even when the joystick does not change. 
See [this](https://index.ros.org/p/joy/)
