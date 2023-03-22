`# Wolfwagen

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

## xbox_controller
```shell
python xbox_controller.py
```

## driver (pwm_gen)
```shell
python pwm_genV3.py
```

## Lane following (detection + PID)
```shell
python LaneDetectionV6.py
```

## stop sign detection
```shell
cd stop_sign_detection
python stop_sign_detect_node.py 
```

## rplidar s2
```shell
ros2 launch sllidar_ros2  sllidar_s2_launch.py
```

## Obstacle detector (LIDAR-based)
```shell
python obstacle_detector.py 
```

## rosbridge_server 
```shell
python ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```