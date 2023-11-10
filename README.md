### When working on new features, please switch to the development branch and create your feature branch from the development branch until your feature is ready to be merged to the development branch

Steps to do so are [Here](version_control_guide.md)

# Wolfwagen Startup Sequence

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
python3 node_xbox_controller.py
```

## how to connect controller
See [this](https://github.com/atar-axis/xpadneo#connection)

## driver (pwm_gen)
```shell
python3 node_motor_act.py
```

## Lane following (detection + PID)
```shell
python3 node_process_image.py
```

## stop sign detection
```shell
python3 node_stop_sign_detect.py
```

## rplidar s2
```shell
ros2 launch sllidar_ros2  sllidar_s2_launch.py
```

## Obstacle detector (LIDAR-based)
```shell
python3 node_od.py
```

## rosbridge_server 
```shell
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## voice commander
Don't forget to run it on mk laptop



# Instructions for Setting Up Cameras and Using Vicon Tracker

## Setting Up Cameras
1. Place the network switch on the left side of the desk with the Alienware computer.
2. Plug in the power cord to the back of the network switch.
3. Log in to the computer.
4. Run the VICON TRACKER application.
5. Wait for all the cameras to turn blue, indicating they are ready to use. If cameras are red or purple, troubleshoot the issue.

## Running the Vicon Node
1. Open a terminal on your car.
2. Navigate to the appropriate directory:
    ```bash
    cd ~/ros2_ws2/src/wolfwagen/wolfwagen
    ```
3. Run the Vicon node:
    ```bash
    python3 wolfwagen_vicon.py
    ```
4. The position and orientation data will print in the terminal. If you see "Car not being picked up by camera, re-run the program, or make sure the car is in view of the cameras," there is an error.
## Subscribing to the Data
You can also create a subscriber in a different file/node. Create a callback function to handle incoming data:
```python

import rclpy
from geometry_msgs.msg import PoseStamped

def vicon_callback(data):
    # Add your processing logic here

    # "data" has fields for x, y, z, roll, pitch, yaw, and magnitude of rotation
    # Access the fields as follows:
    # data.pose.position.x (the x-value)
    # data.pose.position.y (the y-value)
    # data.pose.position.z (the z-value)
    # data.pose.orientation.x (the roll-value)
    # data.pose.orientation.y (the pitch-value)
    # data.pose.orientation.z (the yaw-value)
    # data.pose.orientation.w (magnitude of rotation)

## Creating a Subscription
def main():
    rclpy.init()
    vicon_node = rclpy.create_node('vicon_node')
    subscription = vicon_node.create_subscription(
        PoseStamped,  # Replace PoseStamped with the appropriate message type
        'vicon_pos',  # Specify the topic you want to subscribe to
        vicon_callback,  # Specify the callback function
        1  # Queue size
    )
    rclpy.spin(vicon_node)

if __name__ == '__main__':
    main()
```
## Visualizing the Data in RViz2
Follow these steps to visualize the Vicon data using RViz2:
1. **Run RViz2:** Open a terminal and enter the following command to start RViz2.
    ```bash
    rviz2
    ```
2. **Add a Pose Display Type:**
   - When you first add the Pose display type, it might display an error.
   - Open the dropdown menu of the Pose display.
   - **Check the Topic Field:**
     - If the Topic field is empty, click into the field, or click the dropdown arrow on the right side of the empty field.
     - **Select the Vicon Topic:**
       - Choose `vicon_pos` from the available topics.
     - **Adjust the Dimensions:**
       - If the Pose arrow is too small to see clearly, adjust its dimensions until the visualization is clear and understandable.

