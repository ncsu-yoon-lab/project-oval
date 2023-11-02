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
def vicon_callback(data):
    # Add your processing logic here
    # "data" has fields for x, y, z, roll, pitch, yaw, and scalar
    # Access the fields as follows:
    # data.pose.position.x (the x-value)
    # data.pose.position.y (the y-value)
    # data.pose.position.z (the z-value)
    # data.pose.orientation.x (the roll-value)
    # data.pose.orientation.y (the pitch-value)
    # data.pose.orientation.z (the yaw-value)
    # data.pose.orientation.w (magnitude of rotation)

## Creating a Subscription
import rclpy
from geometry_msgs.msg import PoseStamped

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


Visualizing the Data in RViz2

    Run RViz2 in a terminal.
    Add a Pose display type.
        Initially, the display type may show an error.
        Open the dropdown and check the Topic field.
        If it's empty, click into the empty field or hit the dropdown on the right side of the empty field.
        Select vicon_pos.
        Adjust the dimensions of the Pose arrow if it's too small to see clearly.
