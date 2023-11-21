import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import threading
from rclpy.node import Node
import time
import math
import numpy as np
x_pos = y_pos = x_roll = y_pitch = z_yaw = 0.0

x_goals = []
y_goals = []

# Measured in meters
wheel_base = 0.208
def vicon_callback(data):
    global x_pos , y_pos , z_yaw
    # Add your processing logic here

    # "data" has fields for x, y, z, roll, pitch, yaw, and magnitude of rotation
    # Access the fields as follows:
    x_pos = data.pose.position.x
    y_pos = data.pose.position.y
    # z_pos = data.data.data.z
    # x_roll = data.pose.orientation.x # (the roll-value)
    y_pitch = data.pose.orientation.y # (the pitch-value)
    z_yaw = data.pose.orientation.z # (the yaw-value)
    w_mag = data.pose.orientation.w # (magnitude of rotation)

    # Converting from quaternion to euler
    t0 = +2.0 * (w_mag * z_yaw + x_roll * y_pitch)
    t1 = +1.0 - 2.0 * (y_pitch ** 2 + z_yaw ** 2)
    
    z_yaw = math.degrees(math.atan2(t0 , t1)) - 90

def waypoints_callback(data):
    global x_goals, y_goals
    message = data.data
    for i in range(len(message)):
        if i % 2 == 0:
            x_goals.append(message[i])
        else:
            y_goals.append(message[i])
    

## Creating a Subscription
def main():
    global x_pos , y_pos , x_roll , y_pitch , z_yaw , w_mag
    rclpy.init()
    vicon_node = rclpy.create_node('vicon_node')
    subscription = vicon_node.create_subscription(
        PoseStamped,  # Replace PoseStamped with the appropriate message type
        '/zed/zed_node/pose',  # Specify the topic you want to subscribe to
        vicon_callback,  # Specify the callback function
        1  # queue size
    )
    subscription2 = vicon_node.create_subscription(Float64MultiArray, 'waypoints', waypoints_callback, 1)

    pure_pursuit_pub = vicon_node.create_publisher(Float64MultiArray, "pure_pursuit", 1) # publishing one value for now as a test, later change the data type and values


    thread = threading.Thread(target=rclpy.spin, args=(vicon_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = vicon_node.create_rate(FREQ, vicon_node.get_clock())

    # Initializing time
    new_time = time.time()

    i = 0

    radius = 1

    y_goal = 250
    x_goal = 250

    while rclpy.ok():
        if len(y_goals) > 0:
            y_goal = y_goals[i]
            x_goal = x_goals[i]

        # Only getting position and yaw every second and that data is being transferred
        if time.time() - new_time > 1:

            # Calculating the distance between the current position and target position
            distance_to_waypoint = math.sqrt((y_goal - y_pos)**2 + (x_goal - x_pos)**2)

            distance_to_goal = math.sqrt((y_goals[len(y_goals) - 1] - y_pos)**2 + (x_goals[len(x_goals) - 1] - x_pos)**2)

            if distance_to_waypoint < radius:
                if i == len(x_goals) - 1:
                    pass
                else:
                    i += 1


            # The yaw of the robot
            print(f"z_yaw: {z_yaw:.4f} degrees")

            # The alpha between the current position and the target position
            # Measured in radians
            alpha = math.atan((x_goal - x_pos)/(y_goal - y_pos))

            # Path curvature
            k = (2 * math.sin(alpha))/distance_to_waypoint

            # Steering angle
            steering = math.atan(k * wheel_base)

            # Converting steering angle for node_motor_act
            steering = math.degrees(steering) * 100 / 40

            print(f"Distance to goal: {distance_to_waypoint}")
            print(f"Current x_pos: {x_pos}\nCurrent y_pos: {y_pos}")
            print(f"Alpha: {math.degrees(alpha)}")
            print(f"Path curvature: {k}")
            print(f"Steering: {steering}")

            pure_pursuit_data = Float64MultiArray()
            pure_pursuit_data.data = [distance_to_goal, x_pos, y_pos, alpha, k, steering]
            pure_pursuit_pub.publish(pure_pursuit_data)


            new_time = time.time() 

        # if x_pos is not None:
        #     x_data = Float64()
        #     x_data.data = x_pos
        #     pure_pursuit_pub.publish(x_data)

        # if y_pos is not None:
        #     print(y_pos)
        rate.sleep()

    vicon_node.destroy_node()
    rclpy.shutdown()






if __name__ == '__main__':
    main()
