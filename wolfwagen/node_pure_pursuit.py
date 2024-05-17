import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses

stdscr = curses.initscr()

current_x = current_y = current_yaw = current_x_goal = current_y_goal = distance_to_waypoint = 0.0

waypoints = []

throttle = 0

steering = 0

radius = 0.5

VICON = True

# Distance between the two wheels of the car (m)
wheel_base = 0.208

# Receives the current x, y, and yaw from the vicon node
def vicon_callback(data):

    global current_x , current_y , current_yaw

    # Current x and y position (-inf, inf) Float
    current_x = data.data[0]
    current_y = data.data[1]

    # Current yaw (-180, 180) Float
    current_yaw = data.data[2]

# Callback to get the waypoints
def ui_callback(data):
    global waypoints
    waypoints = data.data 

def imu_callback(data):
    global throttle
    throttle = data.data


def vector_math(x_target, y_target):
    global current_x , current_y , current_yaw

    yaw = math.radians(current_yaw)

    vector_p = [math.cos(yaw) , math.sin(yaw)]
    vector_r = [x_target - current_x, y_target - current_y]
    
    try:
        theta = math.acos((vector_r[0] * vector_p[0] + vector_r[1] * vector_p[1])/(math.sqrt(vector_r[0] ** 2 + vector_r[1] ** 2) * math.sqrt(vector_p[0] ** 2 + vector_p[1] ** 2)))
    except ZeroDivisionError:
        theta = 0.0
    crossProduct = vector_r[0] * vector_p[1] - vector_r[1] * vector_p[0]

    if crossProduct < 0:
        sign = -1
    else:
        sign = 1

    return theta , sign

def main():
    global current_x, current_y, current_yaw, current_x_goal, current_y_goal, waypoints, radius, distance_to_waypoint, throttle, steering

    rclpy.init()

    pure_pursuit_node = rclpy.create_node('pure_pursuit_node')

    sub_to_ui = pure_pursuit_node.create_subscription(
        Float64MultiArray,  # Replace PoseStamped with the appropriate message type
        'ui_topic',  # Specify the topic you want to subscribe to
        ui_callback,  # Specify the callback function
        1  # queue size
    )

    sub_to_vicon = pure_pursuit_node.create_subscription(
        Float64MultiArray, 
        'vicon_topic', 
        vicon_callback, 
        1)

    pub_pure_pursuit_location = pure_pursuit_node.create_publisher(Float64MultiArray, "pure_pursuit_topic", 1)

    pub_pure_pursuit_motor = pure_pursuit_node.create_publisher(Int64MultiArray, "pure_pursuit_motor_topic", 1)

    thread = threading.Thread(target=rclpy.spin, args=(pure_pursuit_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = pure_pursuit_node.create_rate(FREQ, pure_pursuit_node.get_clock())

    i = 0

    new_time = time.time()

    while rclpy.ok():

        # # Only getting position and yaw every second and that data is being transferred
        # if time.time() - new_time > 0.1:

        #     if len(waypoints) > 0:
        #         current_x_goal = waypoints[i]
        #         i += 1
        #         current_y_goal = waypoints[i]
            
        #     # Calculating the distance between the current position and target position
        #     distance_to_waypoint = math.sqrt((current_x_goal - current_x)**2 + (current_x_goal - current_y)**2)

        #     if distance_to_waypoint < radius:
        #         if i == len(waypoints) - 1:
        #             throttle = 0
        #             pass
        #         else:
        #             i += 1

        #     # The alpha between the current position and the target position
        #     # Measured in radians
        #     theta , sign = vector_math(current_x_goal, current_y_goal)

        #     # Calculating the line of curvature (k) to the goal
        #     k = (2 * math.sin(theta))/distance_to_waypoint

        #     # Creating the steering angle
        #     steering = math.atan(k * wheel_base)

        #     # # Converting the steering angle to be between -100 and 100 relative to the angles of -40 and 40 which is what the robot is capable of
        #     steering = int(math.degrees(steering) * sign)

        #     new_time = time.time() 



        pure_pursuit_location_data = Float64MultiArray()
        pure_pursuit_location_data.data = [current_x, current_y, current_yaw, current_x_goal, current_y_goal, distance_to_waypoint]
        pub_pure_pursuit_location.publish(pure_pursuit_location_data)

        pure_pursuit_motor_data = Int64MultiArray()
        pure_pursuit_motor_data.data = [throttle, steering]
        pub_pure_pursuit_motor.publish(pure_pursuit_motor_data)

        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'Current X :  %.4f		         ' % float(current_x))
        stdscr.addstr(2 , 5 , 'Current Y :  %.4f	                ' % float(current_y))
        stdscr.addstr(3 , 5 , 'Goal X :  %.4f		         ' % float(current_x_goal))
        stdscr.addstr(4 , 5 , 'Goal Y :  %.4f	                ' % float(current_y_goal))
        stdscr.addstr(5 , 5 , 'Throttle :  %.4f		         ' % float(throttle))
        stdscr.addstr(6 , 5 , 'Steering :  %.4f	                ' % float(steering))
        stdscr.addstr(7 , 5 , 'Waypoints : [' + ", ".join(str(x) for x in waypoints) + "]")
        
        rate.sleep()

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
