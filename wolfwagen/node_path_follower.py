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

VICON = True

exception = ""

current_segment = None

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

    temp_list = data.data

    if (len(temp_list)) % 2 != 0:
        temp_list = temp_list[:-1]
    
    for i in range(0, len(temp_list), 2):
        waypoints.append([temp_list[i], temp_list[i + 1]])

def imu_callback(data):
    global throttle
    throttle = data.data

def get_lookahead(current_x, current_y, waypoints):

    # Sets the closest point as none to start
    closest_point = (None, None)

    m1 = m2 = None
    
    # If it is the first time the closest point is being found
    for i in range(len(waypoints) - 1):
        waypoint_start = waypoints[i]
        waypoint_end = waypoints[i + 1]

        # Get the slope between the two waypoints
        try:
            m1 = (waypoint_end[1] - waypoint_start[1]) / (waypoint_end[0] - waypoint_start[0])
        except ZeroDivisionError:
            x = waypoint_start[0]
            y = (waypoint_start[1] + waypoint_end[1]) / 2

        # The slope for the perpendicular line to the vehicle is the reciprical
        try:
            m2 = -1.0 / m1
        except ZeroDivisionError:
            x = (waypoint_start[0] + waypoint_end[0]) / 2
            y = waypoint_start[1]

        if m1 is not None and m2 is not None:
            # Calculate the b value for y = mx + b equation between the two waypoints
            b1 = waypoint_start[1] - m1 * waypoint_start[0]

            # Calculate the b value for the y = mx + b equation from the vehicle to the perpendicular point between the two waypoints
            b2 = current_x - m2 * current_y

            # Calculate the x value
            x = (b2 - b1) / (m1 - m2)

            # Calculate the y value
            y = m1 * x + b1
        
        # Minimum and maximum x
        x_min = min(waypoint_start[0], waypoint_end[0])
        x_max = max(waypoint_start[0], waypoint_end[0])

        # Check that they are within the range
        if x < x_min:
            if waypoint_start[0] > waypoint_end[0]:
                x = waypoint_end[0]
                y = waypoint_end[1]
            else:
                x = waypoint_start[0]
                y = waypoint_start[1]
        elif x > x_max:
            if waypoint_start[0] < waypoint_end[0]:
                x = waypoint_end[0]
                y = waypoint_end[1]
            else:
                x = waypoint_start[0]
                y = waypoint_start[1]
        
        # Check for the first iteration
        if closest_point[0] is None:
            closest_point = (x, y)

        if (math.sqrt((x - current_x)**2 + (y - current_y)**2) 
            < math.sqrt((closest_point[0] - current_x)**2 + (closest_point[1] - current_y)**2)):
            closest_point = (x, y)
            
    return closest_point
    

def main():
    global current_x, current_y, current_yaw, current_x_goal, current_y_goal, waypoints, distance_to_waypoint, throttle, steering, exception

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

    look_ahead = 0.1 # meters
    closest_y = 0.0
    closest_x = 0.0
    while rclpy.ok():

        if len(waypoints) > 0:
            look_ahead_y, look_ahead_x = get_lookahead(current_x, current_y, waypoints)

        pure_pursuit_location_data = Float64MultiArray()
        pure_pursuit_location_data.data = [current_x, current_y, current_yaw, current_x_goal, current_y_goal, distance_to_waypoint]
        pub_pure_pursuit_location.publish(pure_pursuit_location_data)

        pure_pursuit_motor_data = Int64MultiArray()
        pure_pursuit_motor_data.data = [throttle, steering]
        pub_pure_pursuit_motor.publish(pure_pursuit_motor_data)

        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'PATH FOLLOWER NODE')
        stdscr.addstr(3 , 5 , 'Current X :  %.4f		         ' % float(current_x))
        stdscr.addstr(4 , 5 , 'Current Y :  %.4f	                ' % float(current_y))
        stdscr.addstr(5 , 5 , 'Goal X :  %.4f		         ' % float(current_x_goal))
        stdscr.addstr(6 , 5 , 'Goal Y :  %.4f	                ' % float(current_y_goal))
        stdscr.addstr(7 , 5 , 'Closest X :  %.4f		         ' % float(closest_x))
        stdscr.addstr(8 , 5 , 'Closest Y :  %.4f	                ' % float(closest_y))
        stdscr.addstr(9 , 5 , 'Throttle :  %.4f		         ' % float(throttle))
        stdscr.addstr(10 , 5 , 'Steering :  %.4f	                ' % float(steering))
        stdscr.addstr(11 , 5 , 'Waypoints : [' + ", ".join(str(x) for x in waypoints) + "]")
        stdscr.addstr(13 , 5 , 'Exception :  %s	                ' % exception)
        
        rate.sleep()

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
