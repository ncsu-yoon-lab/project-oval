import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses
import os


current_x = current_y = current_yaw = current_goal_x = current_goal_y = distance_to_waypoint = 0.0

waypoints = []


def pure_pursuit_callback(data):
    global current_x, current_y, current_yaw, current_goal_x, current_goal_y

    # Current x and y position (-inf, inf) Float
    current_x = data.data[0]
    current_y = data.data[1]

    # Current yaw (-180, 180) Float
    current_yaw = data.data[2]

    # Current x and y goal position (-inf, inf) Float
    current_goal_x = data.data[3]
    current_goal_y = data.data[4]

# Allows the user to input as many x and y points until they enter x. It then allows the user to double check the results
def user_input():
    global waypoints

    clean_x_input = clean_y_input = False

    # Continue getting inputs until x is input
    while True:
        x = input("X: ")
        if x == "x":
            break
        else:
            try:
                x = float(x)
                clean_x_input = True
            except ValueError:
                clean_x_input = False

        y = input("Y: ")
        if y == "x":
            break
        else:
            try:
                y = float(y)
                clean_y_input = True
            except ValueError:
                clean_y_input = False
        
        if clean_x_input and clean_y_input:
            waypoints.append(x)
            waypoints.append(y)
            print("Points added")
        else:
            print("Invalid input")

    # Double check that the waypoints are correct
    print(waypoints)
    all_good = input("Enter x if the points are incorrect: ")
    if all_good == "x":
        waypoints = []
        user_input()


def main():
    global current_x, current_y, current_yaw, current_goal_x, current_goal_y

    rclpy.init()
    ui_node = rclpy.create_node('ui_node')

    sub_to_pure_pursuit = ui_node.create_subscription(
        Float64MultiArray,  # Replace PoseStamped with the appropriate message type
        'pure_pursuit_topic',  # Specify the topic you want to subscribe to
        pure_pursuit_callback,  # Specify the callback function
        1  # queue size
    )

    pub_waypoints_to_pure_pursuit = ui_node.create_publisher(Float64MultiArray, "ui_topic", 1) # publishing one value for now as a test, later change the data type and values

    thread = threading.Thread(target=rclpy.spin, args=(ui_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = ui_node.create_rate(FREQ, ui_node.get_clock())

    print("Enter first location (Ex: 1.54)")
    print("Enter x when you are done")

    user_input()
    
    os.system('clear')

    stdscr = curses.initscr()
        
    while rclpy.ok():
        msg = Float64MultiArray()
        msg.data = waypoints
        pub_waypoints_to_pure_pursuit.publish(msg)
        stdscr.refresh()
        stdscr.addstr(1, 5, 'UI NODE')

        stdscr.addstr(3 , 5 , 'Current X :  %.4f		         ' % float(current_x))
        stdscr.addstr(4 , 5 , 'Current Y :  %.4f	                ' % float(current_y))
        stdscr.addstr(5 , 5 , 'Yaw :  %.4f		         ' % float(current_yaw))
        stdscr.addstr(6 , 5 , 'Current Waypoint X :  %.4f		         ' % float(current_goal_x))
        stdscr.addstr(7 , 5 , 'Current Waypoint Y :  %.4f	                ' % float(current_goal_y))
        stdscr.addstr(8, 5, 'Distance to next waypoint:  %.4f            ' % float(distance_to_waypoint))

        rate.sleep()

    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
