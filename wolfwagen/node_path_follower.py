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

slopes = []

throttle = 0

steering = 0

VICON = True

exception = ""

lookahead = 1.0

current_segment = 0

previous_error = 0

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
    global waypoints, slopes

    temp_list = data.data

    if (len(temp_list)) % 2 != 0:
        temp_list = temp_list[:-1]
    
    for i in range(0, len(temp_list), 2):
        waypoints.append([temp_list[i], temp_list[i + 1]])

    for i in range(len(waypoints) - 1):
        x1 = waypoints[i][0]
        y1 = waypoints[i][1]
        x2 = waypoints[i + 1][0]
        y2 = waypoints[i + 1][1]

        slopes.append((y2 - y1) / (x2 - x1))

def imu_callback(data):
    global throttle
    throttle = data.data

def get_lookahead(current_x, current_y, waypoints):

    global current_segment, lookahead, slopes

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
            b2 = current_y - m2 * current_x

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
            current_segment = i

        # Sets the closest point coordinate
        if (math.sqrt((x - current_x)**2 + (y - current_y)**2) 
            < math.sqrt((closest_point[0] - current_x)**2 + (closest_point[1] - current_y)**2)):
            closest_point = (x, y)
            current_segment = i

    # After current segment is found and the closest point is found, get the lookahead point
    mag = math.sqrt(1**2 + slopes[current_segment]**2)
    lookahead_unit_vector = (1 / mag, slopes[current_segment] / mag)

    # Multiply the unit vector by the lookahead magnitude to get the look ahead vector
    lookahead_vector = (lookahead_unit_vector[0] * lookahead, lookahead_unit_vector[1] * lookahead)

    # Add the vector to the closest point found previously to get the x,y coordinate of the lookahead
    lookahead_point = (lookahead_vector[0] + closest_point[0], lookahead_vector[1] + closest_point[1])

    ## FAILED LOOP TO CHECK BOUNDS OF NEW LOOK AHEAD POINT (NOT NECESSARY?)
    # While true loop until the lookahead point is known to be within a segment and does not exceed it
    # while True:
    #     # Get the unit vector by finding the magnitude of the vector of the current segment that it is on then dividing the vector of the segment (1, slope) by the magnitude
    #     mag = math.sqrt(1**2 + slopes[current_segment]**2)
    #     lookahead_unit_vector = (1 / mag, slopes[current_segment] / mag)

    #     # Multiply the unit vector by the lookahead magnitude to get the look ahead vector
    #     lookahead_vector = (lookahead_unit_vector[0] * lookahead_mag, lookahead_unit_vector[1] * lookahead_mag)

    #     # Add the vector to the closest point found previously to get the x,y coordinate of the lookahead
    #     lookahead_point = (lookahead_vector[0] + closest_point[0], lookahead_vector[1] + closest_point[1])

    #     # Check that the lookahead does not exceed the bounds of the segment
    #     # Gets the x min and max from the waypoints that it is between
    #     x_min = min(waypoints[current_segment][0], waypoints[current_segment + 1][0])
    #     x_max = max(waypoints[current_segment][0], waypoints[current_segment + 1][0])

    #     # Checks if the max of the lookahead exceeds the max x
    #     if lookahead_point[0] > x_max:

    #         # Checks if it is referring to waypoint it is coming from or going to and gets the x and y of that waypoint
    #         if waypoints[current_segment][0] > waypoints[current_segment + 1][0]:
    #             x = waypoints[current_segment][0]
    #             y = waypoints[current_segment][1]
    #         else:
    #             x = waypoints[current_segment + 1][0]
    #             y = waypoints[current_segment + 1][1]

    #         # Subtracts the distance it has already gone on the current lookahead and adds 1 to the lookahead segment that it is on
    #         lookahead_mag = lookahead_mag - math.sqrt((closest_point[0] - x)**2 + (closest_point[1] - y)**2)
    #         lookahead_segment += 1
        
    #     # Checks if the min of the lookahead exceeds the min x
    #     elif lookahead_point[0] < x_min:

    #         # Checks if it is referring to the waypoint it is coming from or going to and gets the x and y of that waypoint
    #         if waypoints[current_segment][0] < waypoints[current_segment + 1][0]:
    #             x = waypoints[current_segment][0]
    #             y = waypoints[current_segment][1]
    #         else:
    #             x = waypoints[current_segment + 1][0]
    #             y = waypoints[current_segment + 1][1]

    #         # Subtracts the distance it has already gone on the current lookahead and adds 1 to the lookahead segment that it is on
    #         lookahead_mag = lookahead_mag - math.sqrt((closest_point[0] - x)**2 + (closest_point[1] - y)**2)
    #         lookahead_segment += 1

    #     # If the lookahead point is within the range then it breaks from the while loop
    #     else:
    #         break
    
    # Return the point of the lookahead
    return lookahead_point

def steering_PID(current_x, current_y, lookahead_x, lookahead_y, FREQ):
    global waypoints, current_segment, previous_error
    
    adjacent = math.sqrt((waypoints[current_segment][0] - waypoints[current_segment + 1][0])**2 + (waypoints[current_segment][1] - waypoints[current_segment + 1][1])**2)
    hypotenuse = math.sqrt((current_x - lookahead_x)**2 + (current_y - lookahead_y)**2)

    # Theta in degrees
    theta = math.degrees(math.acos(adjacent/hypotenuse))

    Kp = 0.0
    Ki = 0.0
    Kd = 0.0
    dt = 1/float(FREQ)
    integral = 0

    set_point = 0
    error = set_point - theta
    integral = integral + error * dt
    derivative = (error - previous_error) / dt
    steering = Kp * error + Ki * integral + Kd * derivative
    previous_error = error


    if steering > 100:
        steering = 100
    if steering < -100:
        steering = -100

    return steering

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

    closest_y = 0.0
    closest_x = 0.0
    while rclpy.ok():

        if len(waypoints) > 0:
            lookahead_x, lookahead_y = get_lookahead(current_x, current_y, waypoints)

            steering = steering_PID(current_x, current_y, lookahead_x, lookahead_y, FREQ)

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
