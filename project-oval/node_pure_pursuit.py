import rclpy
from std_msgs.msg import Float64MultiArray, Int64MultiArray, Float64, Int64
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import curses

stdscr = curses.initscr()

current_x = closest_x = closest_y = current_y = current_z = current_yaw = current_x_goal = current_y_goal = distance_to_waypoint = 0.0
waypoints = []
slopes = []
current_segment = 0
previous_error = 0
waypoints_received = False

throttle = 16
steering = 0
lookahead = 0.5
VICON = False

exception = ""


# Receives the current x, y, and yaw from the vicon node
def vicon_callback(data):
    global current_x, current_y, current_z, current_yaw

    # Current x and y position (-inf, inf) Float
    current_x = data.data[0]
    current_y = data.data[1]
    current_z = data.data[2]

    # Current yaw (-180, 180) Float
    current_yaw = data.data[3]


# Callback to get the waypoints
def ui_callback(data):
    global waypoints, slopes, waypoints_received
    if not waypoints_received:
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

            # If vertical line, catch it and make it a large slope
            try:
                slopes.append((y2 - y1) / (x2 - x1))
            except ZeroDivisionError:
                slopes.append(20000)

        waypoints_received = True

# Callback for the server node to get the waypoints
def server_callback(data):

    global waypoints, slopes, waypoints_received

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

        # If vertical line, catch it and make it a large slope
        try:
            slopes.append((y2 - y1) / (x2 - x1))
        except ZeroDivisionError:
            slopes.append(20000)

    waypoints_received = True


def get_lookahead(current_x, current_y, waypoints):
    global current_segment, lookahead, slopes, closest_x, closest_y

    # Sets the closest point as none to start
    closest_point = (None, None)

    m1 = m2 = None

    # If it is the first time the closest point is being found
    for i in range(len(waypoints) - 1):
        waypoint_start = waypoints[i]
        waypoint_end = waypoints[i + 1]

        # Get the slope between the two waypoints
        if waypoint_end[0] - waypoint_start[0] == 0:
            x = float(waypoint_start[0])
            y = current_y
        elif waypoint_end[1] - waypoint_start[1] == 0:
            x = current_x
            y = float(waypoint_start[1])
        else:
            m1 = float(waypoint_end[1] - waypoint_start[1]) / float(waypoint_end[0] - waypoint_start[0])
            m2 = -1.0 / m1

            # Calculate the b value for y = mx + b equation between the two waypoints
            b1 = waypoint_start[1] - m1 * waypoint_start[0]

            # Calculate the b value for the y = mx + b equation from the vehicle to the perpendicular point between the two waypoints
            b2 = current_y - m2 * current_x

            # Calculate the x value
            x = (b2 - b1) / (m1 - m2)

            # Calculate the y value
            y = m1 * x + b1

        # Finding the distance between closest point and each of the waypoints
        segment_length = distance(waypoint_start[0], waypoint_start[1], waypoint_end[0], waypoint_end[1])
        point_to_start = distance(x, y, waypoint_start[0], waypoint_start[1])
        point_to_end = distance(x, y, waypoint_end[0], waypoint_end[1])

        if point_to_start > segment_length:
            x = waypoint_end[0]
            y = waypoint_end[1]
        elif point_to_end > segment_length:
            x = waypoint_start[0]
            y = waypoint_start[1]

        # Check for the first iteration
        if closest_point[0] is None:
            closest_point = (x, y)
            current_segment = i

        # Sets the closest point coordinate
        if (distance(x, y, current_x, current_y) < distance(closest_point[0], closest_point[1], current_x, current_y)):
            closest_point = (x, y)
            current_segment = i

    closest_x = closest_point[0]
    closest_y = closest_point[1]

    # Initializing lookahead_segment and lookahead_mag
    lookahead_segment = current_segment
    lookahead_mag = lookahead

    # While true loop until the lookahead point is known to be within a segment and does not exceed it
    while True:
        
        # After current segment is found and the closest point is found, get the lookahead point
        mag = math.sqrt(1.0 ** 2 + slopes[lookahead_segment] ** 2)
        lookahead_unit_vector = (1.0 / mag, slopes[lookahead_segment] / mag)

        # Multiply the unit vector by the lookahead magnitude to get the look ahead vector
        lookahead_vector = (lookahead_unit_vector[0] * lookahead_mag, lookahead_unit_vector[1] * lookahead_mag)

        # Check to see if the vector is going to the right or the left
        if waypoints[lookahead_segment][0] > waypoints[lookahead_segment + 1][0]:
            sign = -1.0
        else:
            sign = 1.0

        # Add the vector to the closest point found previously to get the x,y coordinate of the lookahead
        lookahead_point = (sign * lookahead_vector[0] + closest_point[0], sign * lookahead_vector[1] + closest_point[1])

        # Get the length of the current segment
        segment_length = distance(waypoints[lookahead_segment + 1][0], waypoints[lookahead_segment + 1][1],
                                  closest_point[0], closest_point[1])

        # If the length of the lookahead is less than the length of the segment, return lookahead point, else continue in while loop
        if (segment_length >= lookahead_mag):
            return lookahead_point
        else:
            lookahead_mag -= segment_length
            closest_point = (waypoints[lookahead_segment + 1][0], waypoints[lookahead_segment + 1][1])
            lookahead_segment += 1

# Calculates the distance between two x y points
def distance(x1, y1, x2, y2):
    dist = math.sqrt((float(y2 - y1)) ** 2 + (float(x2 - x1)) ** 2)
    return dist

# PID for calculating the speed, but just doing proportional gain
def speed_callback(data):
    global throttle
    throttle = data.data

    # Bounds throttle
    if throttle >= 50:
        throttle = 50

# Callback to get the current position
def pos_callback(data):
    global current_x, current_y, current_z, current_yaw

    # Current x and y position (-inf, inf) Float
    current_x = data.data[0]
    current_y = data.data[1]
    current_z = data.data[2]

    # Current yaw (-180, 180) Float
    current_yaw = data.data[3]

# PID for adjusting the steering
def steering_PID(current_x, current_y, lookahead_x, lookahead_y, FREQ):
    global current_yaw, previous_error, exception

    # Getting the angle of the vector from current position to lookahead position
    delta_x = lookahead_x - current_x
    delta_y = lookahead_y - current_y

    # Doing the cross product between vector to look ahead point and yaw to get the sign if phi should be negative or positive
    cp = delta_x * math.sin(math.radians(current_yaw)) - delta_y * math.cos(math.radians(current_yaw))
    if cp > 0:
        sign = -1.0
    else:
        sign = 1.0

    theta = sign * math.degrees(math.acos(
        (delta_x * math.cos(math.radians(current_yaw)) + delta_y * math.sin(math.radians(current_yaw))) / distance(
            lookahead_x, lookahead_y, current_x, current_y)))

    # K values of PID
    Kp = 1.0
    Ki = 0.0
    Kd = 0.0
    dt = 1 / float(FREQ)
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

    return int(-1 * steering)


def main():
    global current_x, current_y, current_yaw, current_x_goal, current_y_goal, waypoints, distance_to_waypoint, throttle, steering, closest_x, closest_y, exception

    rclpy.init()

    pure_pursuit_node = rclpy.create_node('pure_pursuit_node')

    if VICON:
        sub_to_ui = pure_pursuit_node.create_subscription(
            Float64MultiArray,  # Replace PoseStamped with the appropriate message type
            'ui_topic',  # Specify the topic you want to subscribe to
            ui_callback,  # Specify the callback function
            1  # queue size
        )

        sub_to_vicon = pure_pursuit_node.create_subscription(
            Float64MultiArray,
            'pos_topic',
            vicon_callback,
            1)
    else:
        sub_to_rtk_pos = pure_pursuit_node.create_subscription(
            Float64MultiArray,
            'pos_topic',
            pos_callback,
            1
        )

        sub_to_rtk_speed = pure_pursuit_node.create_subscription(
            Int64,
            'speed_topic',
            speed_callback,
            1
        )

        sub_to_server = pure_pursuit_node.create_subscription(
            Float64MultiArray,
            'waypoints_topic',
            server_callback,
            1
        )

    pub_pure_pursuit_location = pure_pursuit_node.create_publisher(Float64MultiArray, "pure_pursuit_topic", 1)

    pub_pure_pursuit_motor = pure_pursuit_node.create_publisher(Int64MultiArray, "pure_pursuit_motor_topic", 1)

    pub_pure_pursuit_logger = pure_pursuit_node.create_publisher(Float64MultiArray, 'pure_pursuit_logging_topic', 1)

    thread = threading.Thread(target=rclpy.spin, args=(pure_pursuit_node,), daemon=True)
    thread.start()

    FREQ = 10
    rate = pure_pursuit_node.create_rate(FREQ, pure_pursuit_node.get_clock())
    lookahead_x = 0.0
    lookahead_y = 0.0

    start_time = time.time()

    while rclpy.ok():

        if len(waypoints) > 0:
            lookahead_x, lookahead_y = get_lookahead(current_x, current_y, waypoints)
            steering = steering_PID(current_x, current_y, lookahead_x, lookahead_y, FREQ)

        pure_pursuit_location_data = Float64MultiArray()
        pure_pursuit_location_data.data = [current_x, current_y, current_yaw, current_x_goal, current_y_goal,
                                           distance_to_waypoint]
        pub_pure_pursuit_location.publish(pure_pursuit_location_data)

        # Publishes the motor actuations of the pure pursuit node
        pure_pursuit_motor_data = Int64MultiArray()
        pure_pursuit_motor_data.data = [throttle, steering]
        pub_pure_pursuit_motor.publish(pure_pursuit_motor_data)

        # Publishes the pure pursuit data to be logged
        pure_pursuit_logger_data = Float64MultiArray()
        pure_pursuit_logger_data.data = [current_x, current_y, current_yaw, current_x_goal, current_y_goal, lookahead_x, lookahead_y]
        pub_pure_pursuit_logger.publish(pure_pursuit_logger_data)

        stdscr.refresh()
        stdscr.addstr(1, 5, 'PURE PURSUIT NODE')

        stdscr.addstr(3, 5, 'Current X :  %.4f		         ' % float(current_x))
        stdscr.addstr(4, 5, 'Current Y :  %.4f	                ' % float(current_y))
        stdscr.addstr(5, 5, 'Lookahead X :  %.4f		         ' % float(lookahead_x))
        stdscr.addstr(6, 5, 'Lookahead Y :  %.4f	                ' % float(lookahead_y))
        stdscr.addstr(7, 5, 'Closest X :  %.4f		         ' % float(closest_x))
        stdscr.addstr(8, 5, 'Closest Y :  %.4f	                ' % float(closest_y))

        stdscr.addstr(10, 5, 'Throttle :  %d		         ' % throttle)
        stdscr.addstr(11, 5, 'Steering :  %d	                ' % steering)

        stdscr.addstr(13, 5, 'Exception :  %s	                ' % str(exception))
        stdscr.addstr(14, 5, 'Time :  %.4f                      ' % float(time.time() - start_time))

        rate.sleep()

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
