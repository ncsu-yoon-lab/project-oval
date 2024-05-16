import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import threading
from rclpy.node import Node
import time
import math
import numpy as np
from geopy import distance
from coords_to_cartesian import CoordsToCartesian as format
import curses


stdscr = curses.initscr()
x_pos = y_pos = x_roll = y_pitch = z_yaw = 0.0


y_goal = 250
x_goal = 250

x_goals = []
y_goals = []

# Measured in meters
wheel_base = 0.208
def vicon_callback(data):

    #print( "Received data " , data.data )
    global x_pos , y_pos , z_yaw
    # Add your processing logic here

    x_pos = data.data[0]
    y_pos = data.data[1]

    # Receiving yaw in radians
    z_yaw = math.degrees( data.data[2] )

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
    global x_pos , y_pos , x_roll , y_pitch , z_yaw
    rclpy.init()
    vicon_node = rclpy.create_node('vicon_node')
    # subscription = vicon_node.create_subscription(
    #     PoseStamped,  # Replace PoseStamped with the appropriate message type
    #     '/zed/zed_node/pose',  # Specify the topic you want to subscribe to
    #     vicon_callback,  # Specify the callback function
    #     1  # queue size
    # )

    subscription = vicon_node.create_subscription(
        Float64MultiArray,  # Replace PoseStamped with the appropriate message type
        'zed_pos',  # Specify the topic you want to subscribe to
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

    i = 1

    radius = 0.25
    theta = 10000.0
    x_goal = 10000.0
    y_goal = 10000.0
    throttle = 0.0

    while rclpy.ok():
        if len(y_goals) > 0:
            y_goal = y_goals[i]
            x_goal = x_goals[i]
            # print( "xgoal1 : " , x_goal )
            # print( "ygoal1: " , y_goal)
        # Only getting position and yaw every second and that data is being transferred
        if time.time() - new_time > 0.1:


            # Initializing the coordinate system
            # origin = [35.771650, -78.674103]
            # pointY = [35.76926314060246, -78.67596498545836]
            # pointX = [35.77257728014708, -78.67586909648608]

            # Making the coordinate system plane
            # plane = format.get_cartesian( origin, pointX, pointY )
            # plane = CoordsToCartesian(origin, pointX, pointY)

            # coords = [ x_pos, y_pos ]
            # coords = plane.get_cartesian( coords )
            # y_pos = plane.get_cartesian( y_pos )
            
            # Calculating the distance between the current position and target position
            distance_to_waypoint = math.sqrt((x_goal - x_pos)**2 + (y_goal - y_pos)**2)

            distance_to_goal = math.sqrt((x_goals[len(x_goals) - 1] - x_pos)**2 + (y_goals[len(y_goals) - 1] - y_pos)**2)

            if distance_to_waypoint < radius:
                if i == len(x_goals) - 1:
                    throttle = 0
                    pass
                else:
                    throttle = 17
                    i += 1
            
            # if math.degrees(theta) > 120 and i != 1:
            #     i += 1


            # The yaw of the robot
            # print(f"z_yaw: {z_yaw:.4f} degrees")
            # print( "xgoal2 : " , x_goal )
            # print( "ygoal2 : " , y_goal)
            # print( "xcoord : " , x_pos )
            # print( "ycoord: " , y_pos)

            # The alpha between the current position and the target position
            # Measured in radians
            #alpha = math.atan((x_goal - x_pos)/(y_goal - y_pos))
            # alpha = math.atan((x_goal - x_pos)/(y_goal - y_pos)) + math.radians(z_yaw)
            theta , sign = vector_math(x_goal, y_goal)
            # print(f"theta: {math.degrees(theta)}")

            # Calculating the line of curvature (k) to the goal
            k = (2 * math.sin(theta))/distance_to_waypoint

            # # Creating the steering angle
            steering = math.atan(k * wheel_base)
            #steering = theta
           #print(f"Steering before conversion: {math.degrees(steering)}")
            coeff = 400 / 90
            # # Converting the steering angle to be between -100 and 100 relative to the angles of -40 and 40 which is what the robot is capable of
            # steering = math.degrees(steering) * 100 / 40 * sign
            steering = math.degrees(steering) * coeff * sign
           # print(f"Steering after conversion: {steering}")

            # # Path curvature
            # k = (2 * math.sin(alpha))/distance_to_waypoint

            # Steering angle
            # steering = math.atan(k * wheel_base)

            # Converting steering angle for node_motor_act
            # steering = math.degrees(steering) * 40 / 90

            # print(f"Distance to waypoint: {distance_to_waypoint}")
            # print(f"Current x_pos: {x_pos}\nCurrent y_pos: {y_pos}")
            # # print(f"Alpha in radians: {alpha}")
            # # print(f"Alpha in degrees: {math.degrees(alpha)}")
            # print(f"Path curvature: {k}")
            # print(f"Steering: {steering}")
            # print(f"Current goal: {i}")

            pure_pursuit_data = Float64MultiArray()
            pure_pursuit_data.data = [distance_to_goal, x_pos, y_pos, theta, k, steering]
            pure_pursuit_pub.publish(pure_pursuit_data)


            new_time = time.time() 

        # if x_pos is not None:
        #     # x_data = Float64()
        #     # x_data.data = x_pos
        #     # pure_pursuit_pub.publish(x_data)
        #     x_data = Float64()
        #     x_data.data = x_pos
        #     pure_pursuit_data.push( x_data )

        # if y_pos is not None:
        #     print(y_pos)
        stdscr.refresh()
        stdscr.addstr(1 , 5 , 'Current X :  %.4f		         ' % float(x_pos))
        stdscr.addstr(2 , 5 , 'Current Y :  %.4f	                ' % float(y_pos))
        stdscr.addstr(3 , 5 , 'Goal X :  %.4f		         ' % float(x_goal))
        stdscr.addstr(4 , 5 , 'Goal Y :  %.4f	                ' % float(y_goal))

        stdscr.addstr(6 , 5 , 'Theta :  %.4f		         ' % float(math.degrees(theta)))
        stdscr.addstr(7 , 5 , 'Current Goal Num :  %s	                ' % str(i))
        
        rate.sleep()

    vicon_node.destroy_node()
    rclpy.shutdown()

def vector_math(x_target, y_target):
    global x_pos , y_pos , z_yaw

    yaw = math.radians(z_yaw)

    vector_p = [math.cos(yaw) , math.sin(yaw)]
    vector_r = [x_target - x_pos, y_target - y_pos]

    theta = math.acos((vector_r[0] * vector_p[0] + vector_r[1] * vector_p[1])/(math.sqrt(vector_r[0] ** 2 + vector_r[1] ** 2) * math.sqrt(vector_p[0] ** 2 + vector_p[1] ** 2)))

    crossProduct = vector_r[0] * vector_p[1] - vector_r[1] * vector_p[0]

    if crossProduct < 0:
        sign = -1
    else:
        sign = 1

    return theta , sign

if __name__ == '__main__':
    main()
