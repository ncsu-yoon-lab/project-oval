# ROS Version: ROS 2 Foxy Fitzroy

############## IMPORT LIBRARIES #################
# Python math library
import math

# ROS client library for Python
import rclpy

# Enables pauses in the execution of code
from time import sleep

# Used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)
from sensor_msgs.msg import LaserScan

# Handle Pose messages
from geometry_msgs.msg import Pose

# Handle float64 arrays
from std_msgs.msg import Float64MultiArray

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data

# Scientific computing library
import numpy as np

import random


class Controller(Node):
    """
    Create a Controller class, which is a subclass of the Node
    class for ROS2.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        ##################### ROS SETUP ####################################################
        # Initiate the Node class's constructor and give it a name
        super().__init__("Controller")

        # Create a subscriber
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /demo/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Float64MultiArray, "/demo/state_est", self.state_estimate_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Create a subscriber
        # This node subscribes to messages of type
        # sensor_msgs/LaserScan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            "/demo/laser/out",
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Create a publisher
        # This node publishes the desired linear and angular velocity of the robot (in the
        # robot chassis coordinate frame) to the /demo/cmd_vel topic. Using the diff_drive
        # plugin enables the robot model to read this /demo/cmd_vel topic and execute
        # the motion accordingly.
        self.publisher_ = self.create_publisher(Twist, "/demo/cmd_vel", 10)

        # Initialize the LaserScan sensor readings to some large value
        # Values are in meters.
        self.left_dist = 999999.9  # Left
        self.leftfront_dist = 999999.9  # Left-front
        self.front_dist = 999999.9  # Front
        self.rightfront_dist = 999999.9  # Right-front
        self.right_dist = 999999.9  # Right

        # max sensor reading
        self.max_dist = 7.0

        ################### ROBOT CONTROL PARAMETERS ##################
        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed = 0.05

        # Turning speed of the car
        self.turning_speed = 1.0

        # Current position and orientation of the robot in the global
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        ############# WALL FOLLOWING PARAMETERS #######################
        # Finite states for the wall following mode
        #  "turn left": Robot turns towards the left
        #  "search for wall": Robot tries to locate the wall
        #  "follow wall": Robot moves parallel to the wall
        self.wall_following_state = "turn left"

        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        # self.turning_speed_wf_fast = 2.0  # Fast turn
        # self.turning_speed_wf_slow = 0.05  # Slow turn

        #start state
        self.start = (self.current_x, self.current_y)

        #goal state
        self.goal = (30.0, 30.0)

        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.5  # in meters

        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.2  # in meters

        # costToCome - hashmap to store the distance of the nodes from the start node
        self.costToCome = {}
        
        # path - hashmap used for backtracking from the goal node to the start node
        self.path = {}
        
        # goalThreshold - threshold from goal node
        self.goalThreshold = 2
        
        # vertices of the graph
        self.vertices = []
        
        # step size
        self.stepSize = 3
        
        # step factor
        self.stepFactor = 2

        # whether or not the goal state was reached
        self.goal_reached = False


    def state_estimate_callback(self, msg):
        """
        Extract the position and orientation data.
        This callback is called each time
        a new message is received on the '/demo/state_est' topic
        """
        # Update the current estimated state in the global reference frame
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

        self.start = (self.current_x, self.current_y)

        # # Command the robot to keep following the wall
        # self.follow_path(self.backtrack)

        self.search()

    def scan_callback(self, msg):
        """
        This method gets called every time a LaserScan message is
        received on the '/demo/laser/out' topic
        """
        # Read the laser scan data that indicates distances
        # to obstacles (e.g. wall) in meters and extract
        # 5 distinct laser readings to work with.
        # Each reading is separated by 45 degrees.
        # Assumes 181 laser readings, separated by 1 degree.
        # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)

        # number_of_laser_beams = str(len(msg.ranges))
        self.left_dist = msg.ranges[180]
        self.leftfront_dist = msg.ranges[135]
        self.front_dist = msg.ranges[90]
        self.rightfront_dist = msg.ranges[45]
        self.right_dist = msg.ranges[0]

        self.max_dist = msg.range_max


    # beginning of RRT implementation


    # eucledian heuristic
    def euc_heuristic(self, point1, point2):
        """
        Inputs:
        
        point1: the first position of the robot, tuple (x, y).
        point2: the second posiiton of the robot, tuple (x, y).
        
        Output:
        
        Returns the eucledian distance between point1 and point2
        """
        return (np.sqrt(((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2)))

    # checks if a node is within an obstacle
    def isObstacle(self, x, y):
        """
        Inputs:
        
        x - the x position of the node.
        y - the y position of the node.
        
        Outputs:
        
        True / False depending on whether the node lies within obstacle or not.
        """
        # max = LaserScan.range_max

        d = self.dist_too_close_to_wall

        curr_x = self.current_x
        curr_y = self.current_y

        # if the node is somehow outside of the max laser range
        if self.euc_heuristic((x, y), (curr_x, curr_y)) > self.max_dist:
            return True
        
        # if laser reading is less than max range, aka there is an obstacle in front in a 90 degree sweep
        if self.front_dist < self.max_dist or self.leftfront_dist < self.max_dist or self.rightfront_dist < self.max_dist:
            if self.euc_heuristic((x, y), (curr_x, curr_y)) + d > self.front_dist:
                return True
            elif self.euc_heuristic((x, y), (curr_x, curr_y)) + d > self.leftfront_dist:
                return True
            elif self.euc_heuristic((x, y), (curr_x, curr_y)) + d > self.rightfront_dist:
                return True
        
        return False # point is not in an obstacle
    
            # check obstacle between points
    def checkObstacleBetweenPoints(self, point1, point2):
        """
        Inputs:
        
        point1: the first position of the robot, tuple (x, y).
        point2: the second posiiton of the robot, tuple (x, y).
        
        Output:
        
        Returns True/False, whether an obstacle occurs between point1 and point2 or not
        """

        # get diff1 and diff2
        diff1 = point2[0] - point1[0]
        diff2 = point2[1] - point1[1]
        
        # points to check for obstacle
        points_to_check = []
        points_to_check.append(point1)
        
        # get value of diff
        if(np.abs(diff1) > np.abs(diff2)):
            diff = np.abs(diff1)
        else:
            diff = np.abs(diff2)
        
        for index in range(1, int(np.abs(diff))):
            point = (point1[0] + (index * diff1 / np.abs(diff)), point1[1] + (index * diff2 / np.abs(diff)))
            points_to_check.append(point)
        
        # check for obstacle at every point between point1 and point2
        for point in points_to_check:
            if(self.isObstacle(point[0], point[1])):
                return True
        return False
    
    # get a sub goal, which is a point in the current sensor window to travel to
    def getSubGoal(self, curr_pos, goal):
        """
        Inputs:

        curr_pos: curr position of the car
        goal: final goal state

        Output:

        If the final goal state is within the current sensor window, return the goal state. If not, then 
        return the intersection between the window and the line between the current position and the goal state.
        If the intersection point is within an obstacle, move the the point left and right along the path of the 
        max sensor range until it is not within the obstacle, and return that point. 
        
        """


        # if final goal is within current window, return final goal
        print(self.euc_heuristic(curr_pos, goal))
        print(self.max_dist)

        if self.euc_heuristic(curr_pos, goal) < self.max_dist:
            return goal
        
        # find intersection between window and line between curr_pos and goal using unit vectors
        vector_to_goal = (goal[0] - curr_pos[0], goal[1] - curr_pos[1])
        norm = math.sqrt(vector_to_goal[0] ** 2 + vector_to_goal[1] ** 2)
        direction = (vector_to_goal[0] / norm, vector_to_goal[1] / norm) # unit vector

        print("dist = ", self.max_dist)

        # line that leads from current position to the point of intersection -> d*u
        line_to_intersection = tuple(i * int(self.max_dist) for i in direction)

        print("line = ", line_to_intersection)

        # new point -> (x0,y0) + d*u
        subgoal = tuple(map(lambda i, j: i + j, curr_pos, line_to_intersection))

        print("subgoal = ", subgoal)

        # if subgoal is not inside an obstacle
        if not self.isObstacle(subgoal[0], subgoal[1]):
            return subgoal
        # if subgoal is inside an obstacle
        else:
            pointleft = subgoal
            pointright = subgoal
            angle = 5 #degrees
            radius = self.max_dist
            
            while self.isObstacle(pointleft[0], pointleft[1]) and self.isObstacle(pointright[0], pointright[1]):
                # move left along the max range of the sensor
                templeft = list(pointleft)
                templeft[0] = curr_pos[0] + math.cos(math.radians(angle)) * radius
                templeft[1] = curr_pos[1] + math.sin(math.radians(angle)) * radius
                pointleft = tuple(templeft)

                # move right along the max range of the sensor
                tempright = list(pointright)
                tempright[0] = curr_pos[0] - math.cos(math.radians(angle)) * radius
                tempright[1] = curr_pos[1] - math.sin(math.radians(angle)) * radius
                pointright = tuple(pointright)

                angle += 5
            
            if not self.isObstacle(pointleft[0], pointright[1]):
                return pointleft
            else:
                return pointright
    
    # random position generator
    def getRandomPosition(self):
        """
        Output:
        
        Returns the random node inside the current window (sensor range)
        """
        
        subgoal = self.getSubGoal(self.start, self.goal)

        # This is the range which any random node will be generated
        window = self.euc_heuristic(self.start, subgoal)

        r = window * math.sqrt(random.random())
        theta = random.random() * 0.5 * math.pi
        
        # randX = round(random.uniform((self.current_x - window), (self.current_x + window)), 2)
        # randY = round(random.uniform((self.current_y - window), (self.current_y + window)), 2)

        randX = self.start[0] + r * math.cos(theta)
        randY = self.start[1] + r * math.sin(theta)

        return (randX, randY)
    
    # nearest neighbour in the graph
    def getNearestNeighbour(self, x, y):
        """
        Inputs:
        
        x: x position of the random node we are getting the nearest node of
        y: y position of the random node we are getting the nearest node of
        
        Outputs:
        
        nearestVertex: the nearest node in the array of vertices of the graph
        """
        
        # set vertex to -1
        minDistance = float('inf')
        nearestVertex = -1
        
        # loop through vertices of graph
        for vertex in self.vertices:
            distance = self.euc_heuristic(vertex, (x, y))
            if(distance < minDistance):
                minDistance = distance
                nearestVertex = vertex
        
        # return nearest vertex
        return nearestVertex
    
    # new node
    def getNewNode(self, x_rand, x_nearest):
        """
        Inputs:
        
        x_rand: the random node
        x_nearest: the nearest node to the random node
        
        Outputs:
        
        newNode: the Xnew node at a distance of self.stepSize from x_nearest and in the direction of x_rand
        """
        
        # # slope of line joining x_rand and x_nearest
        # slope = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0])
        # factor = self.stepSize * np.sqrt(1.0 / (1.0 + (slope ** 2)))

        if self.euc_heuristic(x_nearest, x_rand) <= self.stepFactor:
            if self.checkObstacleBetweenPoints(x_nearest, x_rand):
                return (True, x_rand)
            else:
                return (False, x_rand)
            
        
        # find node at the step factor on the path from the nearest node to the random node
        vector = (x_rand[0] - x_nearest[0], x_rand[1] - x_nearest[1])
        norm = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        direction = (vector[0] / norm, vector[1] / norm) # unit vector
        # line that leads from nearest node to random node with a max distance of our step factor -> d*u
        nearest_to_rand = tuple(i * self.stepFactor for i in direction)
        # new point -> (x0,y0) + d*u
        new_node = tuple(map(lambda i, j: i + j, x_nearest, nearest_to_rand))

        if self.checkObstacleBetweenPoints(x_nearest, new_node):
            return (True, new_node)
        else:
            return (False, new_node)


    # # get neighbourhood
    # def getNeighbourhood(self, x_new):
    #     """
    #     Inputs:
        
    #     x_new: the new node
        
    #     Outputs:
        
    #     neighbourhood: the list of nodes in the neighbourhood of x_new
    #     """
        
    #     # iterate through the vertices and get nodes within a certain radius
    #     neighbourhood = []
    #     for index in range(0, len(self.vertices)):
    #         dist = self.euc_heuristic(x_new, self.vertices[index])
    #         if(dist < self.stepFactor):
    #             neighbourhood.append(self.vertices[index])
    #     return neighbourhood
    
    # # get neighbourhood parent
    # def getNeighbourhoodParent(self, neighbourhood):
    #     """
    #     Inputs:
        
    #     neighbourhood: the list of nodes in the neighbourhood of x_new
        
    #     Outputs:
        
    #     parent: the node that is the ideal parent for the x_new node
    #     """
        
    #     dist = self.costToCome[neighbourhood[0]]
    #     parent = neighbourhood[0]
    #     for index in range(1, len(neighbourhood)):
    #         curr_dist = self.costToCome[neighbourhood[index]]
    #         if(curr_dist < dist):
    #             dist = curr_dist
    #             parent = neighbourhood[index]
    #     return parent
    

    # rrt-star algo
    def search(self):
        """
        Outputs:
        
        exploredStates: the states explored when moving from start node to goal node.
        backtrackStates: the path from start node to goal node.
        actions: list containing the (dvx, dvy) values for each possible node between start and goal node.
        distance: the total distance between start node and goal node.
        """

        subgoal = self.getSubGoal(self.start, self.goal)
        self.goal_reached = False

        
        # initial steps for rrt-star algo
        self.costToCome[self.start] = 0
        self.vertices.append(self.start)
        backtrackNode = None
        
        
        # run the rrt-star algo
        while(not self.goal_reached):
            print(self.start)
            print("subgoal = ", subgoal)

            # get random node
            x_rand = self.getRandomPosition()
            
            # get nearest node
            x_nearest = self.getNearestNeighbour(x_rand[0], x_rand[1])
            
            
            # # check whether x_nearest[0] == x_rand[0] or x_nearest[1] == x_rand[1]
            # if((x_nearest[0] == x_rand[0]) or (x_nearest[1] == x_rand[1])):
            #     continue
    
            # get new node between x_nearest and x_rand
            (flag, x_new) = self.getNewNode(x_rand, x_nearest)
            if flag:
                continue
            
            # # get neighbourhood region for x_new
            # neighbourhood = self.getNeighbourhood(x_new)
            
            # # get parent for the neighbourhood region
            # parent = self.getNeighbourhoodParent(neighbourhood)
            # x_nearest = parent
            
            # check obstacle between x_nearest and x_new
            if self.checkObstacleBetweenPoints(x_nearest, x_new):
                continue
            
            # add x_new to graph
            self.vertices.append(x_new)
            self.path[x_new] = x_nearest
            self.costToCome[x_new] = self.costToCome[x_nearest] + self.euc_heuristic(x_nearest, x_new)

            print("no. of vertices = ", len(self.vertices))

            # # rewire graph
            # for index in range(0, len(neighbourhood)):
            #     distance_from_start = self.costToCome[x_new] + self.euc_heuristic(x_new, neighbourhood[index])
            #     if(distance_from_start < self.costToCome[neighbourhood[index]]):
            #         self.costToCome[neighbourhood[index]] = distance_from_start
            #         self.path[neighbourhood[index]] = x_new
            
            # check distance between goal and x_new
            dist_from_goal = self.euc_heuristic(x_new, subgoal)
            if(dist_from_goal <= self.goalThreshold):
                backtrackNode = x_new
                self.goal_reached = True
                print("goal reached")
        
                
        # backtrack path        
        backtrackStates = []
        while(backtrackNode != self.start):
            backtrackStates.append(backtrackNode)
            backtrackNode = self.path[backtrackNode]
        backtrackStates.append(self.start)
        backtrackStates = list(reversed(backtrackStates))


        self.follow_path(backtrackStates)

        # return (self.vertices, backtrackStates)

    
    # linear velocity of the car
    def linear_vel(self, goal):
        return self.forward_speed * self.euc_heuristic((self.current_x, self.current_y), goal)
    
    # steering angle for the car's motion
    def steering_angle(self, goal):
        return math.atan2(goal[1] - self.current_y, goal[0] - self.current_x)
    
    # angular velocity of the car
    def angular_vel(self, goal):
        return self.turning_speed * (self.steering_angle(goal) - self.current_yaw)
    
    # commands car to goal state by publishing twist message
    def move_to_goal(self, goal):
        '''
        Inputs:
        
        goal: the goal state to reach

        '''
        cmd = Twist()
        curr_location = (self.current_x, self.current_y)

        while self.euc_heuristic(goal, curr_location) > 0.1:

            # linear velocity in the x-axis
            cmd.linear.x = self.linear_vel(goal)
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0

            # angular velocity in the z-axis
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = self.angular_vel(goal)

            self.publisher_.publish(cmd)

    
    # Follows a given set of path - Reaches all the points in a list in consecutive order
    def follow_path(self, path):
        '''
        Inputs:

        path: the path to follow
        '''

        # print('following path -->', path)

        for loc in path:
            self.move_to_goal(loc)
            if self.euc_heuristic(loc, self.goal) <= self.goalThreshold:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.x = 0.0
                self.publisher_.publish(cmd)
                print("goal reached")
                break

    # end of RRT implementation, just to separate RRT code from everything else for better readability




    # def follow_wall(self):
    #     """
    #     This method causes the robot to follow the boundary of a wall.
    #     """
    #     # Create a geometry_msgs/Twist message
    #     msg = Twist()
    #     msg.linear.x = 0.0
    #     msg.linear.y = 0.0
    #     msg.linear.z = 0.0
    #     msg.angular.x = 0.0
    #     msg.angular.y = 0.0
    #     msg.angular.z = 0.0

    #     # Logic for following the wall
    #     # >d means no wall detected by that laser beam
    #     # <d means an wall was detected by that laser beam
    #     d = self.dist_thresh_wf

    #     if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
    #         self.wall_following_state = "search for wall"
    #         msg.linear.x = self.forward_speed
    #         msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

    #     elif (
    #         self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d
    #     ):
    #         self.wall_following_state = "turn left"
    #         msg.angular.z = self.turning_speed_wf_fast

    #     elif (
    #         self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d
    #     ):
    #         if self.rightfront_dist < self.dist_too_close_to_wall:
    #             # Getting too close to the wall
    #             self.wall_following_state = "turn left"
    #             msg.linear.x = self.forward_speed
    #             msg.angular.z = self.turning_speed_wf_fast
    #         else:
    #             # Go straight ahead
    #             self.wall_following_state = "follow wall"
    #             msg.linear.x = self.forward_speed

    #     elif (
    #         self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d
    #     ):
    #         self.wall_following_state = "search for wall"
    #         msg.linear.x = self.forward_speed
    #         msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

    #     elif (
    #         self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d
    #     ):
    #         self.wall_following_state = "turn left"
    #         msg.angular.z = self.turning_speed_wf_fast

    #     elif (
    #         self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d
    #     ):
    #         self.wall_following_state = "turn left"
    #         msg.angular.z = self.turning_speed_wf_fast

    #     elif (
    #         self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d
    #     ):
    #         self.wall_following_state = "turn left"
    #         msg.angular.z = self.turning_speed_wf_fast

    #     elif (return (self.vertices, backtrackStates)
    #         self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d
    #     ):
    #         self.wall_following_state = "search for wall"
    #         msg.linear.x = self.forward_speed
    #         msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

    #     else:
    #         pass

    #     # Send velocity command to the robot
    #     self.publisher_.publish(msg)


def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()

    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()