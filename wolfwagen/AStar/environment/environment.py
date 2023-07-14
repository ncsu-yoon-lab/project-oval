import position as pos
import positiontype
import positiontypestatus as pts
import sys
import action
import robot_orientation

# This adds a dir path to the current runtime to import modules in other folders
# you will need to change this across all AStar files if working on a different machine
sys.path.insert(0, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/robotAgent')
import robot

sys.path.insert(1, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/utils')
import costsloader as cl

"""
This is the environment containing positional information about the track and the robot's whereabouts on the track
Can be instantiated using a string map to produce many different environments for testing

allows the robot agent to explore the track/environment

@author: Sarvesh
"""


class Environment:

    # Initialize the class, like a constructor
    # The '*' in the args represents that the following arguments need to be specified by name but are optional
    # ( no longer used )
    def __init__(self, costs, straight_line, track_map, targetX, targetY):
        # we lead in the current robot_orientation as an object so that we can track changes and save the
        # new orientation
        self.robot_orientation = robot_orientation.RobotOrientation()

        # the rows and cols are set to the respective values from the map
        rows = len(track_map)
        cols = len(track_map[0])

        # we set the global variables for the current object
        self.rows = rows
        self.cols = cols
        self.robot_position = None
        self.num_intersections = 0

        # this is a list of positions of the intersections and curves of the track
        self.positions = []
        # we use dict() here like a hashmap, this means we also have to create hash functions for the objects being used
        # as keys. The roads dict serves as a map of positions to roads
        self.roads = dict()
        # list of all positions that are intersections
        self.list_intersections = []

        # these functions allow us to set starting and target positions using coordinates provided in the constructor
        # args
        # we set the starting position coords by using the robot_orientation to get the row and column values
        self.starting_pos = self.set_starting_pos(int(self.robot_orientation.get_orientation()[1]),
                                                  int(self.robot_orientation.get_orientation()[2]))
        self.target_pos = self.set_target_pos(targetX, targetY)

        # next, we fill in the positions list with a list of position objects created from the rows and cols
        # the roads map is also filled with positions and road values and an intersections list is created for use in
        # building costs files

        # this for loop is reversed so that we can go bottom up building the world frame x and y axes rather than
        # setting (0,0) to be the top left, this algorithm sets origin to bottom left
        y_val = 0
        for i in reversed(range(rows)):
            x_val = 0
            list_pos = []
            for j in range(cols):
                tile_val = track_map[i][j]
                p = pos.Position(y_val, x_val)
                list_pos.append(p)
                if tile_val == "I":
                    self.roads[p] = positiontype.PositionType(pts.PositionTypeStatus.INTERSECTION)
                    self.num_intersections += 1
                    self.list_intersections.append(p)
                if tile_val == "C":
                    self.roads[p] = positiontype.PositionType(pts.PositionTypeStatus.CURVE)
                x_val += 1
            self.positions.append(list_pos)
            y_val += 1

        # this nested for loop relates all the positions to each other by setting the above, below, left, and right
        # positions
        for row in range(rows):
            for col in range(cols):
                if row > 0:
                    self.positions[row][col].set_below(self.positions[row - 1][col])
                if row < self.rows - 1:
                    self.positions[row][col].set_above(self.positions[row + 1][col])
                if col > 0:
                    self.positions[row][col].set_left(self.positions[row][col - 1])
                if col < self.cols - 1:
                    self.positions[row][col].set_right(self.positions[row][col + 1])

        # build a costs map
        # create a cost_map that maps two positions as tuples to a value that is
        # the cost {(from_position, to_position), cost_val}
        self.cost_map = dict()
        self.create_cost_map(costs)
        # straight line costs are also mapped using the same form dict() as the cost_map
        self.straight_line_costs = cl.CostLoader.load_straight_line(straight_line, self.list_intersections)
        # the robot agent is created using the environment object
        # The robot starting position is initialized and set
        self.robot_agent = robot.Robot(self, self.cost_map, self.straight_line_costs, self.robot_orientation)
        self.add_robot(self.get_starting_pos())

    """
    These are the traditional getters and setters we need to have for the environment object
    """

    # this gets the current road status of a position
    def get_road_status(self, p):
        return self.roads.get(p).get_status()

    # this gets the current position of the robot on the track
    def get_robot_pos(self):
        return self.robot_position

    # this gets the road at a position
    def get_position_road(self, p):
        return self.roads.get(p)

    # this sets the robot position to a given position
    def add_robot(self, p):
        self.robot_position = p

    # this gets the neighbor positions of a given position as a dict
    @staticmethod
    def get_neighbor_positions(p):
        neighbors = dict()
        if p.get_above() is not None:
            neighbors["above"] = p.get_above()
        if p.get_below() is not None:
            neighbors["below"] = p.get_below()
        if p.get_right() is not None:
            neighbors["right"] = p.get_right()
        if p.get_left() is not None:
            neighbors["left"] = p.get_left()

        return neighbors

    # this checks of a specific coordinate is a valid position
    def valid_pos(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    # This gets the starting position of the map
    def get_starting_pos(self):
        return self.starting_pos

    # this sets the starting position of the map
    @staticmethod
    def set_starting_pos(x, y):
        return pos.Position(x, y)

    # sets the target position of the map
    @staticmethod
    def set_target_pos(x, y):
        return pos.Position(x, y)

    # This updates the robot's position so that we can check later if it reached the target or not
    def update_robot_pos(self, row, col):
        p = self.positions[row][col]
        self.robot_position = p

    # creates the cost map
    def create_cost_map(self, costs):
        # we have a boolean to determine whether we are connecting from top to bottom or left to right
        switch_dir = False
        # iterator to go through the costs array
        cost_iterator = 0

        # iterator to go through rows, we need to use while loop so that we can restart rows that have top to bottom
        # connections as well as left to right connections
        i = 0
        while i < self.rows:
            for j in range(self.cols):
                # check if the road is a valid road, not an EDGE
                if self.get_position_road(self.positions[i][j]).is_valid():
                    # get current position, current status, and valid neighbors (not edges)
                    curr_pos = self.positions[i][j]
                    curr_pos_status = self.get_road_status(curr_pos)
                    neighbors = self.get_neighbor_positions(curr_pos)
                    # if the road status is a curve, then join the roads that are to the right and below
                    # we will add logic in A star to say if a curve is encountered use cost values from
                    # the related intersections and treat the curve as a straight line between the two nodes

                    if curr_pos_status is pts.PositionTypeStatus.CURVE:
                        pos_1 = neighbors.get("right")
                        pos_2 = neighbors.get("below")
                        # check to see if we have not already encountered a curve before
                        if self.cost_map.get((pos_1, pos_2)) is None:
                            self.cost_map[(pos_1, pos_2)] = costs[cost_iterator]
                            cost_iterator += 1
                    else:
                        # we are mapping bottom to top, use below values to map
                        if switch_dir and neighbors.get("below") is not None and \
                                self.get_road_status(neighbors.get("below")) is not pts.PositionTypeStatus.CURVE:
                            self.cost_map[(curr_pos, neighbors.get("below"))] = costs[cost_iterator]
                            cost_iterator += 1
                        # if we are mapping left to right, use right values to map
                        elif not switch_dir and neighbors.get("right") is not None:
                            self.cost_map[(curr_pos, neighbors.get("right"))] = costs[cost_iterator]
                            cost_iterator += 1
            # check if there is more costs to add as well as check if we need to change directions and restart
            # the row
            if cost_iterator < len(costs):
                # switch directions from left to right to bottom to top if needed and restart row
                if switch_dir:
                    switch_dir = False
                    i = i - 1
                else:
                    switch_dir = True
            i += 1

    # this updates the environment with the robot's next action, current position, and next position
    # need to update this method to send commands to the lane following code, per action.
    def update_env(self):
        solution_stack = self.robot_agent.get_action()

        # check to see if the agent has solved the problem and that the solution is not a STOP action
        # otherwise just update the robot's position with the current row and column values
        if self.robot_agent.get_solved() and solution_stack is not action.Action.STOP:
            return solution_stack

        else:
            self.update_robot_pos(self.get_robot_pos().get_row(), self.get_robot_pos().get_col())
            return None

    # this gets the target position
    def get_target_pos(self):
        return self.target_pos

    # this checks if the goal has been met by the robot
    def goal_condition_met(self):
        robot_position = self.get_robot_pos()
        return robot_position == self.target_pos

    # this function is used to actuate the robot when given Action items
    def actuate_env(self, robot_action):
        # we get the current position, row, and column
        robot_pos = self.get_robot_pos()
        row = robot_pos.get_row()
        col = robot_pos.get_col()

        # get the movement action from the robot_orientation object, this is used to tell Lane Detection when to turn
        movement_action = self.robot_orientation.movement_interpreter(robot_action)

        # actuate the Action by using the update_robot_pos function with new coords
        print(robot_action)
        if robot_action is action.Action.LEFT:
            if self.valid_pos(row, col - 1):
                self.update_robot_pos(row, col - 1)
        elif robot_action is action.Action.RIGHT:
            if self.valid_pos(row, col + 1):
                self.update_robot_pos(row, col + 1)
        elif robot_action is action.Action.DOWN:
            if self.valid_pos(row - 1, col):
                self.update_robot_pos(row - 1, col)
        elif robot_action is action.Action.UP:
            if self.valid_pos(row + 1, col):
                self.update_robot_pos(row + 1, col)
        elif robot_action is action.Action.STOP:
            self.update_robot_pos(row, col)
        elif robot_action is action.Action.TURN:
            if self.valid_pos(row + 1, col + 1):
                self.update_robot_pos(row + 1, col + 1)
            else:
                self.update_robot_pos(row - 1, col - 1)

        # return the movement_action to be used in LaneDetection
        return movement_action

    # this function gets the next hypothetical action, this is used in the astar algorithm in combination with the
    # robot's orientation to disqualify certain movements from being added to the solution
    def get_action(self, curr, next_pos):

        neighbors = self.get_neighbor_positions(curr)
        if next_pos.__eq__(neighbors.get("above")):
            robot_action = action.Action.UP
        elif next_pos.__eq__(neighbors.get("below")):
            robot_action = action.Action.DOWN
        elif next_pos.__eq__(neighbors.get("left")):
            robot_action = action.Action.LEFT
        elif next_pos.__eq__(neighbors.get("right")):
            robot_action = action.Action.RIGHT
        else:
            robot_action = action.Action.TURN

        return robot_action
