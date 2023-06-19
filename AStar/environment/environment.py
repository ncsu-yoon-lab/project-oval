import position as pos
import positiontype
import positiontypestatus as pts
import sys
import action

# This adds a dir path to the current runtime to import modules in other folders
# you will need to change this in all files for the robot

sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/AStar/robotAgent')
import robot

sys.path.insert(1, '/home/sarvesh/Documents/GitHub/wolfwagen/AStar/utils')
import costsloader as cl

"""
This is the environment containing positional information about the track and the robot's whereabouts on the track
Can be instantiated using a string map to produce many different environments for testing

allows the robot agent to explore the track/environemnt

@author: Sarvesh
"""


# the class keyword allows us to treat the Environment class as an object
class Environment:

    # Initialize the class, like a constructor for java
    # The '*' in the args represents that the following arguments need to be specified by name but are optional
    def __init__(self, costs, straight_line, track_map, startX, startY, targetX, targetY):

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
        self.starting_pos = self.set_starting_pos(startX, startY)
        self.target_pos = self.set_target_pos(targetX, targetY)

        # next, we fill in the positions list with a list of positions created from the rows and cols
        # the roads map is also filled with positions and road values and an intersections list is created for use in
        # building costs files
        for i in range(rows):
            list_pos = []
            for j in range(cols):
                tile_val = track_map[i][j]
                p = pos.Position(i, j)
                list_pos.append(p)
                if tile_val == "I":
                    self.roads[p] = positiontype.PositionType(pts.PositionTypeStatus.INTERSECTION)
                    self.num_intersections += 1
                    self.list_intersections.append(p)
                if tile_val ==  "C":
                    self.roads[p] = positiontype.PositionType(pts.PositionTypeStatus.CURVE)

            self.positions.append(list_pos)

        # this nested for loop relates all the positions to each other by setting the above, below, left, and right
        # positions
        for row in range(rows):
            for col in range(cols):
                if row > 0:
                    self.positions[row][col].set_above(self.positions[row - 1][col])
                if row < self.rows - 1:
                    self.positions[row][col].set_below(self.positions[row + 1][col])
                if col > 0:
                    self.positions[row][col].set_left(self.positions[row][col - 1])
                if col < self.cols - 1:
                    self.positions[row][col].set_right(self.positions[row][col + 1])

        # build a costs map
        # create a cost_map that files two positions as tuples to a value that is cost {(pos1, pos2), cost_val}
        self.cost_map = dict()
        self.create_cost_map(costs)
        self.straight_line_costs = cl.CostLoader.load_straight_line(straight_line, self.list_intersections)

        # the robot agent is created using the environment object
        # The robot starting position is initialized and set
        self.robot_agent = robot.Robot(self, self.cost_map, self.straight_line_costs)
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

                    # if the road status is a curve, then join the roads that are to the left and below
                    # we will add logic in A star to say if a curve is encountered use cost values from
                    # the related intersections and treat the curve as a straight line between the two nodes

                    if curr_pos_status is pts.PositionTypeStatus.CURVE:
                        pos_1 = neighbors.get("right")
                        pos_2 = neighbors.get("below")
                        self.cost_map[(pos_1, pos_2)] = costs[cost_iterator]
                        cost_iterator += 1
                    else:
                        # if we are mapping top to bottom, use above values to map
                        if switch_dir and neighbors.get("above") is not None and \
                                self.get_road_status(neighbors.get("above")) is not pts.PositionTypeStatus.CURVE:
                            self.cost_map[(curr_pos, neighbors.get("above"))] = costs[cost_iterator]
                            cost_iterator += 1
                        # if we are mapping left to right, use right values to map
                        elif not switch_dir and neighbors.get("right") is not None:
                            self.cost_map[(curr_pos, neighbors.get("right"))] = costs[cost_iterator]
                            cost_iterator += 1

            # check if there is more costs to add as well as check if we need to change directions and restart
            # the row
            if cost_iterator < len(costs):
                # switch directions if needed and restart row
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
        robot_action = action.Action.STOP
        if self.robot_agent.get_solved() and solution_stack is not action.Action.STOP:
            # get the Position on the top
            curr = solution_stack.pop()
            # get all neighbors of the position
            neighbors = self.get_neighbor_positions(curr)
            # peek at the next value at the top to see which way to move
            curr = solution_stack[-1]
            # logic to return the action to get to the next Position
            if curr.__eq__(neighbors.get("above")):
                robot_action = action.Action.UP
            elif curr.__eq__(neighbors.get("below")):
                robot_action = action.Action.DOWN
            elif curr.__eq__(neighbors.get("left")):
                robot_action = action.Action.LEFT
            elif curr.__eq__(neighbors.get("right")):
                robot_action = action.Action.RIGHT
            else:
                robot_action = action.Action.TURN

        robot_pos = self.get_robot_pos()
        row = robot_pos.get_row()
        col = robot_pos.get_col()

        print(robot_action)
        if robot_action is action.Action.LEFT:
            if self.valid_pos(row, col - 1):
                self.update_robot_pos(row, col - 1)
        elif robot_action is action.Action.RIGHT:
            if self.valid_pos(row, col + 1):
                self.update_robot_pos(row, col + 1)
        elif robot_action is action.Action.DOWN:
            if self.valid_pos(row + 1, col):
                self.update_robot_pos(row + 1, col)
        elif robot_action is action.Action.UP:
            if self.valid_pos(row - 1, col):
                self.update_robot_pos(row - 1, col)
        elif robot_action is action.Action.STOP:
            self.update_robot_pos(row, col)
        elif robot_action is action.Action.TURN:
            if self.valid_pos(row - 1, col + 1):
                self.update_robot_pos(row - 1, col + 1)
            else:
                self.update_robot_pos(row + 1, col - 1)

    # this gets the target position
    def get_target_pos(self):
        return self.target_pos

    # this checks if the goal has been met by the robot
    def goal_condition_met(self):
        robot_position = self.get_robot_pos()
        return robot_position == self.target_pos
