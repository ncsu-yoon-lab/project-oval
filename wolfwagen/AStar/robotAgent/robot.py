import sys

# add a path to the system to add modules from the environment directory
sys.path.insert(0, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/environment')
import position
import action
import positiontypestatus as pts
from collections import deque
from queue import PriorityQueue

"""
This represents the robot agent moving through an environment/track
Basic implementation, uses A star implementation with Manhattan Distance Heuristic (need to change this in future) and
distances between intersection in meters for cost values between nodes

@author: Sarvesh
"""


class Robot:
    """
    This is an internal Node class. This is used to wrap a Position object in a comparable Node class
    This class also allows us to calculate the cost to travel between two Position
    """

    class Node:
        # initialize Node obj with Position data that is passed in the args the priority is initially set to 0
        def __init__(self, data):
            self.data = data
            self.priority = 0

        # set the priority value
        def set_priority(self, p):
            self.priority = p

        # gets the position value
        def get_position(self):
            return self.data

        # calculate the cost between current and next position
        def cost(self, next_pos, costs):
            pos1 = self.data
            pos2 = next_pos.get_position()
            cost_val = costs.get((pos1, pos2)) if costs.get((pos1, pos2)) is not None else costs.get((pos2, pos1))
            if cost_val is None:
                mod_pos2 = position.Position(next_pos.get_position().get_row() + 1,
                                             next_pos.get_position().get_col()) if self.data.get_row() != next_pos.get_position().get_row() + 1 else position.Position(
                    next_pos.get_position().get_row(), next_pos.get_position().get_col() + 1)
                cost_val = costs.get((pos1, mod_pos2)) if costs.get((pos1, mod_pos2)) is not None else costs.get(
                    (mod_pos2, pos1))
            return cost_val

            # calculate cost here, need measurements

        # calculate distance to target
        def cost_to_target(self, robot):
            env = robot.env
            curr_pos = self.data
            target_pos = robot.target_node.get_position()
            cost_val = env.straight_line_costs.get((curr_pos, target_pos))
            if cost_val is None:
                cost_val = env.straight_line_costs.get((target_pos, curr_pos))
            return cost_val

        # this compares two Node objects by position data.
        def compare_to(self, o):
            if self.data.__eq__(o.get_position()):
                return 0
            return -1

        # implements the equals function for two nodes, used for ordering
        def __eq__(self, n):
            if isinstance(n, Robot.Node):
                return self.priority == n.priority
            return False

        # implements the less than function for two nodes, used for ordering in the PQ
        def __lt__(self, other):
            if isinstance(other, Robot.Node):
                return self.priority < other.priority
            return False

    # construct the robot obj
    def __init__(self, env, cost_map, straight_line):
        # the environment obj
        self.env = env
        # the costs map
        self.cost_map = cost_map
        # straight line costs
        self.straight_line_map = straight_line
        # frontier is a PriorityQueue of Nodes
        self.frontier = None
        # temporary start and target nodes, will be set to the starting pos of robot and target pos from env
        self.start_node = self.Node(position.Position(0, 0))
        self.target_node = self.Node(position.Position(0, 0))
        # the came from dict is a map of Position -> Position
        # map means key:Position came from value:Position
        self.came_from = dict()
        # the cost so far dict is a map of Position -> float
        # cost amount for each Node in the frontier
        self.cost_so_far = dict()
        # if solved is True, we have reached a solution, changing this to False activates A star
        self.solved = False
        # this is a stack of solution movements, we use the deque data structure
        self.solution = deque()

    # this is the get action function, it runs the A star algorithm to find the best path from point A to point B
    # it uses a custom heuristic function to determine costs.txt of travelling between two different Nodes.
    # cost is determined by f = g + h, where f is the total cost of moving from Node_i to Node_n, g is the current
    # total cost at Node_i and h is the cost of moving from Node_i to Node_n. Then a priority value is assigned and the
    # Node is added to the frontier PQ to be explored in order of most promising to least promising.
    def get_action(self):

        # set the current position and target position
        self_pos = self.env.get_robot_pos()
        target_pos = self.env.get_target_pos()

        # run A star while a solution is not reached
        if not self.solved:

            # initialize the PQ, set the start node and target node
            self.frontier = PriorityQueue()
            self.start_node = self.Node(self_pos)
            self.target_node = self.Node(target_pos)

            # add the first Node with priority 0
            self.frontier.put(self.start_node)

            # initialize the maps for came from and cost so far
            self.came_from = dict()
            self.cost_so_far = dict()

            # add in the first values to the maps
            self.came_from[self.start_node.get_position()] = None
            self.cost_so_far[self.start_node.get_position()] = 0

            # continue exploring while there are still Nodes to explore in the PQ
            while not self.frontier.empty():
                # get the node with the highest priority
                current_node = self.frontier.get()

                # if the curr node is at the target node, then the solution is complete
                if current_node.compare_to(self.target_node) == 0:
                    self.solved = True

                    # this small algorithm builds the solution stack bottom up from the came from map
                    # start at the target position, and add that position to a new solution Stack
                    # then get the value associated with the target position key in the came from map
                    # this position value will be the Position that leads to the position in the key.
                    # add that new position value to the solution stack, then update the iterator value with the next
                    # position value that leads from the previous position value, continue doing this until the start
                    # position is reached then break. Now we have a stack of positions starting from the start position
                    # leading to the target position

                    iterator = self.target_node.get_position()
                    self.solution = deque()
                    self.solution.append(self.target_node.get_position())

                    while self.came_from.get(iterator) is not None:
                        self.solution.append(self.came_from.get(iterator))
                        iterator = self.came_from.get(iterator)
                    break

                # get the neighbor positions from the current positions to explore outward from the current position
                neighbors = self.env.get_neighbor_positions(current_node.get_position())
                # for all the values in the neighbors map loop through to determine which of the positions are the best
                # choice for finding the target position, this is where we use the cost values
                for value in neighbors.values():

                    if self.env.get_road_status(value) is pts.PositionTypeStatus.CURVE:
                        if list(neighbors.keys())[list(neighbors.values()).index(value)] == 'left':
                            value = value.get_below()
                        else:
                            value = value.get_right()
                    # find the next node and get the new cost
                    next_node = self.Node(value)

                    # if the next node is a valid position, continue
                    if self.env.valid_pos(next_node.get_position().get_row(), next_node.get_position().get_col()):
                        # new cost calculated by getting cost so far and adding the cost to go from curr Node to next
                        # Node
                        new_cost = self.cost_so_far.get(current_node.get_position()) + current_node.cost(next_node,
                                                                                                         self.cost_map)

                        # if the next node is not in the cost so far map or the new cost is less than the current
                        # cost of that node, continue
                        if (self.cost_so_far.get(next_node.get_position()) is None) or (
                                new_cost < self.cost_so_far.get(next_node.get_position())):
                            # update the next node cost to the cost so far map
                            self.cost_so_far[next_node.get_position()] = new_cost
                            # print(new_cost)
                            # set the priority value by adding the new cost with the cost to reach the goal
                            priority = float(new_cost) + next_node.cost_to_target(self)

                            # set the next node's priority then use that priority to add the node into the PQ
                            next_node.set_priority(priority)
                            self.frontier.put(next_node)

                            # set the came from key value pair with the next and curr Nodes
                            self.came_from[next_node.get_position()] = current_node.get_position()

            # return action Stop while still calculating A Star
            return action.Action.STOP

        # if solved, we can use the solution stack we built to return Action instructions
        else:
            """
            we can add logic here, if there was to be an obstacle detected or placed, then set solved to False
            clear the solution deque, update the map to add the obstacle as an EDGE road type, and set the current
            position as the start position and find a new routs to the target position using A star.
            """

            return self.solution

            # # get the Position on the top
            # curr = self.solution.pop()
            # # get all neighbors of the position
            # neighbors = self.env.get_neighbor_positions(curr)
            # # peek at the next value at the top to see which way to move
            # curr = self.solution[-1]
            #
            # # logic to return the action to get to the next Position
            # if curr.__eq__(neighbors.get("above")):
            #     return action.Action.UP
            # if curr.__eq__(neighbors.get("below")):
            #     return action.Action.DOWN
            # if curr.__eq__(neighbors.get("left")):
            #     return action.Action.LEFT
            # if curr.__eq__(neighbors.get("right")):
            #     return action.Action.RIGHT
            # return action.Action.TURN

    def get_solved(self):
        return self.solved
