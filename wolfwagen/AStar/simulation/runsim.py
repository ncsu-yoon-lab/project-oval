import sys

sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/wolfwagen/AStar/utils')
import maploader
import costsloader as cl

sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/wolfwagen/AStar/environment')
import environment
import action

"""
Simulation file to test A star with different files but is also an example of a module that we can import to run A Star
from by modifying a few things.
"""


class RunSim:
    # set the map file and iterations when constructing the RunSim object
    def __init__(self, map_file, iterations, costs, straight_line_costs, targetX, targetY):
        cost_vals = cl.CostLoader.load_map(costs)
        # load the map_file into a list of strings
        map_string = maploader.MapLoader.load_map(map_file)
        # initialize the environment with the track_map argument set
        self.env = environment.Environment(track_map=map_string, costs=cost_vals, straight_line=straight_line_costs,
                                           targetX=targetX, targetY=targetY)
        # set the number of iterations
        self.iterations = iterations

        # solution stack
        self.solution = None

    # run the simulation
    def run(self, test):

        # if the test boolean is True, we are in testing mode, otherwise run in normal mode
        if test:
            # loop through the iterations, we can just set this to true or false later.
            for i in range(self.iterations):
                self.solution = self.env.update_env()

                # if the target is reached, break out of loop
                if self.solution is not None:
                    break

        else:
            for i in range(self.iterations):
                try:
                    # update env will run robot.get_action() which will give us a path to follow
                    self.solution = self.env.update_env()

                # catch any exceptions
                except Exception as e:
                    print("error at time step " + str(i))
                    print(e)

                # if the target is reached, break out of loop
                if self.solution is not None:
                    break

        return self.solution

    def get_solution(self):
        return self.solution

    # this function gets the next action from the solution stack
    def get_next_action(self):
        if self.solution[-1] is not None:
            # get the Position on the top
            curr = self.solution.pop()
            # get all neighbors of the position
            neighbors = self.env.get_neighbor_positions(curr)
            # peek at the next value at the top to see which way to move
            curr = self.solution[-1]
            if curr is None:
                # if next action is None, we have reached the end, save the current orientation
                # and stop Action is returned
                # self.env.robot_orientation.save_orientation()
                return action.Action.STOP
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

            return robot_action

    # checks if the goal condition is met for outside functions, such as the unit tests or Lane Detection when we
    # incorporate
    def goal_condition_met(self):
        return self.env.goal_condition_met()
