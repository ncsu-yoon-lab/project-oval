import sys
sys.path.insert(0, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/utils')
import maploader
import costsloader as cl
sys.path.insert(0, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/environment')
import environment

"""
Simulation file to test A star with different files but is also an example of a module that we can import to run A Star
from by modifying a few things.
"""


class RunSim:
    # set the map file and iterations when constructing the RunSim object
    def __init__(self, map_file, iterations, costs, straight_line_costs, startX, startY, targetX, targetY):
        cost_vals = cl.CostLoader.load_map(costs)
        # load the map_file into a list of strings
        map_string = maploader.MapLoader.load_map(map_file)
        # initialize the environment with the track_map argument set
        self.env = environment.Environment(track_map=map_string, costs=cost_vals, straight_line=straight_line_costs,
                                           startX=startX, startY=startY, targetX=targetX, targetY=targetY)
        # set the number of iterations
        self.iterations = iterations

    # run the simulation
    def run(self):
        # loop through the iterations, we can just set this to true or false later.
        for i in range(self.iterations):
            # self.env.update_env()
            try:
                # update env will run robot.get_action() which will give us a path to follow
                self.env.update_env()

            # catch any exceptions
            except Exception as e:
                print("error at time step " + str(i))
                print(e)

            # if the target is reached, break out of loop
            if self.env.goal_condition_met():
                break

    # checks if the goal condition is met for outside functions, such as the unit tests or Lane Detection when we
    # incorporate
    def goal_condition_met(self):
        return self.env.goal_condition_met()
