import sys
sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/wolfwagen/AStar/simulation')
import runsim
from pathlib import Path


num_trials = 1
iterations = 20
successful_trials = 0
targetX = 0
targetY = 0

map_file = '../files/map01.txt'
costs = '../files/costs.txt'
straight_line = '../files/straight_line.txt'
for trial in range(num_trials):
    sim = runsim.RunSim(map_file, iterations, costs, straight_line, targetX, targetY)
    solution = sim.run(False)
    action = 0
    print(solution)
    while action is not None:
        action = sim.get_next_action()
        # print(action)
        sim.env.actuate_env(action)
    print("----------- trial " + str(trial) + " done ---------------")
    if sim.goal_condition_met():
        successful_trials += 1

print("test 1 success rate " + str(successful_trials/(num_trials*1.0)*100) + " after 100 trials")
