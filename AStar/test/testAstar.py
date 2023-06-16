import sys
sys.path.insert(0, '/home/sarvesh/Documents/GitHub/wolfwagen/AStar/simulation')
import runsim

num_trials = 100
iterations = 200
successful_trials = 0
startX = 0
startY = 2
targetX = 1
targetY = 0

map_file = '../maps/map01.txt'
costs = '../maps/costs.txt'
straight_line = '../maps/straight_line.txt'
for trial in range(num_trials):
    sim = runsim.RunSim(map_file, iterations, costs, straight_line, startX, startY, targetX, targetY)
    sim.run()
    print("----------- trial " + str(trial) + " done ---------------")
    if sim.goal_condition_met():
        successful_trials += 1

print("test 1 success rate " + str(successful_trials/(num_trials*1.0)*100) + " after 100 trials")
