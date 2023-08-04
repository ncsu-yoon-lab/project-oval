import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray
import threading
import sys

sys.path.insert(0, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/simulation')
sys.path.insert(1, '/home/anglia/ros2_ws2/src/wolfwagen/wolfwagen/AStar/environment')
import runsim
import robot_orientation
import action

"""
ASTAR SENDS TURNING DIRECTION TO PWMGEN, LANEDETECTION THEN ATTEMPTS TO SUBSCRIBE TO PWMGEN TO GET NEXT TURNING DIRECTION WHEN AN INTERSECTION IS REACHED
IF NULL, CONTINUE AS NORMAL IN LANEDETECTION. HOW TO SEND JUST ONE PIECE OF INFORMMATION FROM ASTAR TO PWMGEN AT A TIME -> WE NEED COMMUNICATION THAT THE TURN IS COMPLETE
FOR COMUNICATION, HAVE LANEDETECTION SEND A SIGAL TO PWMMGEN TO ASTAR WHEN A TURN IS COMPLETE

IF ASTAR SENDS STOP, SET PWM_THROTTLE TO 0 IN PWMGEN TO STOP

"""

ITERATIONS = 25

# starting and target coordinates
TARGET_ROW = 1
TARGET_COL = 2

# map and costs file locations
map_file = './AStar/files/map01.txt'
costs = './AStar/files/costs.txt'
straight_line = './AStar/files/straight_line.txt'

sim = runsim.RunSim(map_file, ITERATIONS, costs, straight_line, TARGET_ROW, TARGET_COL)

astar_signal = 0
turn_val = None
act = None
move = None

list_moves = []

def traverse_solution():
    action = sim.get_next_action()
    movement = sim.env.actuate_env(action)
    return action, movement


def astar_signal_callback(data):
    global astar_signal
    astar_signal = data.data


def main(args=None):
    global turn_val, astar_signal, act, move, list_moves
    rclpy.init(args=args)
    node = Node("AStar")
    print("AStar Node")

    sub_astar_turnsig = node.create_subscription(Int64, 'astar_turn_sig', astar_signal_callback, 1)
    pub_astar_turnval = node.create_publisher(Int64MultiArray, 'astar_turn_val', 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    # For AStar
    # initialize the simulation and get path
    solution = sim.run(False)
    print(solution)

    act, move = traverse_solution()
    while act is not None:
        if act == action.Action.STOP:
            list_moves.append(-1)

        elif move == 'left':  # left
            list_moves.append(1)

        elif move == 'right':  # right
            list_moves.append(2)

        else:
            list_moves.append(0)
            
        act, move = traverse_solution()

    while rclpy.ok():
        if astar_signal != 0:
            print("Signal Received from PWM")
            print(list_moves)
            data = Int64MultiArray()
            data.data = list_moves
            pub_astar_turnval.publish(data)
            print("signal sent to PWM")
        astar_signal = 0
        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
