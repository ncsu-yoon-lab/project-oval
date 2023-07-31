import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
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
TARGET_ROW = 0
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

def traverse_solution():
    action = sim.get_next_action()
    movement = sim.env.actuate_env(action)
    return action, movement


def astar_signal_callback(data):
    global astar_signal
    astar_signal = data.data


def main(args=None):
    global turn_val, astar_signal, act, move
    rclpy.init(args=args)
    node = Node("AStar Node")
    print("AStar Node")

    sub_astar_turnsig = node.create_subscription(Int64, 'astar_turn_sig', astar_signal_callback, 1)
    pub_astar_turnval = node.create_publisher(Int64, 'astar_turn_val', 1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    # For AStar
    # initialize the simulation and get path
    solution = sim.run(False)

    while rclpy.ok():
        if astar_signal != 0:
            act, move = traverse_solution()
            if act.equals(action.Action.STOP):
                pub_astar_turnval.publish(-1)

            elif move == 'left':  # left
                turn_val = 1

            elif move == 'right':  # right
                turn_val = 2

            else:
                turn_val = 0
            astar_signal = 0
            pub_astar_turnval.publish(turn_val)
        rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
