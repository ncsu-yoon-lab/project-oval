from matplotlib.lines import Line2D
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
import cv2 as cv
import numpy as np
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from collections import deque

# graph axis limit constants
# > the interval in milliseconds between graph updates
UPDATE_INTERVAL = 50 # 20 times per second
# > the number of previous time values to show on the graph
Y_TIME_MAX = UPDATE_INTERVAL # one second's worth of values
# > upper and lower bounds on error values
X_MIN = -100
X_MAX = 100

x_data = deque(np.zeros(Y_TIME_MAX), maxlen=Y_TIME_MAX)

def lane_callback(msg: Int64) -> None:
    x_data.appendleft(msg.data)

def update_animation(frame, line: Line2D) -> tuple:
    global x_data
    line.set_xdata(x_data)
    return line,

def main() -> None:
    rclpy.init()
    node = Node("pid_graph_node")

    node.create_subscription(
        Int64,
        "lane",
        lane_callback,
        10
    )

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    y = np.arange(0, Y_TIME_MAX)
    fig, ax = plt.subplots()
    fig, ax = plt.subplots()
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(0, Y_TIME_MAX)
    ax.invert_yaxis()
    line, = ax.plot(x_data, y)
    ani = anim.FuncAnimation(fig, update_animation, fargs=[line], interval=UPDATE_INTERVAL, blit=True)
    plt.show()

if __name__ == '__main__':
    main()
