import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pure_pursuit import PurePursuit  # your file with PathPoint2D + PurePursuit

# NOTE: This file is AI-generated. I asked chatgpt for a visual of the pure pursuit algorithm I wrote to demonstrate that it works.

class RobotPose2D:
    def __init__(self, x, y):
        self.point = np.array([float(x), float(y)])

    def get_point(self):
        return self.point

    def set_point(self, x, y):
        self.point = np.array([float(x), float(y)])


def interpolate_polyline(points, resolution=0.05):
    dense = []
    for i in range(len(points) - 1):
        p0 = np.array(points[i], dtype=float)
        p1 = np.array(points[i + 1], dtype=float)
        dist = float(np.linalg.norm(p1 - p0))
        steps = max(2, int(dist / resolution))
        for t in np.linspace(0.0, 1.0, steps):
            dense.append(tuple(p0 + t * (p1 - p0)))
    return dense


def main():
    # Rigid polyline (sharp-ish turns) but NOT perfect right angles
    base_path = [
        (0.0, 0.0),
        (2.2, 0.7),
        (4.5, -0.8),
        (6.8, 1.4),
        (8.7, 0.2),
        (10.0, 1.8),
        (11.6, 0.5),
        (13.0, 2.2),
    ]
    path = interpolate_polyline(base_path)

    pp = PurePursuit(lookahead_distance=0.7, velocity=0.8)
    pp.alpha = 0.0               # constant speed for clarity
    pp.lookahead_scaling = 0.3   # velocity-scaled lookahead
    pp.goal_tolerance = 0.10
    pp.init_path(path)

    robot = RobotPose2D(0, 0)  # start near the first segment
    yaw = 0.0
    vcurrent = 0.0
    dt = 0.05

    path_arr = np.array(path)

    fig, ax = plt.subplots()
    ax.plot(path_arr[:, 0], path_arr[:, 1])
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Pure Pursuit")
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    robot_plot, = ax.plot([], [], marker='o', linestyle='None')
    target_plot, = ax.plot([], [], marker='o', linestyle='None')
    traj_plot, = ax.plot([], [])
    circle_plot, = ax.plot([], [])
    heading_plot, = ax.plot([], [])

    trajx = []
    trajy = []

    pad = 1.0
    ax.set_xlim(path_arr[:, 0].min() - pad, path_arr[:, 0].max() + pad)
    ax.set_ylim(path_arr[:, 1].min() - pad, path_arr[:, 1].max() + pad)

    def init():
        robot_plot.set_data([], [])
        target_plot.set_data([], [])
        traj_plot.set_data([], [])
        circle_plot.set_data([], [])
        heading_plot.set_data([], [])
        return robot_plot, target_plot, traj_plot, circle_plot, heading_plot

    def update(frame):
        nonlocal yaw, vcurrent

        vcmd, wcmd = pp.update_state(robot, yaw, vcurrent)

        if abs(vcmd) < 1e-9 and abs(wcmd) < 1e-9:
            anim.event_source.stop()
            return robot_plot, target_plot, traj_plot, circle_plot, heading_plot

        yaw = yaw + wcmd * dt
        x = robot.get_point()[0] + vcmd * math.cos(yaw) * dt
        y = robot.get_point()[1] + vcmd * math.sin(yaw) * dt
        robot.set_point(x, y)
        vcurrent = vcmd

        tp = pp.compute_target_point(robot)
        if tp is None:
            tp = pp.path[pp.last_closest_index].get_point()

        robot_plot.set_data([robot.get_point()[0]], [robot.get_point()[1]])
        target_plot.set_data([tp[0]], [tp[1]])

        trajx.append(robot.get_point()[0])
        trajy.append(robot.get_point()[1])
        traj_plot.set_data(trajx, trajy)

        th = np.linspace(0.0, 2.0 * math.pi, 120)
        cx = robot.get_point()[0] + pp.lookahead_distance * np.cos(th)
        cy = robot.get_point()[1] + pp.lookahead_distance * np.sin(th)
        circle_plot.set_data(cx, cy)

        hx = robot.get_point()[0] + 0.5 * math.cos(yaw)
        hy = robot.get_point()[1] + 0.5 * math.sin(yaw)
        heading_plot.set_data([robot.get_point()[0], hx], [robot.get_point()[1], hy])

        return robot_plot, target_plot, traj_plot, circle_plot, heading_plot

    anim = FuncAnimation(fig, update, init_func=init, interval=int(dt * 1000), blit=True)
    plt.show()


if __name__ == "__main__":
    main()