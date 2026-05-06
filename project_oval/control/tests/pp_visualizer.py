#!/usr/bin/env python3
"""
pp_visualizer.py

Real-time 2D matplotlib visualizer for the pure_pursuit_node.

Subscribes to:
  /gpsfix                       (gps_msgs/GPSFix)           — robot position + current heading
  /pure_pursuit/path            (nav_msgs/Path)              — planned waypoint path
  /pure_pursuit/omega           (std_msgs/Float64MultiArray) — [left_rpm, right_rpm]
  /pure_pursuit/target_point    (std_msgs/Float64MultiArray) — [x, y] lookahead target in meters
  /pure_pursuit/lookahead_distance (std_msgs/Float64)        — current lookahead radius

Requires these additions to pure_pursuit_node.py:

  init_publishers():
      self.target_point_pub   = self.create_publisher(Float64MultiArray, "/pure_pursuit/target_point", 10)
      self.lookahead_dist_pub = self.create_publisher(Float64, "/pure_pursuit/lookahead_distance", 10)

  control_loop(), after update_state():
      target = self.pure_pursuit.last_target_point
      if target is not None:
          tp_msg = Float64MultiArray()
          tp_msg.data = [float(target[0]), float(target[1])]
          self.target_point_pub.publish(tp_msg)
      self.lookahead_dist_pub.publish(Float64(data=self.pure_pursuit.lookahead_distance))

Usage:
    python3 pp_visualizer.py

Dependencies:
    pip install matplotlib
    ros-humble-gps-msgs
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from gps_msgs.msg import GPSFix
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Float64

import matplotlib
matplotlib.use("TkAgg")   # change to "Qt5Agg" if TkAgg is unavailable
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Circle

# ─────────────────────────────────────────────
#  Match these to pure_pursuit_node.py
# ─────────────────────────────────────────────
ORIGIN_LAT = 35.770730
ORIGIN_LON = -78.674728
LAT_TO_METERS = 111320.0
# ─────────────────────────────────────────────

ARROW_LEN  = 10.0    # meters — heading arrow length
UPDATE_HZ  = 10     # matplotlib redraw rate

COLOR_CURRENT   = "#ffdd44"   # yellow  — current GPS heading
COLOR_TARGET    = "#ff8800"   # orange  — lookahead target point
COLOR_LOOKAHEAD = "#336699"   # blue    — lookahead circle


def latlon_to_meters(lat, lon):
    lon_to_meters = LAT_TO_METERS * math.cos(math.radians(ORIGIN_LAT))
    x = (lon - ORIGIN_LON) * lon_to_meters
    y = (lat - ORIGIN_LAT) * LAT_TO_METERS
    return x, y


class Visualizer(Node):
    def __init__(self):
        super().__init__("pp_visualizer")

        self._lock = threading.Lock()

        self.robot_x        = None
        self.robot_y        = None
        self.robot_yaw      = None
        self.robot_speed    = None

        self.target_x       = None   # lookahead target point
        self.target_y       = None
        self.lookahead_dist = None   # current lookahead radius

        self.planned_xs = []
        self.planned_ys = []
        self.actual_xs  = []
        self.actual_ys  = []

        self.left_rpm   = None
        self.right_rpm  = None

        gps_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(GPSFix,            "/gpsfix",                        self._gps_cb,       gps_qos)
        self.create_subscription(Path,              "/pure_pursuit/path",              self._path_cb,      10)
        self.create_subscription(Float64MultiArray, "/pure_pursuit/omega",             self._omega_cb,     10)
        self.create_subscription(Float64MultiArray, "/pure_pursuit/target_point",      self._target_cb,    10)
        self.create_subscription(Float64,           "/pure_pursuit/lookahead_distance",self._lookahead_cb, 10)

        self.get_logger().info("Visualizer ready — waiting for data…")

    def _gps_cb(self, msg: GPSFix):
        x, y = latlon_to_meters(msg.latitude, msg.longitude)
        yaw = math.radians(90.0 - msg.track)
        with self._lock:
            self.robot_x     = x
            self.robot_y     = y
            self.robot_yaw   = yaw
            self.robot_speed = msg.speed
            self.actual_xs.append(x)
            self.actual_ys.append(y)

    def _path_cb(self, msg: Path):
        xs, ys = [], []
        for ps in msg.poses:
            mx, my = latlon_to_meters(ps.pose.position.x, ps.pose.position.y)
            xs.append(mx)
            ys.append(my)
        with self._lock:
            self.planned_xs = xs
            self.planned_ys = ys
        self.get_logger().info(f"Path received: {len(xs)} waypoints")

    def _omega_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            with self._lock:
                self.left_rpm  = msg.data[0]
                self.right_rpm = msg.data[1]

    def _target_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            with self._lock:
                self.target_x = msg.data[0]
                self.target_y = msg.data[1]

    def _lookahead_cb(self, msg: Float64):
        with self._lock:
            self.lookahead_dist = msg.data


class LivePlot:
    def __init__(self, node: Visualizer):
        self.node = node

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title("Pure Pursuit — Live Visualizer")
        self.fig.patch.set_facecolor("#1e1e2e")
        self.ax.set_facecolor("#1e1e2e")
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#444466")
        self.ax.tick_params(colors="#aaaacc")
        self.ax.xaxis.label.set_color("#aaaacc")
        self.ax.yaxis.label.set_color("#aaaacc")
        self.ax.title.set_color("#ccccff")
        self.ax.set_xlabel("X — East (m)")
        self.ax.set_ylabel("Y — North (m)")
        self.ax.set_title("Pure Pursuit Live View")
        self.ax.set_aspect("equal")
        self.ax.grid(True, color="#2a2a4a", linestyle="--", linewidth=0.5)

        # ── static plot objects ──
        self.planned_line,  = self.ax.plot([], [], "o--", color="#5588ff",
                                           markersize=5, linewidth=1.5, zorder=2)
        self.waypoint_dots, = self.ax.plot([], [], "o", color="#88aaff",
                                           markersize=8, zorder=3)
        self.actual_line,   = self.ax.plot([], [], "-", color="#44dd88",
                                           linewidth=1.5, zorder=2)
        self.robot_dot,     = self.ax.plot([], [], "o", color="#ffffff",
                                           markersize=11, zorder=6)

        # lookahead target point — orange X
        self.target_dot,    = self.ax.plot([], [], "x", color=COLOR_TARGET,
                                           markersize=14, markeredgewidth=3, zorder=7,
                                           label="Target point")

        # line from robot to target point
        self.target_line,   = self.ax.plot([], [], "--", color=COLOR_TARGET,
                                           linewidth=1.2, alpha=0.6, zorder=4)

        # lookahead circle — added/removed each frame as a matplotlib patch
        self._lookahead_circle = None

        # heading arrow
        self._heading_arrow = None

        # ── legend ──
        handles = [
            mpatches.Patch(color="#5588ff",      label="Planned path"),
            mpatches.Patch(color="#44dd88",      label="Robot trail"),
            mpatches.Patch(color="#ffffff",      label="Robot"),
            mpatches.Patch(color=COLOR_CURRENT,  label="Heading"),
            mpatches.Patch(color=COLOR_TARGET,   label="Target point"),
            mpatches.Patch(color=COLOR_LOOKAHEAD,label="Lookahead circle"),
        ]
        self.ax.legend(handles=handles, loc="lower right",
                       facecolor="#2a2a4a", labelcolor="#ccccff", edgecolor="#444466")

        # ── info box ──
        self.info_text = self.ax.text(
            0.02, 0.97, "", transform=self.ax.transAxes,
            verticalalignment="top", fontsize=9, color="#ddddff",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#2a2a4a", alpha=0.85)
        )

        self._timer = self.fig.canvas.new_timer(interval=int(1000 / UPDATE_HZ))
        self._timer.add_callback(self._update)
        self._timer.start()

    def _make_arrow(self, existing, ox, oy, yaw, color, scale=1.0):
        if existing is not None:
            existing.remove()
        length = ARROW_LEN * scale
        dx = length * math.cos(yaw)
        dy = length * math.sin(yaw)
        return self.ax.annotate(
            "", xy=(ox + dx, oy + dy), xytext=(ox, oy),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=2.5, mutation_scale=16),
            zorder=8
        )

    def _update(self):
        with self.node._lock:
            rx     = self.node.robot_x
            ry     = self.node.robot_y
            ryaw   = self.node.robot_yaw
            spd    = self.node.robot_speed
            tx     = self.node.target_x
            ty     = self.node.target_y
            ld     = self.node.lookahead_dist
            axs    = list(self.node.actual_xs)
            ays    = list(self.node.actual_ys)
            pxs    = list(self.node.planned_xs)
            pys    = list(self.node.planned_ys)
            lrpm   = self.node.left_rpm
            rrpm   = self.node.right_rpm

        # ── planned path ──
        if pxs:
            self.planned_line.set_data(pxs, pys)
            self.waypoint_dots.set_data(pxs, pys)

        # ── robot trail ──
        if axs:
            self.actual_line.set_data(axs, ays)

        # ── robot + heading ──
        if rx is not None:
            self.robot_dot.set_data([rx], [ry])
            if ryaw is not None:
                self._heading_arrow = self._make_arrow(
                    self._heading_arrow, rx, ry, ryaw, COLOR_CURRENT, scale=1.0)

            # ── lookahead circle ──
            if self._lookahead_circle is not None:
                self._lookahead_circle.remove()
                self._lookahead_circle = None
            if ld is not None:
                self._lookahead_circle = Circle(
                    (rx, ry), radius=ld,
                    fill=False, edgecolor=COLOR_LOOKAHEAD,
                    linewidth=1.5, linestyle="--", alpha=0.7, zorder=4
                )
                self.ax.add_patch(self._lookahead_circle)

            # ── target point + line to it ──
            if tx is not None:
                self.target_dot.set_data([tx], [ty])
                self.target_line.set_data([rx, tx], [ry, ty])
            else:
                self.target_dot.set_data([], [])
                self.target_line.set_data([], [])

        # ── auto-scale axes ──
        all_x = pxs + axs + ([rx] if rx is not None else [])
        all_y = pys + ays + ([ry] if ry is not None else [])
        if len(all_x) > 1:
            pad = max(1.0, (max(all_x) - min(all_x)) * 0.15,
                           (max(all_y) - min(all_y)) * 0.15)
            self.ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
            self.ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

        # ── info box ──
        lines = []
        if rx is not None:
            lines.append(f"Pos:      ({rx:+.2f},  {ry:+.2f}) m")
        if ryaw is not None:
            lines.append(f"Heading:  {math.degrees(ryaw):+.1f}°")
        if spd is not None:
            lines.append(f"Speed:    {spd:.2f} m/s")
        if tx is not None:
            lines.append(f"Target:   ({tx:+.2f},  {ty:+.2f}) m")
        if ld is not None:
            lines.append(f"Lookahead:{ld:.2f} m")
        if lrpm is not None:
            lines.append(f"RPM  L:   {lrpm:.1f}   R: {rrpm:.1f}")
        if axs:
            lines.append(f"Trail:    {len(axs)} pts")
        self.info_text.set_text("\n".join(lines) if lines else "Waiting for data…")

        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    plot = LivePlot(node)

    try:
        plot.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()