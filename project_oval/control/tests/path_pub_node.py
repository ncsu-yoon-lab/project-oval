#!/usr/bin/env python3
"""
path_publisher.py

Publishes a hardcoded list of lat/lon waypoints as a nav_msgs/Path
to /pure_pursuit/path.

Usage:
    python3 path_publisher.py

The path is published once on startup, then re-published every 5 seconds
in case the pure_pursuit_node restarts.

NOTE: The pure_pursuit_node reads pose.position.x as LATITUDE
      and pose.position.y as LONGITUDE — matching latlon_to_meters(x, y).
      Keep that convention here.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


# ─────────────────────────────────────────────
#  EDIT YOUR WAYPOINTS HERE  (lat, lon)
# ─────────────────────────────────────────────
WAYPOINTS = [
    (35.771190, -78.673972),   # origin / start
    (35.77121570085506, -78.67406286566016),   # ~7.8 m north
    (35.77127337094601, -78.67418088285628),
    (35.771449645304436, -78.67452152339975),   # back south
    (35.77154539911294, -78.67467642097485),   # back to origin (closed loop)
]
# ─────────────────────────────────────────────

PUBLISH_INTERVAL_SEC = 5.0   # re-publish in case node restarts


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")

        self.pub = self.create_publisher(Path, "/pure_pursuit/path", 10)

        # Build the message once
        self.path_msg = self._build_path(WAYPOINTS)

        # Publish immediately, then on a timer
        self._one_shot_timer = self.create_timer(0.5, self._publish_once)
        self.create_timer(PUBLISH_INTERVAL_SEC, self._republish)

        self.get_logger().info(
            f"PathPublisher ready — {len(WAYPOINTS)} waypoints, "
            f"re-publishing every {PUBLISH_INTERVAL_SEC}s"
        )

    # ── helpers ──────────────────────────────

    def _build_path(self, waypoints):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for lat, lon in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = msg.header.stamp
            # pure_pursuit_node treats x=lat, y=lon  (see latlon_to_meters call in path_callback)
            ps.pose.position.x = lat
            ps.pose.position.y = lon
            ps.pose.position.z = 0.0
            msg.poses.append(ps)

        return msg

    def _publish_once(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)
        self.get_logger().info(f"Published path with {len(self.path_msg.poses)} poses")
        self.destroy_timer(self._one_shot_timer)  # ← use stored ref directly

    def _republish(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)
        self.get_logger().info("Re-published path")


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()