import math
import sys
import time
import csv
from pathlib import Path
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

from graph_nav_with_pure_pursuit.paths import routes_dir

POSE_TOPIC = '/odometry/filtered'
POSE_MESSAGE_TYPE = 'odometry'

STEERING_TOPIC = '/gemini/steering'
THROTTLE_TOPIC = '/gemini/throttle'

LOOKAHEAD_DISTANCE = 2.0    # in meters

# Throttle settings if you want to use fixed throttle
TARGET_THROTTLE = -15.0
MAX_GEMINI_THROTTLE = 15.0
MIN_THROTTLE = -15.0

# PID settings for steering control
STEERING_KP = 60.0
STEERING_KI = 0.0
STEERING_KD = 0.0

STEERING_CMD_MIN = -100.0
STEERING_CMD_MAX = 100.0
MAX_GEMINI_STEERING = 20.0

# Distance threshold to consider a waypoint "reached"
GOAL_TOLERANCE = 1.0    
STOP_AT_FINAL = True    # Whether to stop at the final waypoint
ENABLE_TEXT_MAP_LOG = False


def _resolve_route_csv() -> Path:
    route_csv = routes_dir() / "planned_route.csv"
    if route_csv.exists():
        return route_csv

    from ament_index_python.packages import get_package_share_directory

    share_dir = Path(get_package_share_directory("graph_nav_with_pure_pursuit"))
    return Path(share_dir) / "routes" / "planned_route.csv"

# Quaternion to Euler angles (roll, pitch, yaw) conversion
def euler_from_quaternion(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

# Normalize angle to [-pi, +pi]
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _closest_point_on_segment(
    ax: float, ay: float, bx: float, by: float, px: float, py: float
) -> tuple[float, float, float]:
    """Closest point on segment AB to P; returns (cx, cy, t) with t in [0,1]."""
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom < 1e-12:
        return ax, ay, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return ax + t * abx, ay + t * aby, t


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time=None):
        
        if current_time is None:
            current_time = time.monotonic()

        if self.prev_time is None:
            dt = 0.05
        else:
            dt = current_time - self.prev_time
            dt = max(dt, 0.001)

        # --------
        # heading_error > 0  →  Target is on the left  →  steering < 0 
        # heading_error < 0  →  Target is on the right  →  steering > 0
        # --------

        # P
        p_term = self.kp * error

        # I
        self.integral += error * dt
        i_term = self.ki * self.integral
    
        # D
        d_term = self.kd * (error - self.prev_error) / dt

        # error > 0 if target is on the left -> output should be negative to steer left
        output = -(p_term + i_term + d_term)

        # Clamp
        output = max(self.output_min, min(self.output_max, output))

        self.prev_error = error
        self.prev_time = current_time

        return output


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter("pose_topic", POSE_TOPIC)
        self.declare_parameter("pose_message_type", POSE_MESSAGE_TYPE)
        self.declare_parameter("steering_topic", STEERING_TOPIC)
        self.declare_parameter("throttle_topic", THROTTLE_TOPIC)

        self.waypoints = self._load_route()
        self.num_waypoints = len(self.waypoints)
        if self.num_waypoints == 0:
            raise RuntimeError(
                f"No waypoints found in route CSV: {_resolve_route_csv()}"
            )

        self._cum_s: list[float] = []
        self._seg_len: list[float] = []
        self._total_length = 0.0
        self._build_polyline_cache()

        # Monotonic arc-length progress along the polyline (meters from start vertex).
        self._s_progress = 0.0

        # ---- PID ----
        self.steering_pid = PIDController(
            kp=STEERING_KP,
            ki=STEERING_KI,
            kd=STEERING_KD,
            output_min=STEERING_CMD_MIN,
            output_max=STEERING_CMD_MAX,
        )

        # ---- State ----
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        self.reached_final = False
        self.pose_received = False # have we received the first pose message yet?
        self.last_text_map_log_time = 0.0

        # Speed estimation using position differentiation
        self.prev_x = None
        self.prev_y = None
        self.prev_pose_time = None

        # Pose subscriber
        pose_topic = str(self.get_parameter("pose_topic").value)
        pose_message_type = str(self.get_parameter("pose_message_type").value).strip().lower()
        if pose_message_type == "odometry":
            self.pose_sub = self.create_subscription(Odometry, pose_topic, self.odom_callback, 10)
        elif pose_message_type == "pose_stamped":
            self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        else:
            raise RuntimeError(
                "pose_message_type must be 'pose_stamped' or 'odometry', "
                f"got {pose_message_type!r}"
            )
        self.get_logger().info(
            f"Pure pursuit pose input: topic={pose_topic}, type={pose_message_type}"
        )

        # Drive publishers
        steering_topic = str(self.get_parameter("steering_topic").value)
        throttle_topic = str(self.get_parameter("throttle_topic").value)
        self.steering_pub = self.create_publisher(Int64, steering_topic, 10)
        self.throttle_pub = self.create_publisher(Int64, throttle_topic, 10)
        self.get_logger().info(
            f"Pure pursuit command output: steering={steering_topic}, throttle={throttle_topic}"
        )
        
        # Control loop at 20 Hz 
        self.timer = self.create_timer(0.05, self.control_loop)


    def _update_pose(self, pose, stamp, speed: float | None = None):
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        q = pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Velocity estimation: delta(distance) / delta(time)
        now = stamp.sec + stamp.nanosec * 1e-9
        if speed is not None and math.isfinite(speed):
            self.current_speed = speed
        elif self.prev_pose_time is not None and self.prev_x is not None:
            dt = now - self.prev_pose_time
            if dt > 0.001:
                dist = math.hypot(
                    self.current_x - self.prev_x,
                    self.current_y - self.prev_y,
                )
                # Noise reduction with low-pass filter
                raw_speed = dist / dt
                self.current_speed = 0.7 * self.current_speed + 0.3 * raw_speed
        
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.prev_pose_time = now
        self.pose_received = True

    def pose_callback(self, msg: PoseStamped):
        self._update_pose(msg.pose, msg.header.stamp)

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        self._update_pose(msg.pose.pose, msg.header.stamp, speed)


    def _build_polyline_cache(self) -> None:
        self._cum_s = [0.0] * self.num_waypoints
        self._seg_len = []
        for i in range(self.num_waypoints - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            L = math.hypot(x2 - x1, y2 - y1)
            self._seg_len.append(L)
            self._cum_s[i + 1] = self._cum_s[i] + L
        self._total_length = self._cum_s[-1] if self.num_waypoints else 0.0

    def _closest_arc_length(self, px: float, py: float) -> tuple[float, float]:
        """Return (arc_length_s, squared_distance) to closest point on polyline."""
        if self.num_waypoints == 1:
            wx, wy = self.waypoints[0]
            d2 = (px - wx) ** 2 + (py - wy) ** 2
            return 0.0, d2

        best_s = 0.0
        best_d2 = float("inf")
        for j in range(self.num_waypoints - 1):
            x1, y1 = self.waypoints[j]
            x2, y2 = self.waypoints[j + 1]
            cx, cy, t = _closest_point_on_segment(x1, y1, x2, y2, px, py)
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self._cum_s[j] + t * self._seg_len[j]
        return best_s, best_d2

    def _point_at_arc_length(self, s: float) -> tuple[float, float]:
        """Point on polyline at arc length s from start (clamped to [0, total_length])."""
        if self.num_waypoints == 1:
            return self.waypoints[0]

        s = max(0.0, min(s, self._total_length))
        if s <= 0.0:
            return self.waypoints[0]

        for j in range(self.num_waypoints - 1):
            s0 = self._cum_s[j]
            s1 = self._cum_s[j + 1]
            if s <= s1 + 1e-9:
                L = self._seg_len[j]
                if L < 1e-9:
                    return self.waypoints[j + 1]
                t = (s - s0) / L
                t = max(0.0, min(1.0, t))
                x1, y1 = self.waypoints[j]
                x2, y2 = self.waypoints[j + 1]
                return x1 + t * (x2 - x1), y1 + t * (y2 - y1)

        return self.waypoints[-1]

    def _segment_index_at_s(self, s: float) -> int:
        """Vertex segment index j such that projection lies on segment (j, j+1), or last segment."""
        if self.num_waypoints <= 1:
            return 0
        s = max(0.0, min(s, self._total_length))
        for j in range(self.num_waypoints - 1):
            if s <= self._cum_s[j + 1] + 1e-9:
                return j
        return self.num_waypoints - 2

    def _load_route(self):
        route_csv = _resolve_route_csv()
        if not route_csv.exists():
            self.get_logger().warning(f"No route file found: {route_csv}")
            return []

        waypoints = []
        with route_csv.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    waypoints.append((float(row["x_m"]), float(row["y_m"])))
                except (KeyError, ValueError):
                    continue
        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {route_csv}")
        return waypoints


    def _publish_drive(self, throttle: float, steering: float):
        # publish steering and throttle commands as Int64 messages, which will be used by the driver node

        throttle = max(-MAX_GEMINI_THROTTLE, min(MAX_GEMINI_THROTTLE, throttle))
        steering = max(-MAX_GEMINI_STEERING, min(MAX_GEMINI_STEERING, steering))
        
        steer_msg = Int64()
        steer_msg.data = int(steering)
        self.steering_pub.publish(steer_msg)

        thr_msg = Int64()
        thr_msg.data = int(throttle)
        self.throttle_pub.publish(thr_msg)


    def control_loop(self):
        
        # -----
        # MAIN CONTROL LOGIC
        # -----

        if not self.pose_received:
            return

        if self.reached_final:
            self._publish_drive(0.0, 0.0)
            return
        
        lookahead_dist = LOOKAHEAD_DISTANCE

        # 1) Project vehicle onto polyline; advance progress monotonically (never move backward).
        s_closest, _ = self._closest_arc_length(self.current_x, self.current_y)
        self._s_progress = max(self._s_progress, s_closest)
        self._s_progress = min(self._s_progress, self._total_length)

        # Remaining distance along path (for logging / goal).
        if self.num_waypoints == 1:
            gx, gy = self.waypoints[0]
            dist_along_remaining = math.hypot(gx - self.current_x, gy - self.current_y)
        else:
            dist_along_remaining = max(0.0, self._total_length - self._s_progress)

        # 2) Goal: end of path.
        if self.num_waypoints == 1:
            at_goal = dist_along_remaining < GOAL_TOLERANCE
        else:
            at_goal = self._s_progress >= self._total_length - GOAL_TOLERANCE

        if at_goal:
            self.reached_final = STOP_AT_FINAL
            if not STOP_AT_FINAL:
                self._s_progress = 0.0
            self.steering_pid.reset()
            self._publish_drive(0.0, 0.0)
            self._log_text_map(
                self.waypoints[-1][0],
                self.waypoints[-1][1],
                dist_along_remaining,
                0.0,
                0.0,
                0.0,
                lookahead_dist,
            )
            return

        # 3) Lookahead point along polyline from current progress.
        target_x, target_y = self._point_at_arc_length(
            min(self._s_progress + lookahead_dist, self._total_length)
        )

        to_tx = target_x - self.current_x
        to_ty = target_y - self.current_y
        if to_tx * to_tx + to_ty * to_ty < 1e-8:
            heading_error = 0.0
        else:
            heading_error = math.atan2(to_ty, to_tx) - self.current_yaw

        heading_error = normalize_angle(heading_error)

        steering_cmd = self.steering_pid.compute(heading_error)
        throttle_cmd = TARGET_THROTTLE

        self._publish_drive(throttle_cmd, steering_cmd)

        self._log_text_map(
            target_x,
            target_y,
            dist_along_remaining,
            heading_error,
            steering_cmd,
            throttle_cmd,
            lookahead_dist,
        )
        


    # To visualize the robot, waypoints, and lookahead point in a simple ASCII map in the terminal
    # Do not modify this function unless you want to change the visualization style or add more debug info
    def _log_text_map(
        self,
        target_x: float,
        target_y: float,
        dist_to_wp: float,
        heading_error: float,
        steering_cmd: float,
        throttle_cmd: float,
        lookahead_dist: float,
    ):
        if not ENABLE_TEXT_MAP_LOG:
            return

        text_map_cell_size = 0.2
        text_map_log_period = 0.5
        text_map_min_x = -4.0
        text_map_max_x = 4.0
        text_map_min_y = -4.0
        text_map_max_y = 4.0
        text_map_x_stretch = 1

        now = time.monotonic()
        if now - self.last_text_map_log_time < text_map_log_period:
            return
        self.last_text_map_log_time = now

        min_x = text_map_min_x
        max_x = text_map_max_x
        min_y = text_map_min_y
        max_y = text_map_max_y

        text_map_width = int(round((max_x - min_x) / text_map_cell_size)) + 1
        text_map_height = int(round((max_y - min_y) / text_map_cell_size)) + 1
        grid = [['.' for _ in range(text_map_width)] for _ in range(text_map_height)]

        def to_grid(x_value: float, y_value: float):
            col = int(round((x_value - min_x) / text_map_cell_size))
            row = int(round((max_y - y_value) / text_map_cell_size))
            col = max(0, min(text_map_width - 1, col))
            row = max(0, min(text_map_height - 1, row))
            return row, col

        def yaw_to_marker(yaw: float):
            yaw = normalize_angle(yaw)
            if -math.pi / 4 <= yaw < math.pi / 4:
                return '>'
            if math.pi / 4 <= yaw < 3 * math.pi / 4:
                return '^'
            if -3 * math.pi / 4 <= yaw < -math.pi / 4:
                return 'v'
            return '<'

        def draw_segment(start_x: float, start_y: float, end_x: float, end_y: float):
            start_row, start_col = to_grid(start_x, start_y)
            end_row, end_col = to_grid(end_x, end_y)
            row_delta = end_row - start_row
            col_delta = end_col - start_col
            steps = max(abs(row_delta), abs(col_delta), 1)

            if abs(col_delta) > abs(row_delta):
                marker = '-'
            elif abs(row_delta) > abs(col_delta):
                marker = '|'
            elif row_delta * col_delta >= 0:
                marker = '\\'
            else:
                marker = '/'

            for step in range(steps + 1):
                row = int(round(start_row + row_delta * step / steps))
                col = int(round(start_col + col_delta * step / steps))
                if grid[row][col] == '.':
                    grid[row][col] = marker

        for index in range(self.num_waypoints - 1):
            x1, y1 = self.waypoints[index]
            x2, y2 = self.waypoints[index + 1]
            draw_segment(
                x1,
                y1,
                x2,
                y2,
            )

        seg_highlight = self._segment_index_at_s(self._s_progress)
        for index, (wx, wy) in enumerate(self.waypoints):
            row, col = to_grid(wx, wy)
            if index == seg_highlight:
                marker = 'N'
            else:
                marker = str(index % 10)
            grid[row][col] = marker

        lookahead_x = target_x
        lookahead_y = target_y
        lookahead_row, lookahead_col = to_grid(lookahead_x, lookahead_y)
        if grid[lookahead_row][lookahead_col] == '.':
            grid[lookahead_row][lookahead_col] = 'L'
        elif grid[lookahead_row][lookahead_col] not in ('@',):
            grid[lookahead_row][lookahead_col] = '&'

        current_row, current_col = to_grid(self.current_x, self.current_y)
        current_marker = yaw_to_marker(self.current_yaw)
        if grid[current_row][current_col] == '.':
            grid[current_row][current_col] = current_marker
        else:
            grid[current_row][current_col] = '@'

        def stretch_row(row):
            return ''.join(cell * text_map_x_stretch for cell in row)

        lines = []
        for row_index, row in enumerate(grid):
            y_label = max_y - row_index * text_map_cell_size
            lines.append(f'{y_label:5.2f} | ' + stretch_row(row))
        map_text = '\n'.join(lines)
        x_axis = '       +' + '-' * (text_map_width * text_map_x_stretch)
        x_labels = (
            f'       x:[{min_x:.2f}, {max_x:.2f}] '
            f'cell={text_map_cell_size:.2f}m x_stretch={text_map_x_stretch}'
        )
        current_x_clipped = min(max(self.current_x, min_x), max_x)
        current_y_clipped = min(max(self.current_y, min_y), max_y)
        clipped = (current_x_clipped != self.current_x) or (current_y_clipped != self.current_y)
        pose_summary = (
            f'pose=({self.current_x:.2f}, {self.current_y:.2f}) '
            f'yaw={math.degrees(self.current_yaw):.1f}deg '
            f's={self._s_progress:.2f}/{self._total_length:.2f}m'
        )
        if clipped:
            pose_summary += (
                f' clipped_to=({current_x_clipped:.2f}, {current_y_clipped:.2f})'
            )
        control_summary = (
            f'remaining={dist_to_wp:.2f}m '
            f'lookahead={lookahead_dist:.2f}m '
            f'heading_error={math.degrees(heading_error):.1f}deg '
            f'steer={steering_cmd:.1f} '
            f'thr={throttle_cmd:.1f}'
        )
        legend = (
            'legend: >=current yaw  ^=+y  v=-y  <=-x  @=overlap  '
            'N=polyline vertex at s  L=lookahead  &=overlap  0-9=waypoints'
        )
        screen_text = (
            'ASCII map\n'
            f'{pose_summary}\n'
            f'{control_summary}\n'
            f'{legend}\n'
            f'{map_text}\n'
            f'{x_axis}\n'
            f'{x_labels}\n'
        )
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write(screen_text)
        sys.stdout.flush()

    





def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node._publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
