"""Reference capture using current RTK GPS + ZED pose."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from graph_nav_with_pure_pursuit.frame_projection import ll_to_local_xy, load_graph_nodes
from graph_nav_with_pure_pursuit.paths import maps_dir


def _safe_float(value: Any) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(numeric):
        return numeric
    return None


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _compass_deg_to_map_yaw(compass_deg: float) -> float:
    """
    Compass bearing (0=N, 90=E, clockwise positive)
    -> ENU yaw (0=+x/East, CCW positive) in radians.
    """
    return _normalize_angle((math.pi / 2.0) - math.radians(compass_deg))


def _cardinal_to_compass_deg(cardinal: str) -> float | None:
    key = cardinal.strip().upper()
    lookup = {
        "N": 0.0,
        "NE": 45.0,
        "E": 90.0,
        "SE": 135.0,
        "S": 180.0,
        "SW": 225.0,
        "W": 270.0,
        "NW": 315.0,
    }
    return lookup.get(key)


def _resolve_graph_nodes_csv() -> Path:
    candidate = maps_dir() / "graph_nodes.csv"
    if candidate.exists():
        return candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("graph_nav_with_pure_pursuit"))
        return share / "maps" / "graph_nodes.csv"
    except Exception:
        return candidate


class SetReferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("set_reference")

        self.declare_parameter("fix_topic", "/navsatfix")
        self.declare_parameter("pose_topic", "/zed/zed_node/pose")
        self.declare_parameter("nodes_csv", "")
        self.declare_parameter("output_file", "/tmp/sdc_reference.json")

        self.declare_parameter("require_covariance", True)
        self.declare_parameter("max_horizontal_std_m", 8)
        self.declare_parameter("min_fix_status", 0)
        self.declare_parameter("startup_timeout_sec", 180.0)

        self.declare_parameter("heading_mode", "auto_motion")
        self.declare_parameter("auto_motion_min_distance_m", 3.0)
        self.declare_parameter("auto_motion_min_zed_distance_m", 1.0)
        self.declare_parameter("use_external_heading", False)
        self.declare_parameter("external_heading_deg", float("nan"))
        self.declare_parameter("external_heading_cardinal", "")

        fix_topic = str(self.get_parameter("fix_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        nodes_param = str(self.get_parameter("nodes_csv").value).strip()
        self.output_file = Path(str(self.get_parameter("output_file").value)).expanduser()

        self._require_covariance = bool(self.get_parameter("require_covariance").value)
        self._max_horizontal_std_m = max(0.05, float(self.get_parameter("max_horizontal_std_m").value))
        self._min_fix_status = int(self.get_parameter("min_fix_status").value)
        self._startup_timeout_sec = max(1.0, float(self.get_parameter("startup_timeout_sec").value))

        self._heading_mode = str(self.get_parameter("heading_mode").value).strip().lower()
        if self._heading_mode not in {"auto_motion", "external"}:
            raise RuntimeError(
                "heading_mode must be 'auto_motion' or 'external', "
                f"got {self._heading_mode!r}"
            )
        self._auto_motion_min_distance_m = max(
            0.5, float(self.get_parameter("auto_motion_min_distance_m").value)
        )
        self._auto_motion_min_zed_distance_m = max(
            0.1, float(self.get_parameter("auto_motion_min_zed_distance_m").value)
        )

        self._use_external_heading = bool(self.get_parameter("use_external_heading").value)
        ext_deg_raw = _safe_float(self.get_parameter("external_heading_deg").value)
        self._external_heading_deg = ext_deg_raw if ext_deg_raw is not None else float("nan")
        self._external_heading_cardinal = str(self.get_parameter("external_heading_cardinal").value).strip().upper()
        if self._heading_mode == "external" or self._use_external_heading:
            self._prompt_external_heading_interactive()

        nodes_csv = Path(nodes_param).expanduser() if nodes_param else _resolve_graph_nodes_csv()
        if not nodes_csv.exists():
            raise RuntimeError(f"graph_nodes.csv not found: {nodes_csv}")

        nodes = load_graph_nodes(nodes_csv)
        if not nodes:
            raise RuntimeError(f"No nodes in {nodes_csv}")
        _, self._origin_lat, self._origin_lon = nodes[0]

        self._latest_fix: tuple[float, float, float, bool, float, int] | None = None
        self._latest_pose: tuple[float, float, float, float] | None = None
        self._start_sample: tuple[float, float, float, float, float, float, float, float, int] | None = None
        self._done = False
        self._start_mono = time.monotonic()
        self._last_wait_log = 0.0

        self.create_subscription(NavSatFix, fix_topic, self._on_fix, 10)
        self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

        self.get_logger().info(
            f"Map origin (node[0]): lat={self._origin_lat:.8f}, lon={self._origin_lon:.8f} from {nodes_csv}"
        )
        self.get_logger().info(
            f"Waiting for current precise GPS + ZED pose once (fix={fix_topic}, pose={pose_topic})."
        )
        self.get_logger().info(
            f"GPS precision gate: status>={self._min_fix_status}, horizontal_std<={self._max_horizontal_std_m:.2f}m"
            + (", covariance_required" if self._require_covariance else "")
        )
        if self._heading_mode == "auto_motion":
            self.get_logger().info(
                "Heading mode: auto_motion. Drive straight until RTK displacement reaches "
                f"{self._auto_motion_min_distance_m:.2f} m."
            )
        else:
            self.get_logger().info("Heading mode: external.")

    def _prompt_external_heading_interactive(self) -> None:
        """Prompt for heading degrees + cardinal direction when run in a terminal."""
        if not sys.stdin or not sys.stdin.isatty():
            return

        print("Enter external heading for map reference.")
        print("Examples: degrees=143, cardinal=SE")
        print("Valid cardinals: N, NE, E, SE, S, SW, W, NW")

        while True:
            raw_deg = input("Compass heading in degrees (0-360): ").strip()
            value = _safe_float(raw_deg)
            if value is None:
                print("Invalid degrees. Enter a numeric value, e.g. 143.")
                continue
            self._external_heading_deg = value % 360.0
            break

        while True:
            card = input("Cardinal direction: ").strip().upper()
            if _cardinal_to_compass_deg(card) is None:
                print("Invalid cardinal. Use one of: N, NE, E, SE, S, SW, W, NW")
                continue
            self._external_heading_cardinal = card
            break

        self.get_logger().info(
            f"Using interactive heading: deg={self._external_heading_deg:.1f}, "
            f"card={self._external_heading_cardinal}"
        )

    def _resolve_external_heading_deg(self) -> float:
        deg_from_value: float | None = None
        deg_from_cardinal: float | None = None

        if math.isfinite(self._external_heading_deg):
            deg_from_value = self._external_heading_deg % 360.0
        if self._external_heading_cardinal:
            deg_from_cardinal = _cardinal_to_compass_deg(self._external_heading_cardinal)

        if deg_from_value is None and deg_from_cardinal is None:
            raise RuntimeError(
                "No external heading provided: set external_heading_deg or external_heading_cardinal"
            )

        if deg_from_value is not None and deg_from_cardinal is not None:
            delta = abs(((deg_from_value - deg_from_cardinal + 180.0) % 360.0) - 180.0)
            if delta > 30.0:
                self.get_logger().warning(
                    f"external_heading_deg ({deg_from_value:.1f}) and card ({self._external_heading_cardinal}) "
                    f"differ by {delta:.1f} deg; using external_heading_deg"
                )

        if deg_from_value is not None:
            return deg_from_value
        assert deg_from_cardinal is not None
        return deg_from_cardinal

    def _on_fix(self, msg: NavSatFix) -> None:
        lat = _safe_float(msg.latitude)
        lon = _safe_float(msg.longitude)
        if lat is None or lon is None:
            return

        status = int(msg.status.status)
        status_ok = status >= self._min_fix_status

        cov_type = int(msg.position_covariance_type)
        cov_known = cov_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
        var_e = _safe_float(msg.position_covariance[0])
        var_n = _safe_float(msg.position_covariance[4])
        std_h = float("inf")
        cov_ok = False
        if var_e is not None and var_n is not None and var_e >= 0.0 and var_n >= 0.0:
            std_h = math.sqrt(max(var_e, var_n))
            cov_ok = std_h <= self._max_horizontal_std_m

        precise = status_ok and cov_ok and (cov_known or not self._require_covariance)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_fix = (lat, lon, stamp, precise, std_h, status)

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        x = _safe_float(p.x)
        y = _safe_float(p.y)
        if x is None or y is None:
            return
        q = msg.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_pose = (x, y, yaw, stamp)

    def _try_finalize(self) -> None:
        if self._done:
            return

        if (time.monotonic() - self._start_mono) > self._startup_timeout_sec:
            raise RuntimeError(
                f"Timed out after {self._startup_timeout_sec:.1f}s waiting for reference capture"
            )

        if self._latest_fix is None or self._latest_pose is None:
            now = time.monotonic()
            if now - self._last_wait_log > 1.0:
                self.get_logger().warning("Waiting for both GPS fix and ZED pose...")
                self._last_wait_log = now
            return

        lat, lon, fix_stamp, precise, std_h, fix_status = self._latest_fix
        zx, zy, zyaw, pose_stamp = self._latest_pose

        if not precise:
            now = time.monotonic()
            if now - self._last_wait_log > 1.0:
                self.get_logger().warning(
                    f"GPS not precise yet (status={fix_status}, horizontal_std={std_h:.3f} m)."
                )
                self._last_wait_log = now
            return

        x_map, y_map = ll_to_local_xy(lat, lon, self._origin_lat, self._origin_lon)

        heading_source = self._heading_mode
        heading_deg: float | None = None
        auto_motion_distance_m: float | None = None
        auto_motion_zed_distance_m: float | None = None
        zed_motion_yaw: float | None = None

        if self._heading_mode == "auto_motion":
            if self._start_sample is None:
                self._start_sample = (
                    lat,
                    lon,
                    x_map,
                    y_map,
                    zx,
                    zy,
                    fix_stamp,
                    pose_stamp,
                    fix_status,
                )
                self.get_logger().info(
                    "Captured start sample. Drive straight slowly for "
                    f"{self._auto_motion_min_distance_m:.2f} m to compute heading."
                )
                return

            (
                start_lat,
                start_lon,
                start_x_map,
                start_y_map,
                start_zx,
                start_zy,
                start_fix_stamp,
                start_pose_stamp,
                _start_fix_status,
            ) = self._start_sample

            dx_map = x_map - start_x_map
            dy_map = y_map - start_y_map
            dzx = zx - start_zx
            dzy = zy - start_zy
            auto_motion_distance_m = math.hypot(dx_map, dy_map)
            auto_motion_zed_distance_m = math.hypot(dzx, dzy)

            if (
                auto_motion_distance_m < self._auto_motion_min_distance_m
                or auto_motion_zed_distance_m < self._auto_motion_min_zed_distance_m
            ):
                now = time.monotonic()
                if now - self._last_wait_log > 1.0:
                    self.get_logger().info(
                        "Waiting for straight-line calibration motion: "
                        f"gps={auto_motion_distance_m:.2f}/{self._auto_motion_min_distance_m:.2f} m, "
                        f"zed={auto_motion_zed_distance_m:.2f}/{self._auto_motion_min_zed_distance_m:.2f} m"
                    )
                    self._last_wait_log = now
                return

            yaw_global = math.atan2(dy_map, dx_map)
            zed_motion_yaw = math.atan2(dzy, dzx)
            yaw_offset = _normalize_angle(yaw_global - zed_motion_yaw)
            heading_deg = (90.0 - math.degrees(yaw_global)) % 360.0
        else:
            heading_deg = self._resolve_external_heading_deg()
            yaw_global = _compass_deg_to_map_yaw(heading_deg)
            yaw_offset = _normalize_angle(yaw_global - zyaw)

            start_lat = lat
            start_lon = lon
            start_x_map = x_map
            start_y_map = y_map
            start_zx = zx
            start_zy = zy
            start_fix_stamp = fix_stamp
            start_pose_stamp = pose_stamp

        # Anchor translation so map and ZED coincide at this current sample.
        tx = x_map - zx
        ty = y_map - zy

        payload = {
            "map_reference_version": 2,
            "origin_lat": self._origin_lat,
            "origin_lon": self._origin_lon,
            "capture_lat": lat,
            "capture_lon": lon,
            "captured_unix_sec": time.time(),
            "x_map": x_map,
            "y_map": y_map,
            "zed_x": zx,
            "zed_y": zy,
            "zed_yaw": zyaw,
            "yaw_global": yaw_global,
            "yaw_offset": yaw_offset,
            "heading_source": heading_source,
            "external_heading_deg": heading_deg,
            "external_heading_cardinal": self._external_heading_cardinal,
            "auto_motion_distance_m": auto_motion_distance_m,
            "auto_motion_zed_distance_m": auto_motion_zed_distance_m,
            "auto_motion_zed_yaw": zed_motion_yaw,
            "auto_motion_start_lat": start_lat,
            "auto_motion_start_lon": start_lon,
            "auto_motion_start_x_map": start_x_map,
            "auto_motion_start_y_map": start_y_map,
            "auto_motion_start_zed_x": start_zx,
            "auto_motion_start_zed_y": start_zy,
            "auto_motion_start_fix_stamp_sec": start_fix_stamp,
            "auto_motion_start_pose_stamp_sec": start_pose_stamp,
            "translation_x": tx,
            "translation_y": ty,
            "gps_horizontal_std_m": std_h,
            "gps_fix_status": fix_status,
            "fix_stamp_sec": fix_stamp,
            "pose_stamp_sec": pose_stamp,
        }

        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.output_file.write_text(json.dumps(payload, indent=2))
        self.get_logger().info(
            f"Reference saved to {self.output_file}: yaw_global={yaw_global:.4f} rad, "
            f"yaw_offset={yaw_offset:.4f} rad, heading_deg={heading_deg:.2f}, "
            f"translation=({tx:.3f}, {ty:.3f})"
        )
        self._done = True

    def run(self) -> None:
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            self._try_finalize()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SetReferenceNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
