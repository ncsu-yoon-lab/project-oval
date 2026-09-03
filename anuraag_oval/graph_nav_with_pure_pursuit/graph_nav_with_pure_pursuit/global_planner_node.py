"""Planner node that converts BFS route output into a CSV route for control."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from maps.route_bfs import path_to_target_coords
from graph_nav_with_pure_pursuit.frame_projection import ll_to_local_xy
from graph_nav_with_pure_pursuit.paths import maps_dir as default_maps_dir
from graph_nav_with_pure_pursuit.paths import plots_dir as default_plots_dir
from graph_nav_with_pure_pursuit.paths import routes_dir as default_routes_dir
from graph_nav_with_pure_pursuit.reference_frame import map_to_route_planner_xy


TARGET_MENU: list[tuple[str, str]] = [
    ("1", "Hunt"),
    ("2", "Fits"),
    ("3", "EB1"),
    ("4", "EB2"),
    ("5", "EB3"),
    ("6", "Oval"),
]


def _closest_point_on_segment(
    ax: float, ay: float, bx: float, by: float, px: float, py: float
) -> tuple[float, float, float]:
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom < 1e-12:
        return ax, ay, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return ax + t * abx, ay + t * aby, t


def _polyline_offset_vector_at_point(
    points: list[tuple[float, float]], px: float, py: float
) -> tuple[int, float, float, float, float, float, float]:
    """Return (seg_idx, t, cx, cy, delta_x, delta_y, distance_m) from closest segment."""
    if len(points) < 2:
        return 0, 0.0, px, py, 0.0, 0.0, 0.0

    best_j = 0
    best_t = 0.0
    best_cx = points[0][0]
    best_cy = points[0][1]
    best_dx = 0.0
    best_dy = 0.0
    best_d2 = float("inf")
    for j in range(len(points) - 1):
        x1, y1 = points[j]
        x2, y2 = points[j + 1]
        cx, cy, t = _closest_point_on_segment(x1, y1, x2, y2, px, py)
        vx = px - cx
        vy = py - cy
        d2 = vx * vx + vy * vy
        if d2 < best_d2:
            best_j = j
            best_t = t
            best_cx = cx
            best_cy = cy
            best_dx = vx
            best_dy = vy
            best_d2 = d2
    return best_j, best_t, best_cx, best_cy, best_dx, best_dy, math.sqrt(best_d2)


def _polyline_arc_lengths(points: list[tuple[float, float]]) -> list[float]:
    if not points:
        return []
    out = [0.0]
    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        out.append(out[-1] + math.hypot(x2 - x1, y2 - y1))
    return out


def _interp_on_polyline(
    points: list[tuple[float, float]], cum_s: list[float], s: float
) -> tuple[float, float, int, float]:
    if not points:
        return 0.0, 0.0, 0, 0.0
    if len(points) == 1:
        return points[0][0], points[0][1], 0, 0.0

    total = cum_s[-1]
    s = max(0.0, min(s, total))
    for i in range(len(points) - 1):
        s0 = cum_s[i]
        s1 = cum_s[i + 1]
        if s <= s1 + 1e-9:
            seg = s1 - s0
            t = 0.0 if seg < 1e-12 else (s - s0) / seg
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            return x1 + t * (x2 - x1), y1 + t * (y2 - y1), i, t
    return points[-1][0], points[-1][1], len(points) - 2, 1.0


def _closest_point_on_polyline(points: list[tuple[float, float]], px: float, py: float) -> tuple[float, float, float]:
    if not points:
        return px, py, 0.0
    if len(points) == 1:
        x0, y0 = points[0]
        return x0, y0, math.hypot(px - x0, py - y0)

    best_cx = points[0][0]
    best_cy = points[0][1]
    best_d2 = float("inf")
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        cx, cy, _ = _closest_point_on_segment(x1, y1, x2, y2, px, py)
        d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy)
        if d2 < best_d2:
            best_d2 = d2
            best_cx = cx
            best_cy = cy
    return best_cx, best_cy, math.sqrt(best_d2)


def _chaikin_smooth(points: list[tuple[float, float]], iterations: int) -> list[tuple[float, float]]:
    if len(points) < 3 or iterations <= 0:
        return points

    out = points
    for _ in range(iterations):
        if len(out) < 3:
            break
        nxt: list[tuple[float, float]] = [out[0]]
        for i in range(len(out) - 1):
            x1, y1 = out[i]
            x2, y2 = out[i + 1]
            qx = 0.75 * x1 + 0.25 * x2
            qy = 0.75 * y1 + 0.25 * y2
            rx = 0.25 * x1 + 0.75 * x2
            ry = 0.25 * y1 + 0.75 * y2
            nxt.append((qx, qy))
            nxt.append((rx, ry))
        nxt.append(out[-1])
        out = nxt
    return out


def _resample_polyline(points: list[tuple[float, float]], spacing_m: float) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points
    spacing = max(0.05, spacing_m)
    cum_s = _polyline_arc_lengths(points)
    total = cum_s[-1]
    if total < 1e-9:
        return [points[0], points[-1]]

    samples: list[tuple[float, float]] = []
    n = int(math.floor(total / spacing))
    for k in range(n + 1):
        s = min(k * spacing, total)
        x, y, _, _ = _interp_on_polyline(points, cum_s, s)
        samples.append((x, y))
    if math.hypot(samples[-1][0] - points[-1][0], samples[-1][1] - points[-1][1]) > 1e-6:
        samples.append(points[-1])
    return samples


def _sample_every_n_excluding_endpoints(
    points: list[tuple[float, float]], every_n: int
) -> list[tuple[float, float]]:
    """Keep start/end and every Nth interior point."""
    if len(points) <= 2:
        return points

    n = max(1, every_n)
    out: list[tuple[float, float]] = [points[0]]
    for i in range(1, len(points) - 1):
        if i % n == 0:
            out.append(points[i])
    out.append(points[-1])
    return out


def _resample_polyline_to_count(points: list[tuple[float, float]], total_points: int) -> list[tuple[float, float]]:
    """Resample polyline to exactly total_points, preserving endpoints."""
    if len(points) < 2:
        return points

    n = max(2, total_points)
    cum_s = _polyline_arc_lengths(points)
    total = cum_s[-1]
    if total < 1e-9:
        return [points[0], points[-1]]

    out: list[tuple[float, float]] = []
    for k in range(n):
        s = (k / (n - 1)) * total
        x, y, _, _ = _interp_on_polyline(points, cum_s, s)
        out.append((x, y))
    return out


def _smooth_interior(points: list[tuple[float, float]], iterations: int, alpha: float = 0.35) -> list[tuple[float, float]]:
    """Post-sample smoothing that preserves point count and endpoints."""
    if len(points) < 3 or iterations <= 0:
        return points

    a = max(0.0, min(0.5, alpha))
    out = points[:]
    for _ in range(iterations):
        prev = out[:]
        for i in range(1, len(out) - 1):
            x0, y0 = prev[i - 1]
            x1, y1 = prev[i]
            x2, y2 = prev[i + 1]
            avg_x = 0.5 * (x0 + x2)
            avg_y = 0.5 * (y0 + y2)
            out[i] = (x1 + a * (avg_x - x1), y1 + a * (avg_y - y1))
    out[0] = points[0]
    out[-1] = points[-1]
    return out


def _cap_deviation_from_raw(
    candidate: list[tuple[float, float]], raw: list[tuple[float, float]], max_dev_m: float
) -> list[tuple[float, float]]:
    if not candidate or not raw or max_dev_m <= 0.0:
        return candidate

    out: list[tuple[float, float]] = []
    max_dev = max(0.05, max_dev_m)
    for i, (px, py) in enumerate(candidate):
        if i == 0:
            out.append(raw[0])
            continue
        if i == len(candidate) - 1:
            out.append(raw[-1])
            continue
        cx, cy, dist = _closest_point_on_polyline(raw, px, py)
        if dist <= max_dev or dist < 1e-9:
            out.append((px, py))
            continue
        scale = max_dev / dist
        out.append((cx + (px - cx) * scale, cy + (py - cy) * scale))
    return out


def _interpolate_rows_for_smoothed(
    raw_rows: list[tuple[str, float, float]],
    raw_xy: list[tuple[float, float]],
    smooth_xy: list[tuple[float, float]],
) -> list[tuple[str, float, float]]:
    if not raw_rows or not raw_xy or not smooth_xy:
        return raw_rows
    if len(raw_rows) != len(raw_xy):
        return raw_rows

    raw_s = _polyline_arc_lengths(raw_xy)
    smooth_s = _polyline_arc_lengths(smooth_xy)
    raw_total = raw_s[-1]
    smooth_total = smooth_s[-1]
    if raw_total < 1e-9 or smooth_total < 1e-9:
        return raw_rows

    out: list[tuple[str, float, float]] = []
    for idx, s_sm in enumerate(smooth_s):
        ratio = s_sm / smooth_total
        s_raw = ratio * raw_total
        _, _, seg_idx, t = _interp_on_polyline(raw_xy, raw_s, s_raw)
        node_a, lat_a, lon_a = raw_rows[seg_idx]
        _, lat_b, lon_b = raw_rows[min(seg_idx + 1, len(raw_rows) - 1)]
        lat = lat_a + t * (lat_b - lat_a)
        lon = lon_a + t * (lon_b - lon_a)
        out.append((f"{node_a}_s{idx}", lat, lon))
    return out


def _apply_smoothing_pipeline(
    input_xy: list[tuple[float, float]],
    smoothing_iterations: int,
    resample_spacing_m: float,
    max_deviation_m: float,
) -> tuple[list[tuple[float, float]], int]:
    """Single smoothing pass that restores original point count."""
    if len(input_xy) < 3:
        return input_xy, max(1, len(input_xy))

    target_count = len(input_xy)
    sample_every = max(1, len(input_xy) // 3)
    smooth_xy = _chaikin_smooth(input_xy, smoothing_iterations)
    smooth_xy = _resample_polyline(smooth_xy, resample_spacing_m)
    smooth_xy = _sample_every_n_excluding_endpoints(smooth_xy, sample_every)
    smooth_xy = _smooth_interior(smooth_xy, max(1, smoothing_iterations))
    smooth_xy = _resample_polyline_to_count(smooth_xy, target_count)
    smooth_xy = _cap_deviation_from_raw(smooth_xy, input_xy, max_deviation_m)
    return smooth_xy, sample_every


def _apply_lateral_offset(
    points: list[tuple[float, float]], offset_m: float
) -> list[tuple[float, float]]:
    """Shift points laterally along left-normal of local path tangent.

    Positive offset shifts left of path direction, negative shifts right.
    Offset ramps linearly from 0 at the first point to full offset at the last point.
    """
    if len(points) < 2 or abs(offset_m) < 1e-9:
        return points

    shifted: list[tuple[float, float]] = []
    last_idx = len(points) - 1
    for i, (x, y) in enumerate(points):
        if i == 0:
            tx = points[1][0] - points[0][0]
            ty = points[1][1] - points[0][1]
        elif i == len(points) - 1:
            tx = points[-1][0] - points[-2][0]
            ty = points[-1][1] - points[-2][1]
        else:
            tx = points[i + 1][0] - points[i - 1][0]
            ty = points[i + 1][1] - points[i - 1][1]

        norm = math.hypot(tx, ty)
        if norm < 1e-9:
            shifted.append((x, y))
            continue

        tx /= norm
        ty /= norm
        nx = -ty
        ny = tx
        ramp = i / last_idx
        point_offset = offset_m * ramp
        shifted.append((x + point_offset * nx, y + point_offset * ny))
    return shifted


def _resolve_data_dirs() -> tuple[Path, Path]:
    return default_maps_dir(), default_routes_dir()


def _save_before_after_plot(
    before_xy: list[tuple[float, float]],
    after_xy: list[tuple[float, float]],
    output_dir: Path,
    target: str,
) -> Path | None:
    if not before_xy or not after_xy:
        return None

    try:
        import matplotlib.pyplot as plt
    except Exception:
        return None

    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_target = "".join(ch if ch.isalnum() else "_" for ch in target)
    out_png = output_dir / f"planner_before_after_{safe_target}_{stamp}.png"

    bx = [p[0] for p in before_xy]
    by = [p[1] for p in before_xy]
    ax = [p[0] for p in after_xy]
    ay = [p[1] for p in after_xy]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].plot(bx, by, "o-", color="#1f77b4", linewidth=1.8, markersize=3.5)
    axes[0].set_title("Before (No Smoothing/Offset)")
    axes[0].set_xlabel("x_m")
    axes[0].set_ylabel("y_m")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(ax, ay, "o-", color="#2ca02c", linewidth=1.8, markersize=3.5)
    axes[1].set_title("After (Smoothing + Offset)")
    axes[1].set_xlabel("x_m")
    axes[1].set_ylabel("y_m")
    axes[1].axis("equal")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("Planner Route Before vs After")
    fig.tight_layout()
    fig.savefig(out_png, dpi=180)
    plt.close(fig)
    return out_png


def _resolve_target_interactive() -> str:
    menu_map = {k: v for k, v in TARGET_MENU}

    if not sys.stdin or not sys.stdin.isatty():
        raise RuntimeError("Interactive target input is required, but no terminal input is available.")

    print("Select target node:")
    print("  1. Hunt")
    print("  2. Fitts (Fits)")
    print("  3. EB1")
    print("  4. EB2")
    print("  5. EB3")
    print("  6. Oval")
    while True:
        raw = input("Enter number (1-6): ").strip()
        if not raw:
            print("Input required. Example: 5")
            continue

        by_number = menu_map.get(raw)
        if by_number is not None:
            return by_number

        print("Invalid input. Enter a number from 1 to 6.")


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("planner_node")

        self.declare_parameter("planned_route_csv", "")
        self.declare_parameter("reference_file", "/tmp/sdc_reference.json")
        self.declare_parameter("route_frame", "zed_reference")
        self.declare_parameter("smoothing_enabled", False)
        self.declare_parameter("smoothing_iterations", 2)
        self.declare_parameter("resample_spacing_m", 0.6)
        self.declare_parameter("max_deviation_m", 2.5)
        self.declare_parameter("lateral_offset_m", 0.0)
        self.declare_parameter("save_before_after_plot", True)
        self.declare_parameter("plot_output_dir", str(default_plots_dir()))
        self._ref_payload: dict = {}
        self.reference_origin, self.car_start_latlon = self._load_reference_origins(
            str(self.get_parameter("reference_file").value)
        )
        self._target = _resolve_target_interactive()
        self.get_logger().info(f"Target selected: {self._target}")
        self._plan_and_write_route()

    def _load_reference_origins(self, reference_file: str) -> tuple[tuple[float, float], tuple[float, float]]:
        if not reference_file:
            raise RuntimeError("reference_file parameter is empty")

        ref_path = Path(reference_file).expanduser()
        if not ref_path.exists():
            raise RuntimeError(f"Reference file not found: {ref_path}")

        try:
            payload = json.loads(ref_path.read_text())
            origin_lat = float(payload["origin_lat"])
            origin_lon = float(payload["origin_lon"])
            car_lat = float(payload["capture_lat"])
            car_lon = float(payload["capture_lon"])
        except (OSError, ValueError, KeyError, TypeError):
            raise RuntimeError(f"Failed to parse reference file: {ref_path}")

        self._ref_payload = payload

        self.get_logger().info(
            f"Loaded reference origin from {ref_path}: lat={origin_lat:.8f}, lon={origin_lon:.8f}; "
            f"plan start lat/lon=({car_lat:.8f}, {car_lon:.8f})"
            "; route CSV in rotated planner frame (yaw_global)"
        )
        return (origin_lat, origin_lon), (car_lat, car_lon)

    def _plan_and_write_route(self) -> None:
        maps_dir, routes_dir = _resolve_data_dirs()
        routes_dir.mkdir(parents=True, exist_ok=True)

        nodes_csv = maps_dir / "graph_nodes.csv"
        edges_csv = maps_dir / "graph_edges.csv"
        car_lat, car_lon = self.car_start_latlon
        origin_lat, origin_lon = self.reference_origin

        target = self._target
        planned_route_csv = str(self.get_parameter("planned_route_csv").value).strip()
        route_frame = str(self.get_parameter("route_frame").value).strip().lower()
        if route_frame not in {"zed_reference", "map_enu"}:
            raise RuntimeError(
                "route_frame must be 'zed_reference' or 'map_enu', "
                f"got {route_frame!r}"
            )
        smoothing_enabled = bool(self.get_parameter("smoothing_enabled").value)
        smoothing_iterations = max(0, int(self.get_parameter("smoothing_iterations").value))
        resample_spacing_m = max(0.05, float(self.get_parameter("resample_spacing_m").value))
        max_deviation_m = max(0.0, float(self.get_parameter("max_deviation_m").value))
        lateral_offset_m = float(self.get_parameter("lateral_offset_m").value)
        save_before_after_plot = bool(self.get_parameter("save_before_after_plot").value)
        plot_output_dir = Path(str(self.get_parameter("plot_output_dir").value)).expanduser()

        route = path_to_target_coords(
            car_lat,
            car_lon,
            target,
            lat_lon_csv=nodes_csv,
            edges_csv=edges_csv,
        )

        planned_csv = (
            Path(planned_route_csv).expanduser().resolve()
            if planned_route_csv
            else routes_dir / "planned_route.csv"
        )
        planned_csv.parent.mkdir(parents=True, exist_ok=True)

        route_rows: list[tuple[str, float, float]] = []
        route_xy: list[tuple[float, float]] = []
        for node_id, lat, lon in route:
            x_m, y_m = ll_to_local_xy(lat, lon, origin_lat, origin_lon)
            if route_frame == "zed_reference":
                x_m, y_m = map_to_route_planner_xy(x_m, y_m, self._ref_payload)
            route_rows.append((node_id, lat, lon))
            route_xy.append((x_m, y_m))

        # Shift full route by the closest-point perpendicular vector from path to zed_x/y.
        zed_x = self._ref_payload.get("zed_x")
        zed_y = self._ref_payload.get("zed_y")
        if route_frame == "zed_reference" and zed_x is not None and zed_y is not None:
            seg_idx, seg_t, cx, cy, delta_x, delta_y, abs_dist_m = _polyline_offset_vector_at_point(
                route_xy, float(zed_x), float(zed_y)
            )
            # Start the route from the closest projected point, then keep the remaining suffix.
            node_id_a, lat_a, lon_a = route_rows[seg_idx]
            _, lat_b, lon_b = route_rows[seg_idx + 1]
            proj_lat = lat_a + seg_t * (lat_b - lat_a)
            proj_lon = lon_a + seg_t * (lon_b - lon_a)

            shifted_rows: list[tuple[str, float, float]] = [(f"{node_id_a}_proj", proj_lat, proj_lon)]
            shifted_xy: list[tuple[float, float]] = [(cx + delta_x, cy + delta_y)]

            for (node_id, lat, lon), (x, y) in zip(
                route_rows[seg_idx + 1 :], route_xy[seg_idx + 1 :], strict=True
            ):
                shifted_rows.append((node_id, lat, lon))
                shifted_xy.append((x + delta_x, y + delta_y))

            route_rows = shifted_rows
            route_xy = shifted_xy
            self.get_logger().info(
                f"Applied route shift from zed_x/y projection: "
                f"seg={seg_idx}, t={seg_t:.3f}, delta_x={delta_x:.3f}m, "
                f"delta_y={delta_y:.3f}m, perp_dist={abs_dist_m:.3f}m, "
                f"new_points={len(route_rows)}"
            )

        before_xy = list(route_xy)

        if smoothing_enabled and len(route_xy) >= 3:
            first_in = route_xy
            first_out, first_sample_every = _apply_smoothing_pipeline(
                first_in,
                smoothing_iterations=smoothing_iterations,
                resample_spacing_m=resample_spacing_m,
                max_deviation_m=max_deviation_m,
            )
            second_in = first_out
            second_out, second_sample_every = _apply_smoothing_pipeline(
                second_in,
                smoothing_iterations=smoothing_iterations,
                resample_spacing_m=resample_spacing_m,
                max_deviation_m=max_deviation_m,
            )
            route_xy = second_out
            self.get_logger().info(
                "Applied route smoothing (2-pass): "
                f"iterations={smoothing_iterations}, spacing={resample_spacing_m:.2f}m, "
                f"pass1_sample_every={first_sample_every}, pass2_sample_every={second_sample_every}, "
                f"restored_count={len(first_in)}, max_deviation={max_deviation_m:.2f}m, "
                f"points {len(first_in)} -> {len(first_out)} -> {len(second_out)}"
            )

        if len(route_xy) >= 2 and abs(lateral_offset_m) > 1e-9:
            route_xy = _apply_lateral_offset(route_xy, lateral_offset_m)
            self.get_logger().info(
                f"Applied lateral offset: {lateral_offset_m:.3f}m "
                "(+ left of path direction, - right)"
            )

        with planned_csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["seq", "node_id", "lat", "lon", "x_m", "y_m"])
            for seq, ((node_id, lat, lon), (x_m, y_m)) in enumerate(
                zip(route_rows, route_xy, strict=True)
            ):
                writer.writerow([
                    seq,
                    node_id,
                    f"{lat:.8f}",
                    f"{lon:.8f}",
                    f"{x_m:.6f}",
                    f"{y_m:.6f}",
                ])

        self.get_logger().info(f"Wrote planned route with {len(route)} points: {planned_csv}")
        self.get_logger().info(
            f"Route origin lat/lon=({origin_lat:.8f}, {origin_lon:.8f}); "
            f"start=({car_lat:.8f}, {car_lon:.8f}); target={target}; route_frame={route_frame}"
        )

        if save_before_after_plot:
            plot_path = _save_before_after_plot(before_xy, route_xy, plot_output_dir, target)
            if plot_path is None:
                self.get_logger().warning(
                    "Could not save before/after plot. "
                    "Install matplotlib or disable save_before_after_plot."
                )
            else:
                self.get_logger().info(f"Wrote before/after plot: {plot_path}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
