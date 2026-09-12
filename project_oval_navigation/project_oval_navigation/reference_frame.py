"""Map reference v2: ZED→map translation and route planner rotation about (x_map, y_map)."""

from __future__ import annotations

import math
from typing import Mapping, Any


def map_to_route_planner_xy(x: float, y: float, payload: Mapping[str, Any]) -> tuple[float, float]:
    """Convert map ENU coordinates into the ZED pose frame."""
    x_map_ref = float(payload["x_map"])
    y_map_ref = float(payload["y_map"])
    zed_x_ref = float(payload["zed_x"])
    zed_y_ref = float(payload["zed_y"])
    yaw_offset = float(payload["yaw_offset"])

    # ENU delta from captured map anchor (capture_lat/capture_lon projected).
    dx_map = x - x_map_ref
    dy_map = y - y_map_ref

    # yaw_map = yaw_zed + yaw_offset => map->zed rotation is -yaw_offset.
    c = math.cos(-yaw_offset)
    s = math.sin(-yaw_offset)
    dx_zed = c * dx_map - s * dy_map
    dy_zed = s * dx_map + c * dy_map
    return zed_x_ref + dx_zed, zed_y_ref + dy_zed
