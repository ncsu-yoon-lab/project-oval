"""Fixed-origin lat/lon to local XY conversion utilities."""

from __future__ import annotations

import csv
import math
from pathlib import Path

EARTH_RADIUS_M = 6_378_137.0


def ll_to_local_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """Convert WGS84 lat/lon (deg) to local tangent-plane XY (meters)."""
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    o_lat_r = math.radians(origin_lat)
    o_lon_r = math.radians(origin_lon)

    x_east = (lon_r - o_lon_r) * math.cos(o_lat_r) * EARTH_RADIUS_M
    y_north = (lat_r - o_lat_r) * EARTH_RADIUS_M
    return x_east, y_north


def load_graph_nodes(nodes_csv: Path) -> list[tuple[str, float, float]]:
    nodes: list[tuple[str, float, float]] = []
    with nodes_csv.open(newline="") as f:
        rows = list(csv.reader(f))

    start_idx = 0
    if rows and len(rows[0]) >= 3 and rows[0][0].strip().lower() in {"node_id", "name"}:
        start_idx = 1

    for row in rows[start_idx:]:
        if len(row) < 3:
            continue
        node_id = row[0].strip()
        if not node_id:
            continue
        nodes.append((node_id, float(row[1]), float(row[2])))
    return nodes


def write_graph_nodes_xy(
    nodes: list[tuple[str, float, float]], output_csv: Path, origin_lat: float, origin_lon: float
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["node_id", "x_m", "y_m", "lat", "lon"])
        for node_id, lat, lon in nodes:
            x_m, y_m = ll_to_local_xy(lat, lon, origin_lat, origin_lon)
            writer.writerow([node_id, f"{x_m:.6f}", f"{y_m:.6f}", f"{lat:.8f}", f"{lon:.8f}"])


def build_xy_map(nodes_csv: Path, out_csv: Path, origin_node: str | None = None) -> tuple[float, float]:
    nodes = load_graph_nodes(nodes_csv)
    if not nodes:
        raise ValueError(f"No nodes found in {nodes_csv}")

    origin_lat: float
    origin_lon: float
    if origin_node is None:
        _, origin_lat, origin_lon = nodes[0]
    else:
        match = next((n for n in nodes if n[0] == origin_node), None)
        if match is None:
            raise ValueError(f"origin_node {origin_node!r} not found in {nodes_csv}")
        _, origin_lat, origin_lon = match

    write_graph_nodes_xy(nodes, out_csv, origin_lat, origin_lon)
    return origin_lat, origin_lon


def main() -> None:
    here = Path(__file__).resolve().parents[1]
    nodes_csv = here / "maps" / "graph_nodes.csv"
    out_csv = here / "maps" / "graph_nodes_xy.csv"
    origin_lat, origin_lon = build_xy_map(nodes_csv, out_csv)
    print(f"wrote {out_csv} using origin lat/lon=({origin_lat:.8f}, {origin_lon:.8f})")


if __name__ == "__main__":
    main()
