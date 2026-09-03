"""
BFS routing on the waypoint graph from graph_nodes.csv / graph_edges.csv.
"""

from __future__ import annotations

import csv
import math
from collections import deque
from pathlib import Path

_EARTH_RADIUS_M = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two WGS84 lat/lon points."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    h = min(1.0, max(0.0, h))
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(h))


def _load_waypoints(lat_lon_csv: Path) -> dict[str, tuple[float, float]]:
    waypoints: dict[str, tuple[float, float]] = {}
    with lat_lon_csv.open(newline="") as f:
        for row in csv.reader(f):
            if not row or not row[0].strip():
                continue
            first = row[0].strip().lower()
            if first in {"node_id", "name"}:
                continue
            name = row[0].strip()
            lat, lon = float(row[1].strip()), float(row[2].strip())
            waypoints[name] = (lat, lon)
    return waypoints


def _load_edges_undirected(edges_csv: Path) -> dict[str, set[str]]:
    """Symmetric adjacency: each undirected edge is one hop for BFS."""
    adj: dict[str, set[str]] = {}
    with edges_csv.open(newline="") as f:
        rows = list(csv.reader(f))
    i = 0
    if (
        rows
        and len(rows[0]) >= 2
        and rows[0][0].strip().lower() == "from"
        and rows[0][1].strip().lower() == "to"
    ):
        i = 1
    for row in rows[i:]:
        if len(row) < 2:
            continue
        a, b = row[0].strip(), row[1].strip()
        if not a or not b:
            continue
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    return adj


def _closest_waypoint_name(
    lat: float, lon: float, waypoints: dict[str, tuple[float, float]]
) -> tuple[str, float]:
    best_name: str | None = None
    best_d = math.inf
    for name, (wlat, wlon) in waypoints.items():
        d = _haversine_m(lat, lon, wlat, wlon)
        if (
            best_name is None
            or d < best_d
            or (math.isclose(d, best_d) and name < best_name)
        ):
            best_d = d
            best_name = name
    assert best_name is not None
    return best_name, best_d


def _bfs_path(adj: dict[str, set[str]], start: str, goal: str) -> list[str] | None:
    if start == goal:
        return [start]
    if start not in adj or goal not in adj:
        return None
    parent: dict[str, str | None] = {start: None}
    q: deque[str] = deque([start])
    while q:
        u = q.popleft()
        for v in adj.get(u, ()):
            if v in parent:
                continue
            parent[v] = u
            if v == goal:
                out: list[str] = []
                cur: str | None = goal
                while cur is not None:
                    out.append(cur)
                    cur = parent[cur]
                out.reverse()
                return out
            q.append(v)
    return None


def path_to_target(
    car_lat: float,
    car_lon: float,
    target: str,
    *,
    lat_lon_csv: Path | None = None,
    edges_csv: Path | None = None,
) -> list[str]:
    """
    Return a list of waypoint names from the graph node nearest the car to ``target``.

    Steps:
    1. Pick the waypoint in ``graph_nodes.csv`` whose coordinates minimize geodesic distance
       (haversine on WGS84) to ``(car_lat, car_lon)``.
    2. Run BFS on the graph from ``graph_edges.csv`` with unit edge weights (shortest hop count).
       Edges are treated as **undirected** so the track can be traversed in either direction.

    Parameters
    ----------
    car_lat, car_lon
        Current vehicle position in decimal degrees.
    target
        Must match a name in ``graph_nodes.csv`` (e.g. ``\"N42\"``, ``\"Fits\"``, ``\"EB1\"``).

    Returns
    -------
    list[str]
        Waypoint names from the closest node (inclusive) to ``target`` (inclusive).

    Raises
    ------
    ValueError
        If ``target`` is unknown or no path exists in the graph.
    """
    here = Path(__file__).resolve().parent
    lat_lon_path = lat_lon_csv or (here / "graph_nodes.csv")
    edges_path = edges_csv or (here / "graph_edges.csv")

    waypoints = _load_waypoints(lat_lon_path)
    if target not in waypoints:
        raise ValueError(f"Unknown target {target!r}; not found in {lat_lon_path}")

    start, _ = _closest_waypoint_name(car_lat, car_lon, waypoints)
    adj = _load_edges_undirected(edges_path)
    path = _bfs_path(adj, start, target)
    if path is None:
        raise ValueError(f"No path from nearest node {start!r} to {target!r} in {edges_path}")
    return path


def path_to_target_coords(
    car_lat: float,
    car_lon: float,
    target: str,
    *,
    lat_lon_csv: Path | None = None,
    edges_csv: Path | None = None,
) -> list[tuple[str, float, float]]:
    """Same as :func:`path_to_target`, but each step is ``(name, lat, lon)``."""
    here = Path(__file__).resolve().parent
    lat_lon_path = lat_lon_csv or (here / "graph_nodes.csv")
    w = _load_waypoints(lat_lon_path)
    names = path_to_target(
        car_lat, car_lon, target, lat_lon_csv=lat_lon_path, edges_csv=edges_csv
    )
    return [(n, w[n][0], w[n][1]) for n in names]


if __name__ == "__main__":
    # Example: car near start of file, target a landmark mid-map
    demo_lat, demo_lon = 35.76988, -78.67598
    tgt = "Fits"
    p = path_to_target(demo_lat, demo_lon, tgt)
    print("path:", " -> ".join(p))
