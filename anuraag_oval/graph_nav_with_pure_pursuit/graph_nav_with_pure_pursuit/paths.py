"""Runtime path resolution for source and installed ROS package layouts."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = "graph_nav_with_pure_pursuit"
DEFAULT_PROJECT_BASE = Path("/home/wolfwagen/ros2_ws/src/project-oval/anuraag_oval")


def project_base_dir() -> Path:
    return Path(os.environ.get("ANURAAG_OVAL_BASE", DEFAULT_PROJECT_BASE)).expanduser()


def source_package_dir() -> Path | None:
    candidates = [
        project_base_dir() / PACKAGE_NAME,
        project_base_dir(),
        Path(__file__).resolve().parents[1],
    ]
    for candidate in candidates:
        if (candidate / "package.xml").exists() and (candidate / "maps").exists():
            return candidate
    return None


def package_share_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME))


def maps_dir() -> Path:
    source_dir = source_package_dir()
    if source_dir is not None and (source_dir / "maps").exists():
        return source_dir / "maps"
    return package_share_dir() / "maps"


def routes_dir() -> Path:
    source_dir = source_package_dir()
    if source_dir is not None:
        return source_dir / "routes"
    return package_share_dir() / "routes"


def plots_dir() -> Path:
    source_dir = source_package_dir()
    if source_dir is not None:
        return source_dir / "plots"
    return project_base_dir() / PACKAGE_NAME / "plots"
