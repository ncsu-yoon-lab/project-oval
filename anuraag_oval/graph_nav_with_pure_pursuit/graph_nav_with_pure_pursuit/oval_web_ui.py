"""Local web dashboard for the Oval navigation package."""

from __future__ import annotations

import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from graph_nav_with_pure_pursuit.frame_projection import ll_to_local_xy
from graph_nav_with_pure_pursuit.paths import maps_dir, routes_dir


PACKAGE = "graph_nav_with_pure_pursuit"
WS_ROOT = Path("/home/wolfwagen/ros2_ws")
PROJECT_ROOT = WS_ROOT / "src" / "project-oval"
PACKAGE_ROOT = PROJECT_ROOT / "anuraag_oval" / PACKAGE
VENV = PROJECT_ROOT / ".venv"


def _setup_prefix() -> str:
    parts = [
        "source /opt/ros/humble/setup.bash",
        f"source {WS_ROOT}/install/setup.bash",
    ]
    if (VENV / "bin" / "activate").exists():
        parts.append(f"source {VENV}/bin/activate")
    return " && ".join(parts)


def _ros_cmd(command: str) -> list[str]:
    return ["bash", "-lc", f"{_setup_prefix()} && cd {PACKAGE_ROOT} && {command}"]


COMMANDS: dict[str, dict[str, Any]] = {
    "sensors": {
        "label": "Sensors",
        "command": _ros_cmd(f"ros2 launch {PACKAGE} sensors_bringup.launch.py"),
        "danger": False,
    },
    "localization": {
        "label": "Localization",
        "command": _ros_cmd(f"ros2 launch {PACKAGE} localization.launch.py"),
        "danger": False,
    },
    "set_reference": {
        "label": "Set Reference",
        "command": _ros_cmd(f"ros2 run {PACKAGE} global_set_reference"),
        "danger": False,
    },
    "planner": {
        "label": "Planner",
        "command": _ros_cmd(f"ros2 run {PACKAGE} global_planner_node"),
        "danger": False,
    },
    "control_nodes": {
        "label": "Control Nodes",
        "command": _ros_cmd(f"ros2 launch {PACKAGE} control_nodes.launch.py"),
        "danger": True,
    },
    "rtk_control": {
        "label": "RTK Control",
        "command": _ros_cmd(f"ros2 launch {PACKAGE} rtk_control.launch.py"),
        "danger": True,
    },
    "manual_driver": {
        "label": "Manual Driver",
        "command": _ros_cmd(f"python3 {PACKAGE_ROOT}/{PACKAGE}/driver_node.py"),
        "danger": True,
    },
    "manual_xbox": {
        "label": "Xbox Copy",
        "command": _ros_cmd(f"python3 {PACKAGE_ROOT}/{PACKAGE}/xbox_controller_node_copy.py"),
        "danger": False,
    },
}


class ManagedProcess:
    def __init__(self, process_id: str, label: str, command: list[str]) -> None:
        self.process_id = process_id
        self.label = label
        self.command = command
        self.process: subprocess.Popen[str] | None = None
        self.logs: list[dict[str, Any]] = []
        self._seq = 0
        self._lock = threading.Lock()

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> None:
        if self.is_running():
            return
        self._append_log(f"$ {' '.join(self.command)}")
        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        threading.Thread(target=self._read_stdout, daemon=True).start()

    def stop(self) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

    def send_input(self, text: str) -> None:
        if not self.is_running() or self.process is None or self.process.stdin is None:
            raise RuntimeError(f"{self.label} is not running")
        self.process.stdin.write(text.rstrip("\n") + "\n")
        self.process.stdin.flush()
        self._append_log(f"> {text.rstrip()}")

    def status(self) -> dict[str, Any]:
        return {
            "id": self.process_id,
            "label": self.label,
            "running": self.is_running(),
            "returncode": None if self.process is None else self.process.poll(),
            "pid": None if self.process is None else self.process.pid,
        }

    def logs_since(self, offset: int) -> dict[str, Any]:
        with self._lock:
            lines = [line for line in self.logs if line["seq"] >= offset]
            next_offset = self._seq
        return {"lines": lines, "next_offset": next_offset}

    def _read_stdout(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for line in self.process.stdout:
            self._append_log(line.rstrip("\n"))
        rc = self.process.wait()
        self._append_log(f"[process exited: {rc}]")

    def _append_log(self, text: str) -> None:
        with self._lock:
            self.logs.append({"seq": self._seq, "time": time.time(), "text": text})
            self._seq += 1
            if len(self.logs) > 1500:
                self.logs = self.logs[-1000:]


class ProcessManager:
    def __init__(self) -> None:
        self.processes = {
            key: ManagedProcess(key, spec["label"], spec["command"])
            for key, spec in COMMANDS.items()
        }

    def command_specs(self) -> list[dict[str, Any]]:
        return [
            {
                "id": key,
                "label": spec["label"],
                "danger": bool(spec.get("danger", False)),
            }
            for key, spec in COMMANDS.items()
        ]

    def status(self) -> dict[str, Any]:
        return {key: proc.status() for key, proc in self.processes.items()}

    def start(self, process_id: str) -> None:
        self._get(process_id).start()

    def stop(self, process_id: str) -> None:
        self._get(process_id).stop()

    def stop_all(self) -> None:
        for proc in self.processes.values():
            proc.stop()

    def send_input(self, process_id: str, text: str) -> None:
        self._get(process_id).send_input(text)

    def logs(self, process_id: str, offset: int) -> dict[str, Any]:
        return self._get(process_id).logs_since(offset)

    def _get(self, process_id: str) -> ManagedProcess:
        if process_id not in self.processes:
            raise KeyError(process_id)
        return self.processes[process_id]


class TelemetryBridge:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {}
        self._started = False
        self._error = ""
        threading.Thread(target=self._run, daemon=True).start()

    def state(self) -> dict[str, Any]:
        with self._lock:
            payload = dict(self._state)
        payload["bridge_started"] = self._started
        payload["bridge_error"] = self._error
        return payload

    def _update(self, key: str, value: dict[str, Any]) -> None:
        value["received_at"] = time.time()
        with self._lock:
            self._state[key] = value

    def _run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import NavSatFix
            from std_msgs.msg import Int64, Int64MultiArray
        except Exception as exc:
            self._error = f"ROS imports failed: {exc}"
            return

        try:
            rclpy.init(args=None)
            node = rclpy.create_node("oval_web_ui_bridge")
            node.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
            node.create_subscription(PoseStamped, "/zed/zed_node/pose", self._on_pose, 10)
            node.create_subscription(NavSatFix, "/navsatfix", self._on_fix, 10)
            node.create_subscription(Int64, "/gemini/steering", lambda m: self._on_scalar("steering", m.data), 10)
            node.create_subscription(Int64, "/gemini/throttle", lambda m: self._on_scalar("throttle", m.data), 10)
            node.create_subscription(Int64MultiArray, "/motors/rpm", lambda m: self._on_array("rpm", list(m.data)), 10)
            self._started = True
            rclpy.spin(node)
        except Exception as exc:
            self._error = str(exc)

    def _on_odom(self, msg: Any) -> None:
        pose = msg.pose.pose
        self._update("odometry", {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": _yaw_from_quaternion(pose.orientation),
            "source": "/odometry/filtered",
        })

    def _on_pose(self, msg: Any) -> None:
        pose = msg.pose
        self._update("zed_pose", {
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": _yaw_from_quaternion(pose.orientation),
            "source": "/zed/zed_node/pose",
        })

    def _on_fix(self, msg: Any) -> None:
        self._update("gps", {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "status": int(msg.status.status),
            "source": "/navsatfix",
        })

    def _on_scalar(self, key: str, value: int) -> None:
        self._update(key, {"value": int(value)})

    def _on_array(self, key: str, value: list[int]) -> None:
        self._update(key, {"value": value})


def _yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _read_map_payload() -> dict[str, Any]:
    nodes_csv = maps_dir() / "graph_nodes.csv"
    edges_csv = maps_dir() / "graph_edges.csv"
    route_csv = routes_dir() / "planned_route.csv"

    nodes: list[dict[str, Any]] = []
    with nodes_csv.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No nodes found in {nodes_csv}")

    origin_lat = float(rows[0]["lat"])
    origin_lon = float(rows[0]["lon"])
    for row in rows:
        lat = float(row["lat"])
        lon = float(row["lon"])
        x_m, y_m = ll_to_local_xy(lat, lon, origin_lat, origin_lon)
        nodes.append({
            "id": row["node_id"],
            "lat": lat,
            "lon": lon,
            "x": x_m,
            "y": y_m,
        })

    edges: list[dict[str, str]] = []
    if edges_csv.exists():
        with edges_csv.open(newline="") as f:
            edges = [{"from": r["from"], "to": r["to"]} for r in csv.DictReader(f)]

    route: list[dict[str, Any]] = []
    if route_csv.exists():
        with route_csv.open(newline="") as f:
            for row in csv.DictReader(f):
                route.append({
                    "seq": int(row["seq"]),
                    "node_id": row["node_id"],
                    "lat": float(row["lat"]),
                    "lon": float(row["lon"]),
                    "x": float(row["x_m"]),
                    "y": float(row["y_m"]),
                })

    return {
        "origin": {"lat": origin_lat, "lon": origin_lon},
        "nodes": nodes,
        "edges": edges,
        "route": route,
        "paths": {
            "nodes_csv": str(nodes_csv),
            "edges_csv": str(edges_csv),
            "route_csv": str(route_csv),
        },
    }


class DashboardHandler(BaseHTTPRequestHandler):
    manager: ProcessManager
    telemetry: TelemetryBridge
    static_dir: Path

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/":
                self._send_file(self.static_dir / "index.html", "text/html")
            elif parsed.path == "/app.js":
                self._send_file(self.static_dir / "app.js", "application/javascript")
            elif parsed.path == "/style.css":
                self._send_file(self.static_dir / "style.css", "text/css")
            elif parsed.path == "/api/map":
                self._json(_read_map_payload())
            elif parsed.path == "/api/status":
                self._json({
                    "commands": self.manager.command_specs(),
                    "processes": self.manager.status(),
                    "telemetry": self.telemetry.state(),
                })
            elif parsed.path == "/api/logs":
                params = parse_qs(parsed.query)
                process_id = params.get("id", [""])[0]
                offset = int(params.get("offset", ["0"])[0])
                self._json(self.manager.logs(process_id, offset))
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            payload = self._read_json()
            if parsed.path == "/api/process/start":
                self.manager.start(str(payload["id"]))
                self._json({"ok": True})
            elif parsed.path == "/api/process/stop":
                self.manager.stop(str(payload["id"]))
                self._json({"ok": True})
            elif parsed.path == "/api/process/input":
                self.manager.send_input(str(payload["id"]), str(payload.get("text", "")))
                self._json({"ok": True})
            elif parsed.path == "/api/process/stop_all":
                self.manager.stop_all()
                self._json({"ok": True})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except Exception as exc:
            self._json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("content-length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length))

    def _json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: Path, content_type: str) -> None:
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main(args: list[str] | None = None) -> None:
    host = "127.0.0.1"
    port = 8000
    if args:
        if len(args) >= 1:
            port = int(args[0])
        if len(args) >= 2:
            host = args[1]

    DashboardHandler.manager = ProcessManager()
    DashboardHandler.telemetry = TelemetryBridge()
    DashboardHandler.static_dir = Path(__file__).with_name("web_ui_static")

    server = ThreadingHTTPServer((host, port), DashboardHandler)
    print(f"Oval web UI: http://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Oval web UI.", flush=True)
    finally:
        DashboardHandler.manager.stop_all()
        server.server_close()


if __name__ == "__main__":
    main(sys.argv[1:])
