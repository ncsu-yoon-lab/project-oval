# TODO: this is a functional package for intellisense:
import json
import math
import threading
import time

import numpy as np
import rclpy
from gps_msgs.msg import GPSFix
from rclpy.node import Node
from std_msgs.msg import Float64, Int64MultiArray

from PIDController import PIDController

TIMEOUT_ERR_THRESHOLD = 5  # 5 seconds

MAX_DRIVE_SPEED_M_S = 0.5
MAX_TURN_SPEED_RADS_SEC = math.pi / 4
ACCEPTABLE_TURN_ERR_TO_DRIVE = math.pi / 4


def deadzone(val, deadzone_=1e-5):
    return 0 if abs(val) <= deadzone_ else val


def clamp(val, min_, max_):
    return min(max(val, min_), max_)


def rotate(l, n):
    return l[n:] + l[:n]


def calculate_heading(start_loc, target):
    """
    Calculates the heading to a given position
    :param start_loc: current pos in [lat, lon]
    :param target: target position in [lat, lon]
    :return: heading needed in radians, clockwise from north
    """
    lat1 = start_loc[0] * math.pi / 180
    lon1 = start_loc[1] * math.pi / 180
    lat2 = target[0] * math.pi / 180
    lon2 = target[1] * math.pi / 180
    delta_lon = lon2 - lon1
    x_c = math.sin(delta_lon) * math.cos(lat2)
    y_c = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    beta = math.atan2(x_c, y_c)
    # beta_deg = beta * 360 / 2 / math.pi
    return beta


def input_modulus(input_, minimumInput, maximumInput):
    diff = maximumInput - minimumInput
    while input_ > maximumInput:
        input_ -= diff
    while input_ < minimumInput:
        input_ += diff
    return input_


def haversine_dist(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in meters between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    # sometimes a is a very small number (e.g. 1e-17) and it flips to -1e-17
    if a < 0:
        a = 0
    c = 2 * math.asin(math.sqrt(a))
    r = 6371
    return c * r * 1000


class PursuitNode(Node):
    def __init__(self, rclpy_args: None):
        if rclpy_args is None:
            rclpy_args = []

        rclpy.init(args=rclpy_args)
        super().__init__("PursuitNode")
        # PID stuff
        # negative P because current=distance, goal=0, goal-current < 0
        self.drive_p = -0.1
        self.drive_i = 0
        self.drive_d = 0
        self.drive_controller = PIDController(self.drive_p, self.drive_i, self.drive_d)

        self.turn_p = 178
        self.turn_i = 0
        self.turn_d = 0
        self.turn_controller = PIDController(self.turn_p, self.turn_i, self.turn_d)

        self.last_updates = {"rtk": 0.0, "path": 0.0, "heading": 0.0}
        self.rtk_data = None

        self.path = []
        self._new_path = False

        # TODO: change to filepath on the Jetson
        with open("oval_points.json") as file:
            self.all_points = json.load(file)

        self.heading = 0
        self._max_turn_err = math.pi  # 2pi / 2
        self.current_pos = self.all_points["origin"]
        self.current_heading = 0

        self.create_subscription(GPSFix, "/gpsfix", self.gps_update, 10)

        # TODO: path planning node here
        self.create_subscription(Float64MultiArray, "/path_planning/path", self.path_update, 10)

        self.throttle_publisher = self.create_publisher(Float64, "/pure_pursuit/throttle", 10)
        self.steer_publisher = self.create_publisher(Float64, "/pure_pursuit/steer", 10)

        # TODO ??? does this do
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()

    def gps_update(self, msg):
        if not np.isnan(msg.latitude) and not np.isnan(msg.longitude):
            # self.rtk_data = {
            #     "latitude": msg.latitude,
            #     "longitude": msg.longitude,
            #     "altitude": msg.altitude,
            #     "track": msg.track,
            #     "rtk_sats": msg.status.satellites_used
            # }
            self.current_pos = [msg.latitude, msg.longitude]
            self.current_heading = math.radians(msg.track)
            self.last_updates["rtk"] = time.time()
            self.get_logger().debug("Received RTK GPS data")
        else:
            self.get_logger().warn("Received invalid RTK GPS data (NaN values)")

    def _is_fresh(self, topic):
        return (time.time() - self.last_updates[topic]) < TIMEOUT_ERR_THRESHOLD

    def path_update(self, msg):
        points = msg.data
        self._new_path = True
        self.last_updates["path"] = time.time()
        self.path = points

    def get_node_loc(self, node_id: int):
        matches = []
        if node_id < 0:
            for node in self.all_points["nodes"]:
                if node["id"] == node_id:
                    matches.append(node["latlon"])
        else:
            for node in self.all_points["points"]:
                if node["id"] == node_id:
                    matches.append(node["latlon"])
        if len(matches) == 0:
            raise ValueError("No matching nodes were found.")
        return matches[0]

    def drive_to_node(self, node_id: int):
        # TODO: dont forget about this
        if not self._is_fresh("rtk"):
            self.get_logger().error("RTK data stale.")
            return
        # where we going
        target_loc = self.get_node_loc(node_id)
        # distance there
        dist = self._distance_to_node(node_id)

        # desired heading
        desired_heading = calculate_heading(self.current_pos, target_loc)
        # change to east being 0, CCW positive for math, and clamp down to -2pi, 2pi
        # TODO: is the % needed? was added for sim
        # TODO: is this entire line needed?
        # desired_heading = (math.pi / 2 - desired_heading) % (2 * math.pi)
        # clamp to 0, 2pi
        if desired_heading < 0:
            desired_heading += 2 * math.pi

        # heading ERR, (continuous input stuff)
        heading_err = input_modulus(desired_heading - self.current_heading, -self._max_turn_err, self._max_turn_err)
        # PID calcs
        # TODO: this will slow down for each node
        # TODO: make constant speed, slow down to turn..?
        # TODO: units for output? throttle? m/s? rads/s?
        drive_output = self.drive_controller.calculate(dist, 0) if abs(heading_err) < ACCEPTABLE_TURN_ERR_TO_DRIVE else 0
        turn_output = self.turn_controller.calculate(self.current_heading, desired_heading)

        # clamp to max robot speed
        drive_output = clamp(drive_output, 0, MAX_DRIVE_SPEED_M_S)
        turn_output = clamp(turn_output, -1 * MAX_TURN_SPEED_RADS_SEC, MAX_TURN_SPEED_RADS_SEC)

        # publish
        self.throttle_publisher.publish(drive_output)
        self.steer_publisher.publish(turn_output)

    def drive_path(self, points: list, acceptable_err: float=0.25):
        """
        Drives along a given list of nodes
        :param points: list of node IDs
        :param acceptable_err: acceptabale error in meters
        """
        num_pts = len(points)
        for i, point in enumerate(points):
            print(f"Heading towards node {point}, point {i + 1}/{num_pts}")
            print(f"Target: {self.get_node_loc(point)}, distance: {self._distance_to_node(point)}")
            steps = 0
            while self._distance_to_node(point) > acceptable_err:
                self.drive_to_node(point)
                steps += 1
                # print(self._distance_to_node(point))
            print(f"Made it to node {point} in {steps} steps")

    def _distance_to_node(self, node_id):
        """
        Gets the distance to a given node
        :param node_id: ID of a node
        :return: distance in meters
        """
        target_loc = self.get_node_loc(node_id)
        return haversine_dist(self.current_pos[0], self.current_pos[1], target_loc[0], target_loc[1])

with open("oval_points.json") as file:
    all_points = json.load(file)

with open("points.json") as file:
    path_points = json.load(file)
