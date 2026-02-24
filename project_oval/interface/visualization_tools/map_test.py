#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Gprmc
import folium
from std_msgs.msg import Int64MultiArray
from sensor_msgs.msg import Image
import math
import cv2
from cv_bridge import CvBridge
import os
import csv


def nmea_to_decimal(value: float, direction: str) -> float:
    """Convert NMEA lat/lon (DDmm.mmmm) to decimal degrees."""
    if value != value or value == 0.0:  # NaN or zero
        return float("nan")
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    decimal = degrees + minutes / 60.0
    if direction in ["S", "W"]:
        decimal = -decimal
    return decimal


class GPSMap(Node):
    def __init__(self):
        super().__init__("gps_map_plotter")
        self.subscription = self.create_subscription(
            Gprmc, "/gps/gprmc", self.listener_callback, 10
        )
        self.subscription_rpm = self.create_subscription(
            Int64MultiArray, "/motors/rpm", self.rpm_callback, 10
        )
        self.subscription_images = self.create_subscription(
            Image, "/zed/zed_node/left/image_rect_color", self.image_callback, 10
        )

        self.lats = []
        self.lons = []
        self.map_file = "/tmp/gps_map.html"
        self.last_lat_dd = None
        self.last_lon_dd = None
        self.image = None
        self.br = CvBridge()

        # Folders and CSV setup
        os.makedirs("images", exist_ok=True)
        self.csv_file = "gps_log.csv"
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["lat_dd", "lon_dd", "track", "image_name"])

        # Initialize empty map
        self.map = folium.Map(location=[0, 0], zoom_start=2)
        self.map.save(self.map_file)
        self.get_logger().info(f"Map initialized at {self.map_file}")

    def image_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def rpm_callback(self, msg):
        left_rpm = msg.data[0]
        right_rpm = msg.data[1]
        left_vel = left_rpm * 15 / 72 * 6 * math.pi / 60 / 12
        right_vel = right_rpm * 15 / 72 * 6 * math.pi / 60 / 12
        # self.get_logger().info(f"Left vel: {left_vel:.3f}, Right vel: {right_vel:.3f}")

    def listener_callback(self, msg: Gprmc):
        lat_dd = nmea_to_decimal(msg.lat, msg.lat_dir)
        lon_dd = nmea_to_decimal(msg.lon, msg.lon_dir)

        if not (lat_dd != lat_dd or lon_dd != lon_dd):  # skip NaNs
            if self.last_lat_dd != lat_dd or self.last_lon_dd != lon_dd:
                self.get_logger().info(f"New GPS coords: {lat_dd:.6f}, {lon_dd:.6f}")
                self.get_logger().info(f"Track: {msg.track}")

                self.last_lat_dd = lat_dd
                self.last_lon_dd = lon_dd

                # Save image with timestamp
                image_name = f"images/frame_{int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec / 1e6)}.png"
                if self.image is not None:
                    cv2.imwrite(image_name, self.image)
                    self.get_logger().info(f"Saved image {image_name}")

                    # Append to CSV
                    with open(self.csv_file, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow([lat_dd, lon_dd, msg.track, image_name])

            self.lats.append(lat_dd)
            self.lons.append(lon_dd)

            # Update folium map
            self.map = folium.Map(location=[lat_dd, lon_dd], zoom_start=15)

            # Plot all previous points
            for lat, lon in zip(self.lats, self.lons):
                folium.CircleMarker(
                    location=[lat, lon],
                    radius=3,
                    color="red",
                    fill=True,
                ).add_to(self.map)

            # Save updated map
            self.map.save(self.map_file)
            self.get_logger().info( 
                f"Updated map with point ({lat_dd:.6f}, {lon_dd:.6f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GPSMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
