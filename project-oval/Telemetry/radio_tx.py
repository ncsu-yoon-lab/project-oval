import serial
import struct
import rclpy
from rclpy.node import Node
import csv
import time

ser = serial.Serial('/dev/ttyTelemetry', baudrate=57600, timeout=1)

CSV_FILE = "position_data_logger.csv"

def send_lat_lon(lat, lon):
    try:
        lat = float(lat)
        lon = float(lon)
        data = struct.pack('!dd', lat, lon)
        ser.write(data)
        print(f"Sent data: lat={lat}, lon={lon} -> {data.hex()} (size={len(data)} bytes)")
    except Exception as e:
        print(f"Failed to send lat/lon: {e}")

def read_csv_and_send():
    try:
        with open(CSV_FILE, mode='r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                rtk_lat = row.get("swift_latitude")
                rtk_lon = row.get("swift_longitude")

                if rtk_lat and rtk_lon:
                    send_lat_lon(rtk_lat, rtk_lon)
                    time.sleep(1) # (10 Hz rate)
                else:
                    print("Invalid or missing lat/lon in row")
    except Exception as e:
        print(f"Error reading CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Node("gps_csv_tx_node")

    try:
        read_csv_and_send()
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
