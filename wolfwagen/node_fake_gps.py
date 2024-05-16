import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import threading
from rclpy.node import Node
import time
import math
import numpy as np
import serial
import csv
from gps_formatter import GPS_Formatter as format


## Creating a Subscription
def main():

    rclpy.init()
    gps_node = rclpy.create_node('fake_gps_node')

    gps_pub = gps_node.create_publisher(Float64MultiArray, "fake_gps_topic", 1) # publishing one value for now as a test, later change the data type and values

    thread = threading.Thread(target=rclpy.spin, args=(gps_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = gps_node.create_rate(FREQ, gps_node.get_clock())

    while rclpy.ok():
        input_csv_file = 'gps_data_converted.csv'
        data_array = []
        gps_data = Float64MultiArray()

        with open(input_csv_file, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                data_array.append(row)

            for row in data_array:
                try:
                    if row[2] != "":
                        gps_data.data = [float(row[0]), float(row[1]), float(row[2])]
                    else:
                        gps_data.data = [float(row[0]), float(row[1]), 0.0]
                except IndexError:
                    gps_data.data = [0.0, 0.0, 0.0]
                gps_pub.publish(gps_data)
                time.sleep(3)

        rate.sleep()

    gps_node.destroy_node()
    rclpy.shutdown()







if __name__ == '__main__':
    main()
