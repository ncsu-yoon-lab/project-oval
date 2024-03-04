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

def parse_gpgll(data):
    if data.startswith('$GPRMC'):

        parts = data.split(',')

        if len(parts) >= 5:

            latitude = parts[3]
            longitude = parts[5]
            heading = parts[8]
            if (len(latitude) > 0 and len(longitude) > 0):
                if latitude[0] == "0":
                    latitude = '-' + latitude[1:]
                if longitude[0] == "0":
                    longitude = '-' + longitude[1:]
                return float(latitude), float(longitude), heading
        return None, None, None
    return None, None, None

## Creating a Subscription
def main():

    rclpy.init()
    gps_node = rclpy.create_node('gps_node')

    gps_pub = gps_node.create_publisher(Float64MultiArray, "gps_topic", 1) # publishing one value for now as a test, later change the data type and values

    thread = threading.Thread(target=rclpy.spin, args=(gps_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = gps_node.create_rate(FREQ, gps_node.get_clock())

    # Initializing time
    new_time = time.time()

    with open('gps_data.csv', 'w') as file:
        writer = csv.writer(file, delimiter='\t', lineterminator='\n',)
        with serial.Serial("/dev/ttyACM0", 115200, timeout = 1) as ser:
            while rclpy.ok():

                # Only getting position and yaw every second and that data is being transferred
                if time.time() - new_time > 1:

                    line = ser.readline().decode('utf-8').strip()
                    latitude, longitude, heading = parse_gpgll(line)

                    if latitude is not None and longitude is not None:
                        row = [latitude, longitude, heading]
                        writer.writerow(row)
                        print(row)
                        gps_data = Float64MultiArray()
                        gps_data.data = [latitude, longitude, float(heading)]
                        gps_pub.publish(gps_data)
                    else:
                        row = ["Empty", "Empty", "Empty"]
                        print(row)
                        writer.writerow(row)
                        gps_data = Float64MultiArray()
                        gps_data.data = [0.0, 0.0, 0.0]
                        gps_pub.publish(gps_data)


                    new_time = time.time() 
                rate.sleep()

            gps_node.destroy_node()
            rclpy.shutdown()
            ser.close()
        file.close()






if __name__ == '__main__':
    main()
