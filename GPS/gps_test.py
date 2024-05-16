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

artificialGPSMode = False

def parse_gpgll1(data):
    print("Receiving data from GPS 1")
    
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

def parse_gpgll2(data):
    print("Receiving data from GPS 2")
    
    if data.startswith('$GNRMC'):

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
    art_gps_pub = gps_node.create_publisher(Float64MultiArray, "art_gps_topic", 1)

    thread = threading.Thread(target=rclpy.spin, args=(gps_node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = gps_node.create_rate(FREQ, gps_node.get_clock())

    # Initializing time
    new_time = time.time()

    with serial.Serial("/dev/ttyACM1", 115200, timeout = 1) as ser1:
        with serial.Serial("/dev/ttyACM0", 115200, timeout = 1) as ser2:
            while rclpy.ok():

                # Maybe consider deleting this?
                if time.time() - new_time > 0.1:

                    line1 = ser1.readline().decode('utf-8').strip()
                    latitude1, longitude1, heading1 = parse_gpgll1(line1)

                    line2 = ser2.readline().decode('utf-8').strip()
                    latitude2, longitude2, heading2 = parse_gpgll2(line2)

                    # Checks if the latitude is found for both or not and averages them
                    if (latitude1 is not None and latitude2 is not None):
                        latitude = (latitude2 + latitude1) / 2
                        print("Averaging")
                    elif (latitude1 is None):
                        print("GPS 2")
                        latitude = latitude2
                    else:
                        print("GPS 1")
                        latitude = latitude1

                    # Checks if the longitude is found for both or not and avergas them
                    if (longitude1 is not None and longitude2 is not None):
                        longitude = (longitude2 + longitude1) / 2
                    elif (longitude1 is None):
                        longitude = longitude2
                    else:
                        longitude = longitude1

                    # Checks if the heading is found for both or not and averages them
                    if ((heading1 is not None and heading2 is not None) and (heading1 != "" and heading2 != "" )):
                        heading = (float(heading2) + float(heading1)) / 2
                    elif (heading1 is None):
                        heading = heading2
                    else:
                        heading = heading1

                    # Checks if the latitude and longitude are none or if the artificalGPS mode is on
                    if (latitude is not None and longitude is not None) or artificialGPSMode:
                        input_csv_file = 'gps_data.csv'
                        data_array = []

                        # Formats the lat and longs
                        latitude = format.coord_converter(latitude)
                        longitude = format.coord_converter(longitude)

                        new_row = [latitude, longitude, heading,time.time()]
                        with open(input_csv_file, 'r') as csv_file:
                            csv_reader = csv.reader(csv_file)
                            for row in csv_reader:
                                data_array.append(row)
                        data_array.append(new_row)
                        print(new_row)

                        # Publish artificial GPS mode for testing 
                        if artificialGPSMode:
                            art_gps_data = Float64MultiArray()
                            for row in data_array:
                                try:
                                    if row[2] != "":
                                        
                                        art_gps_data.data = [float(row[0]), float(row[1]), float(row[2])]
                                    else:
                                        art_gps_data.data = [float(row[0]), float(row[1]), 0.0]
                                except IndexError:
                                    art_gps_data.data = [0.0, 0.0, 0.0]
                                art_gps_pub.publish(art_gps_data)
                                time.sleep(2)

                        with open(input_csv_file, 'w', newline='') as csv_file:
                            csv_writer = csv.writer(csv_file)
                            csv_writer.writerows(data_array)
                        
                        gps_data = Float64MultiArray()
                        if heading:
                            gps_data.data = [latitude, longitude, float(heading)]
                        else:
                            gps_data.data = [latitude, longitude, 0.0]
                        gps_pub.publish(gps_data)
                    else:
                        
                        row = ["Empty", "Empty", "Empty"]
                        print(row)
                        

                    new_time = time.time() 

                rate.sleep()

            gps_node.destroy_node()
            rclpy.shutdown()
            ser2.close()
            ser1.close()
            







if __name__ == '__main__':
    main()
