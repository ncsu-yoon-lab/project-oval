import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import threading

import serial

ser = serial.Serial("/dev/ttyUSB0")

lat = long = deg = 0

artificialGPS = False

def gps_callback(msg):
    global lat, long, deg
    
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = float(1)

    lat = float(x)
    long = float(y)
    deg = z

def art_gps_callback(data):
    global lat, long, deg
    cdata = data.data
    lat = cdata[0]
    long = cdata[1]
    deg = cdata[2]

def main():
    global lat, long, deg

    rclpy.init()
    gps_node = rclpy.create_node("pos_sub")

    gps_sub = gps_node.create_subscription(
        PoseStamped,
        "/zed/zed_node/pose",
        gps_callback,
        1
    )

        

    thread = threading.Thread(target=rclpy.spin, args=(gps_node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        try: 
            bs = str(ser.readline().decode("utf-8"))
            print(repr(bs))

            if (bs == "gps_request\r\n"):
                print("GPS REQUESTED")
                ser.write(str(lat).encode("utf-8"))
                ser.write(",".encode("utf-8"))
                ser.write(str(long).encode("utf-8"))
        except serial.SerialException or UnicodeDecodeError:
            continue
    
    gps_node.destroy_node()
    rclpy.shutdown()

main()
