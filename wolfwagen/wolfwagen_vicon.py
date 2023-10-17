import sys
import time
import math
import rclpy
from rclpy.node import Node
import pyvicon_datastream as pv
from pyvicon_datastream import tools

VICON_TRACKER_IP = "eb2-2235-win00.csc.ncsu.edu"
OBJECT_NAME = "wand"    #it should be wolfwagen_xx

#This will try to connect to the VICON TRACKER
vicontracker = tools.ObjectTracker(VICON_TRACKER_IP)

def periodic_loop():

    # time_ms = int(time.time()*1000.0)
    # print(time_ms)

    raw_data = vicontracker.get_position(OBJECT_NAME)
    if raw_data is False:
        print(f"Cannot find object {OBJECT_NAME}")
        return

    _, _, pos_data = raw_data
    print(f"Raw position data: {pos_data}")
    print("")
    if pos_data != []:
        xyz = pos_data[0][2:5]
        orientation = pos_data[0][7]
        print(f"Position: {xyz}")
        print(f"Orientation Rad/Deg.: {orientation:.4f}/{math.degrees(orientation):.4f}")
    else:
        print("no vicon data!")

    print("----")

def main(args=None):

    if not vicontracker.is_connected:
        sys.exit(0)

    rclpy.init(args=args)
    node = Node("vicon_pos")

    node.create_timer(0.1, periodic_loop)   #10 Hz

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
