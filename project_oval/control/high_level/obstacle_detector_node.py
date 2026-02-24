#!/usr/bin/env python
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time
import math
import numpy as np
import matplotlib.pyplot as plt

THRESHOLD = 4  # m
DIST_THRESH = 1.5  # m

# LiDAR data
scan_data = None
def scan_callback(data):
    global scan_data
    scan_data = data
    
def main():
    rclpy.init()
    node = rclpy.create_node('lidar_view')
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    pub_min_dist = node.create_publisher(Bool, "lidar_od_signal" , 1)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 5
    rate = node.create_rate(FREQ, node.get_clock())

    msg = Bool()
    msg.data = False
    
    # Wait until we receive the first LiDAR message
    while rclpy.ok():    
        if scan_data is None:
            print("Not receiving lidar data")
            rate.sleep()
        else:
            break
    print("Receiving lidar data")
    
    # Main loop
    while rclpy.ok():

        range_array = np.array(scan_data.ranges, copy=True)
        num_distances = len(range_array)
        XY_unsafe = np.zeros((num_distances,2))
        XY_safe = np.zeros((num_distances,2))
        min_dist = scan_data.range_max
        
        for i, distance in enumerate(range_array):
            if np.isinf(distance) or distance < scan_data.range_min:
                continue

            theta = scan_data.angle_min + i * scan_data.angle_increment

            # if (y>0 and math.fabs(x) < THRESHOLD):
            #     XY_unsafe[i,:]=(x,y)
            #     if distance < min_dist:
            #         min_dist = distance
            # else:
            #     XY_safe[i,:]=(x,y)
            
            if -math.pi/12 < theta < math.pi/12:
                if distance < min_dist:
                    min_dist = distance
                

        if (min_dist < DIST_THRESH):
            msg.data = True
        else:
            msg.data = False
        pub_min_dist.publish(msg)
        
        # Plot the LiDAR points (rotated)
        print("Closest Object is at distance: ", min_dist, "m")
        # plt.cla()
        # plt.plot(XY_safe[:,0],XY_safe[:,1], ".g")
        # plt.plot(XY_unsafe[:,0],XY_unsafe[:,1], ".r")
        # plt.grid(True)
        # plt.xlim(-500,500)
        # plt.ylim(-500,500)
        # plt.pause(0.001)        

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
