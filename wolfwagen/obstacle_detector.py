import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import os
import threading
import math
import numpy as np
# import matplotlib.pyplot as plt

angle = 2*math.pi 

scan_data = None
def scan_callback(data):
    global scan_data
    scan_data = data
	

def main(args=None):
    print("Obstacle detector node")
    rclpy.init(args=args)
    node = Node("Obstacle_detector_node")
    
    subscription_laser_scan = node.create_subscription(LaserScan,'scan', scan_callback, 1)
    pub_min_dist = node.create_publisher(Float64, "lidar_min_dist" , 1)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(20, node.get_clock())

    CAR_WIDTH = 30  # cm

    
    while rclpy.ok():
        
        if scan_data is not None:
            dist_array = np.array(scan_data.ranges, copy=True)
            angle_inc = scan_data.angle_increment
            min_dist = scan_data.range_max
            num_distances = len(dist_array)

            XY_safe = np.zeros((num_distances, 2))
            XY_unsafe = np.zeros((num_distances, 2))
            
            for i, distance in enumerate(dist_array):
                if np.isinf(distance) or distance < scan_data.range_min or distance > scan_data.range_max:
                    continue
                angle = np.pi/2 +  i * angle_inc + scan_data.angle_min

                x = -1 * distance * math.sin(angle) * 100   # cm
                y = distance * math.cos(angle) * 100        # cm

                if (y>0 and math.fabs(x) < CAR_WIDTH/2):
                     XY_unsafe[i,:]=(x,y)
                     if distance < min_dist:
                        min_dist = distance
                else:
                    XY_safe[i,:]=(x,y)                

            # plt.cla()
            # plt.plot(XY_safe[:,0], XY_safe[:,1], ".g")
            # plt.plot(XY_unsafe[:,0], XY_unsafe[:,1], ".r")
            # plt.grid(True)
            # plt.xlim(-500,500)
            # plt.ylim(-500,500)
            # plt.pause(0.001)


            print("lidar_min_dist=", min_dist)
            m = Float64()
            m.data = float(min_dist)    #because type(min_dist) = numpy.float32
            pub_min_dist.publish(m)
            


        rate.sleep()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
	main()
