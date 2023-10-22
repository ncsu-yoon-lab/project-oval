import sys
import time
import math
import rclpy
from rclpy.node import Node
import pyvicon_datastream as pv
from pyvicon_datastream import tools
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import math
import threading
import tf_transformations
import tf2_ros

VICON_TRACKER_IP = "eb2-2235-win00.csc.ncsu.edu"
OBJECT_NAME = "wolfwagen_01"    #it should be wolfwagen_xx

#This will try to connect to the VICON TRACKER
vicontracker = tools.ObjectTracker(VICON_TRACKER_IP)
pose = None

def quat_from_euler(roll, pitch, yaw):
    q_orig = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    q_rot = tf_transformations.quaternion_from_euler(math.pi, 0, 0)
    q_new = tf_transformations.quaternion_multiply(q_rot, q_orig)

    
    return q_new



def periodic_loop():
    global pose

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

        orientation_angles = pos_data[0][5:]
        q_new = quat_from_euler(orientation_angles[0], orientation_angles[1], orientation_angles[2])
        #type(pose) = geometry_msgs.msg.Pose
        # Pose.orientation.x = q_new[0]
        # Pose.orientation.y = q_new[1]
        # Pose.orientation.z = q_new[2]
        # Pose.orientation.w = q_new[3]

        pose = q_new    
        print(f"Position: {xyz}")
        print(f"Orientation Rad/Deg.: {orientation:.4f}/{math.degrees(orientation):.4f}")

        return xyz, pose
    else:
        print("no vicon data!")

    print("----")

def main(args=None):
    global pose

    if not vicontracker.is_connected:
        sys.exit(0)

    rclpy.init(args=args)
    node = Node("vicon_pos")
    vicon_publisher = node.create_publisher(PoseStamped , 'vicon_pos' , 1)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())

    # node.create_timer(0.1, periodic_loop)   #10 Hz

    while rclpy.ok():
        xyz, pose = periodic_loop()
        data = Pose()
        retVal = PoseStamped()
        data.position.x = xyz[0]
        data.position.y = xyz[1]
        data.position.y = xyz[2]
        data.orientation.x = pose[0]
        data.orientation.y = pose[1]
        data.orientation.z = pose[2]
        data.orientation.w = pose[3]


        retVal.pose = data
        retVal.header.stamp = node.get_clock().now().to_msg() 
        retVal.header.frame_id = "world"    
        vicon_publisher.publish(retVal)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
