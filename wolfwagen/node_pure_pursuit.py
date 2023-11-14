import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import threading
from rclpy.node import Node
x_pos = y_pos = 0.0
def vicon_callback(data):
    global x_pos , y_pos , z_pos
    # Add your processing logic here

    # "data" has fields for x, y, z, roll, pitch, yaw, and magnitude of rotation
    # Access the fields as follows:
    print("In the callback")
    x_pos = data.pose.position.x
    y_pos = data.pose.position.y
    # z_pos = data.data.data.z
    # data.pose.orientation.x (the roll-value)
    # data.pose.orientation.y (the pitch-value)
    # data.pose.orientation.z (the yaw-value)
    # data.pose.orientation.w (magnitude of rotation)

## Creating a Subscription
def main():
    global x_pos , y_pos , z_pos
    rclpy.init()
    vicon_node = rclpy.create_node('vicon_node')
    subscription = vicon_node.create_subscription(
        PoseStamped,  # Replace PoseStamped with the appropriate message type
        'vicon_pos',  # Specify the topic you want to subscribe to
        vicon_callback,  # Specify the callback function
        1  # queue size
    )

    pure_pursuit_pub = vicon_node.create_publisher(Float64, "pure_pursuit", 1) # publishing one value for now as a test, later change the data type and values


    thread = threading.Thread(target=rclpy.spin, args=(vicon_node, ), daemon=True)
    thread.start()

    while rclpy.ok():
        if x_pos is not None:
            x_data = Float64()
            x_data.data = x_pos
            pure_pursuit_pub.publish(x_data)
            # print(x_pos)
        if y_pos is not None:
            print(y_pos)
        # print(z_pos)




if __name__ == '__main__':
    main()