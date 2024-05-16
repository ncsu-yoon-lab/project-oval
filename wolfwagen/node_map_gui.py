import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from PIL import Image, ImageTk
import tkintermapview
import threading
import numpy as np

class RosNode(Node):
    def __init__(self, gui):
        global publisher_
        super().__init__('gui_node')
        self.gui = gui
        self.subscriber_ = self.create_subscription(Float64MultiArray, 'gps_topic', self.gps_callback, 10)
        publisher_ = self.create_publisher(Float64MultiArray, 'map_topic', 10)

    def send_waypoints(self, waypoints):
        global publisher_
        message = waypoints
        msg = Float64MultiArray()
        msg.data = message
        publisher_.publish(msg)

    def gps_callback(self, msg):
        self.data = msg.data
        self.gui.update_gui(self.data)

class MapGUI:
    def __init__(self, root):
        self.root = root
        self.waypoints = []
        self.path_waypoints = []
        self.radius = 12

        self.map_widget = tkintermapview.TkinterMapView(self.root, width=1000, height=700, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.map_widget.set_position(35.770743,-78.674817, marker=False) 
        self.map_widget.set_address("Starting Location", marker=True)

        self.label_var = tk.StringVar()
        self.label = tk.Label(root, textvariable=self.label_var)
        self.label.pack(pady=10)

    def add_marker_event(self, coords):

        self.waypoints.append(coords[0])

        self.waypoints.append(coords[1])

        self.path_waypoints.append([coords[0], coords[1]])

        self.path_maker()

        RosNode.send_waypoints(self, self.waypoints)

    def path_maker(self):
        if (len(self.path_waypoints) > 1):
            self.map_widget.set_path(self.path_waypoints)

    def update_gui_periodic(self):

        # Perform GUI updates periodically in the main thread
        self.root.after(100, self.update_gui_periodic)

    def update_gui(self, message):
        self.add_marker_event(message)
        # Update waypoints in a thread-safe manner
        self.root.event_generate("<<UpdateGUI>>", when="tail")

def spin_ros(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

def main():
    rclpy.init()

    root = tk.Tk()
    root.geometry(f"{1000}x{700}")
    root.title("Project OVAL")

    gui = MapGUI(root)
    node = RosNode(gui)

    root.node = node  # Attach the node to the Tkinter root for reference

    ros_thread = threading.Thread(target=spin_ros, args=(node,))
    ros_thread.start()

    def update_gui():
        root.update_idletasks()

    root.bind("<<UpdateGUI>>", lambda event: gui.update_gui_periodic())

    root.mainloop()  # Start Tkinter's main loop

    ros_thread.join()  # Wait for the ROS thread to finish

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        quit()