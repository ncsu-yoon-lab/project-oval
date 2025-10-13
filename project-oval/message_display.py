import tkinter as tk
import threading
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---------- Color transition utilities ----------
def random_color():
    return (random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255))

def rgb_to_hex(rgb):
    return "#%02x%02x%02x" % rgb

def smooth_transition(start_color, end_color, steps=100, delay=0.02):
    for i in range(steps):
        r = int(start_color[0] + (end_color[0] - start_color[0]) * i / steps)
        g = int(start_color[1] + (end_color[1] - start_color[1]) * i / steps)
        b = int(start_color[2] + (end_color[2] - start_color[2]) * i / steps)
        yield (r, g, b)
        time.sleep(delay)

# ---------- ROS2 Node ----------
class MessageSubscriber(Node):
    def __init__(self, text_var):
        super().__init__('oval_message_listener')
        self.text_var = text_var
        self.subscription = self.create_subscription(
            String,
            '/oval_message',
            self.listener_callback,
            10)
        self.get_logger().info("Listening on topic /oval_message")

    def listener_callback(self, msg):
        # Update GUI message
        self.text_var.set(msg.data)
        self.get_logger().info(f"Updated bottom message: {msg.data}")

# ---------- Background color loop ----------
def color_loop(root, labels):
    current_color = random_color()
    while True:
        next_color = random_color()
        for color in smooth_transition(current_color, next_color):
            hex_color = rgb_to_hex(color)
            root.config(bg=hex_color)
            for lbl in labels:
                lbl.config(bg=hex_color)
            root.update_idletasks()
        current_color = next_color

# ---------- GUI setup ----------
def create_gui():
    root = tk.Tk()
    root.title("NC State OVAL Project")
    root.geometry("900x600")

    # Title
    title_label = tk.Label(
        root, text="NC State OVAL Project",
        font=("Helvetica", 48, "bold"),
        fg="white", bg="black")
    title_label.pack(pady=30)

    # Bottom message (dynamic from ROS)
    message_var = tk.StringVar()
    message_var.set("Waiting for message on /oval_message ...")
    bottom_label = tk.Label(
        root,
        textvariable=message_var,
        font=("Helvetica", 25),
        fg="white",
        bg="black",
        pady=20
    )
    # bottom_label.pack(side="bottom")
    bottom_label.pack()

    # all_labels = [title_label, bottom_label] + student_labels
    all_labels = [title_label, bottom_label] 

    # Start background color animation
    threading.Thread(target=color_loop, args=(root, all_labels), daemon=True).start()

    return root, message_var

# ---------- ROS2 Thread ----------
def ros_thread_func(text_var):
    rclpy.init()
    node = MessageSubscriber(text_var)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

# ---------- Main ----------
if __name__ == '__main__':
    root, message_var = create_gui()

    # Start ROS2 node in a separate thread
    ros_thread = threading.Thread(target=ros_thread_func, args=(message_var,), daemon=True)
    ros_thread.start()

    # Run Tkinter main loop (blocking)
    root.mainloop()