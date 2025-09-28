import json
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from path_planner import PathPlanner

class NavigationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Navigation Network Visualizer")
        self.root.geometry("1200x800")
        
        # Data storage
        self.planner = None
        
        # GUI state
        self.selected_segments = set()
        self.start_point = None
        self.end_point = None
        self.current_path = []
        
        self.setup_gui()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel for controls
        control_frame = ttk.Frame(main_frame, width=300)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        control_frame.pack_propagate(False)
        
        # File loading
        file_frame = ttk.LabelFrame(control_frame, text="File Loading")
        file_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(file_frame, text="Load JSON File", command=self.load_file).pack(pady=5)
        
        # Navigation controls
        nav_frame = ttk.LabelFrame(control_frame, text="Navigation")
        nav_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(nav_frame, text="Start X (meters):").pack(anchor=tk.W)
        self.start_x = ttk.Entry(nav_frame)
        self.start_x.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(nav_frame, text="Start Y (meters):").pack(anchor=tk.W)
        self.start_y = ttk.Entry(nav_frame)
        self.start_y.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(nav_frame, text="Destination (Node/Point ID):").pack(anchor=tk.W)
        self.dest_entry = ttk.Entry(nav_frame)
        self.dest_entry.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Button(nav_frame, text="Find Route", command=self.find_route).pack(pady=5)
        ttk.Button(nav_frame, text="Clear Route", command=self.clear_route).pack()
        
        # Segment filtering
        segment_frame = ttk.LabelFrame(control_frame, text="Segment Filtering")
        segment_frame.pack(fill=tk.X, pady=(0, 5))
        
        ttk.Label(segment_frame, text="Show Segments:").pack(anchor=tk.W)
        self.segment_var = tk.StringVar()
        self.segment_entry = ttk.Entry(segment_frame, textvariable=self.segment_var)
        self.segment_entry.pack(fill=tk.X, pady=(0, 5))
        self.segment_entry.bind('<Return>', self.update_segment_filter)
        
        ttk.Button(segment_frame, text="Apply Filter", command=self.update_segment_filter).pack(pady=2)
        ttk.Button(segment_frame, text="Show All", command=self.show_all_segments).pack()
        
        # Point information
        info_frame = ttk.LabelFrame(control_frame, text="Point Information")
        info_frame.pack(fill=tk.BOTH, expand=True)
        
        self.info_text = tk.Text(info_frame, height=10)
        self.info_text.pack(fill=tk.BOTH, expand=True, pady=5)
        
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.info_text.config(yscrollcommand=scrollbar.set)
        
        # Plot area
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Bind click event
        self.canvas.mpl_connect('button_press_event', self.on_plot_click)
        
    def load_file(self):
        file_path = filedialog.askopenfilename(
            title="Select JSON file",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                self.planner = PathPlanner(json_file_path=file_path)
                self.selected_segments = self.planner.get_segments()
                self.segment_var.set(','.join(map(str, sorted(self.selected_segments))))
                self.plot_network()
                messagebox.showinfo("Success", "File loaded successfully!")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load file: {str(e)}")
    
    def find_route(self):
        """Find route from start coordinates to destination"""
        if not self.planner:
            messagebox.showerror("Error", "Please load a JSON file first")
            return
            
        try:
            start_x = float(self.start_x.get())
            start_y = float(self.start_y.get())
            dest_id = self.dest_entry.get()
            
            # Parse destination
            try:
                dest_id = int(dest_id)
            except:
                messagebox.showerror("Error", "Destination must be a valid point/node ID")
                return
            
            # Plan the path using PathPlanner
            result = self.planner.plan_path(start_x, start_y, dest_id)
            
            if result['success']:
                self.current_path = result['path']
                self.start_point = (start_x, start_y)
                self.end_point = dest_id
                
                self.plot_network()
                
                # Show route info
                self.info_text.delete(1.0, tk.END)
                self.info_text.insert(tk.END, f"Route found!\n")
                self.info_text.insert(tk.END, f"Start: ({start_x:.1f}m, {start_y:.1f}m)\n")
                self.info_text.insert(tk.END, f"Closest start point: {result['start_point_id']}\n")
                self.info_text.insert(tk.END, f"Destination: {dest_id}\n")
                self.info_text.insert(tk.END, f"Path length: {len(result['path'])} points\n")
                self.info_text.insert(tk.END, f"Total distance: {result['distance']:.1f} meters\n")
                self.info_text.insert(tk.END, f"Path: {' -> '.join(map(str, result['path']))}\n")
            else:
                messagebox.showerror("Error", result['message'])
                
        except ValueError as e:
            if "not found in the network" in str(e):
                messagebox.showerror("Error", str(e))
            else:
                messagebox.showerror("Error", "Please enter valid coordinates")
    
    def clear_route(self):
        """Clear the current route"""
        self.current_path = []
        self.start_point = None
        self.end_point = None
        self.plot_network()
    
    def update_segment_filter(self, event=None):
        """Update which segments to display"""
        if not self.planner:
            return
            
        segment_str = self.segment_var.get()
        try:
            if segment_str.strip():
                segments = [int(s.strip()) for s in segment_str.split(',')]
                self.selected_segments = set(segments)
            else:
                self.selected_segments = self.planner.get_segments()
            self.plot_network()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid segment numbers separated by commas")
    
    def show_all_segments(self):
        """Show all segments"""
        if not self.planner:
            return
            
        self.selected_segments = self.planner.get_segments()
        self.segment_var.set(','.join(map(str, sorted(self.selected_segments))))
        self.plot_network()
    
    def plot_network(self):
        """Plot the network on the matplotlib canvas"""
        if not self.planner:
            return
        
        self.ax.clear()
        
        # Plot points
        for point_id, point_data in self.planner.points.items():
            if point_data['segment_num'] in self.selected_segments:
                color = 'blue'
                size = 20
                
                # Highlight path points
                if point_id in self.current_path:
                    color = 'red'
                    size = 30
                
                self.ax.scatter(point_data['x'], point_data['y'], 
                              c=color, s=size, alpha=0.7, picker=True)
        
        # Plot nodes
        for node_id, node_data in self.planner.nodes.items():
            color = 'green'
            size = 100
            
            # Highlight if in path
            if node_id in self.current_path:
                color = 'red'
                size = 150
            
            self.ax.scatter(node_data['x'], node_data['y'], 
                          c=color, s=size, marker='s', alpha=0.8, picker=True)
            
            # Label nodes
            self.ax.annotate(node_data['name'], 
                           (node_data['x'], node_data['y']),
                           xytext=(5, 5), textcoords='offset points',
                           fontsize=8, fontweight='bold')
        
        # Plot connections for visible segments
        for point_id, point_data in self.planner.points.items():
            if point_data['segment_num'] in self.selected_segments:
                for linked_id in point_data['linked_points']:
                    if linked_id in self.planner.points and self.planner.points[linked_id]['segment_num'] in self.selected_segments:
                        other_point = self.planner.points[linked_id]
                    elif linked_id in self.planner.nodes:
                        other_point = self.planner.nodes[linked_id]
                    else:
                        continue
                    
                    # Draw connection
                    alpha = 0.3
                    color = 'gray'
                    linewidth = 1
                    
                    # Highlight path connections
                    if (point_id in self.current_path and linked_id in self.current_path and
                        abs(self.current_path.index(point_id) - self.current_path.index(linked_id)) == 1):
                        color = 'red'
                        linewidth = 3
                        alpha = 0.8
                    
                    self.ax.plot([point_data['x'], other_point['x']], 
                               [point_data['y'], other_point['y']], 
                               color=color, alpha=alpha, linewidth=linewidth)
        
        # Plot start point if exists
        if self.start_point:
            self.ax.scatter(self.start_point[0], self.start_point[1], 
                          c='orange', s=200, marker='*', 
                          edgecolors='black', linewidth=2, label='Start')
        
        # Plot origin point
        if self.planner and self.planner.origin_lat is not None:
            self.ax.scatter(0, 0, c='purple', s=150, marker='X', 
                          edgecolors='black', linewidth=2, label='Origin')
        
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Navigation Network (Metric Coordinates)')
        self.ax.grid(True, alpha=0.3)
        self.ax.axis('equal')
        
        if self.start_point or (self.planner and self.planner.origin_lat is not None):
            self.ax.legend()
        
        self.canvas.draw()
    
    def on_plot_click(self, event):
        """Handle clicking on the plot"""
        if event.inaxes != self.ax or not self.planner:
            return
        
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        
        # Find closest point/node to click (using X/Y coordinates in meters)
        closest_id = self.planner.find_closest_point(x, y)
        
        if closest_id is not None:
            self.show_point_info(closest_id)
    
    def show_point_info(self, point_id):
        """Display information about a point/node"""
        if not self.planner:
            return
            
        self.info_text.delete(1.0, tk.END)
        
        point_info = self.planner.get_point_info(point_id)
        
        if point_info:
            if point_info['type'] == 'point':
                self.info_text.insert(tk.END, f"POINT ID: {point_id}\n")
                self.info_text.insert(tk.END, f"Type: Point\n")
                self.info_text.insert(tk.END, f"Coordinates (X,Y): ({point_info['x']:.1f}m, {point_info['y']:.1f}m)\n")
                self.info_text.insert(tk.END, f"Coordinates (Lat,Lon): ({point_info['lat']:.6f}, {point_info['lon']:.6f})\n")
                self.info_text.insert(tk.END, f"Segment: {point_info['segment_num']}\n")
                self.info_text.insert(tk.END, f"Linked Points: {point_info['linked_points']}\n")
                
            elif point_info['type'] == 'node':
                self.info_text.insert(tk.END, f"NODE ID: {point_id}\n")
                self.info_text.insert(tk.END, f"Type: Node\n")
                self.info_text.insert(tk.END, f"Name: {point_info['name']}\n")
                self.info_text.insert(tk.END, f"Coordinates (X,Y): ({point_info['x']:.1f}m, {point_info['y']:.1f}m)\n")
                self.info_text.insert(tk.END, f"Coordinates (Lat,Lon): ({point_info['lat']:.6f}, {point_info['lon']:.6f})\n")
                
                # Show connected points
                connected = [neighbor for neighbor, _ in self.planner.graph[point_id]]
                self.info_text.insert(tk.END, f"Connected to: {connected}\n")

def main():
    root = tk.Tk()
    app = NavigationGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()