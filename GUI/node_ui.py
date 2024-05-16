
# from coords_to_cartesian import CoordsToCartesian
import threading 
import sys
from geopy import distance
import math

SHOW_GUI = False


class UI:
    # Adding markers
    def add_marker_event(self, coords, label):

        # Getting the latitude and longitude from the coordinates
        lat = coords[0]
        lon = coords[1]

        # Creating a new marker at that lat lon
        new_marker = self.map_widget.set_marker(lat, lon, text=label, font=("helvetica-bold", 40), text_color="red", marker_color_outside="red")

    # Set a path
    def path_maker(self, route_coords):
        if len(route_coords) > 1:
            path = self.map_widget.set_path(route_coords,width = 15)

    # Get inputs
    def get_inputs(self):
        
        # All nodes
        nodes = ["EB1", "EB2", "EB3", "FW", "MID", "HUNT", "OVAL"]

        # Getting the user input
        start = int(input("Where are you starting from?\n1. EB1\n2. EB2\n3. EB3\n4. Fitts Woolard\n5. Mid\n6. Hunt Library\n7. The Oval\n")) - 1
        print("Starting: ", nodes[start])
        goal = int(input("Where are you starting from?\n1. EB1\n2. EB2\n3. EB3\n4. Fitts Woolard\n5. Mid\n6. Hunt Library\n7. The Oval\n")) - 1
        print("Goal: ", goal)

        # Making a path
        path = Dijkstra(nodes[start], nodes[goal])

        # Printing the path
        print("Shortest route: ", path.get_shortest_path())

        return path.get_shortest_path()

    def get_route_coords(self, route):

        coord_route = []
        for i in range(len(route) - 1):
            if route[i] == "EB1":
                if route[i + 1] == "EB2":
                    coord_route.append(self.EB1toEB2)
                if route[i + 1] == "FW":
                    coord_route.append(self.EB1toFW)
                if route[i + 1] == "EB3":
                    coord_route.append(self.EB1toEB3)
            if route[i] == "EB2":
                if route[i + 1] == "EB1":
                    coord_route.append(self.EB1toEB2[::-1])
                if route[i + 1] == "EB3":
                    coord_route.append(self.EB3toEB2[::-1])
            if route[i] == "EB3":
                if route[i + 1] == "EB2":
                    coord_route.append(self.EB3toEB2)
                if route[i + 1] == "EB1":
                    coord_route.append(self.EB1toEB3[::-1])
                if route[i + 1] == "MID":
                    coord_route.append(self.EB3toMID)
            if route[i] == "FW":
                if route[i + 1] == "EB1":
                    coord_route.append(self.EB1toFW[::-1])
                if route[i + 1] == "MID":
                    coord_route.append(self.FWtoMID)
                if route[i + 1] == "HUNT":
                    coord_route.append(self.FWtoHUNT)
            if route[i] == "MID":
                if route[i + 1] == "EB3":
                    coord_route.append(self.EB3toMID[::-1])
                if route[i + 1] == "FW":
                    coord_route.append(self.FWtoMID[::-1])
                if route[i + 1] == "OVAL":
                    coord_route.append(self.OVALtoMID[::-1])
            if route[i] == "OVAL":
                if route[i + 1] == "MID":
                    coord_route.append(self.OVALtoMID)
                if route[i + 1] == "HUNT":
                    coord_route.append(self.OVALtoHUNT)
            if route[i] == "HUNT":
                if route[i + 1] == "FW":
                    coord_route.append(self.FWtoHUNT[::-1])
                if route[i + 1] == "OVAL":
                    coord_route.append(self.OVALtoHUNT[::-1])
        return coord_route
            
    def gui(self):
        import customtkinter
        import tkintermapview

        # create tkinter window
        root_tk = customtkinter.CTk()
        root_tk.geometry(f"{1000}x{700}")
        root_tk.title("Centennial Map")

        # create map widget
        self.map_widget = tkintermapview.TkinterMapView(root_tk, width=1000, height=700, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

        # set other tile server (standard is OpenStreetMap)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite

        # set current position and zoom
        self.map_widget.set_position(35.770743,-78.674817, marker=False)  # Chapel Hill, NC
        self.map_widget.set_zoom(19.8)

        # set current position with address
        self.map_widget.set_address("Starting Location", marker=True)

        self.add_marker_event(self.route_coords[0], "Start")
        self.add_marker_event(self.route_coords[len(self.route_coords) - 1], "End")

        self.path_maker(self.route_coords)

        # Loop through the tkinter page
        root_tk.mainloop()

    def ros2_node(self, args=None):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray

        # Initializing the ros2 node
        rclpy.init(args=args)

        # Creating the node
        node = Node("node_ui")

        # Create a publisher to another node
        # Parameters of create_publisher(message data type, topic name, method callback, priority)
        pub_sample_message = node.create_publisher(Float64MultiArray, 'waypoints', 1)

        # Threading loop that automatically spins the ros2 node
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        # While ros2 node is ok
        while rclpy.ok():

            # Make an integer message
            m = Float64MultiArray()
            
            list_coords = []
            # Attaching the global message to the m.data
            # for i in range(len(self.route_cart)):
            #     list_coords.append(self.route_cart[i][0])
            #     list_coords.append(self.route_cart[i][1])

            for i in range(len(self.route_coords)):
                list_coords.append(self.route_coords[i][0])
                list_coords.append(self.route_coords[i][1])
                
            m.data = list_coords

            # Publishing the message
            pub_sample_message.publish(m)

        # Need to attach at the end to make sure the node is spinning and shutsdown properly.
        rclpy.spin(node)
        rclpy.shutdown()

    def main(self):

        # Getting user inputs before showing display
        route = self.get_inputs()

        # Routes between nodes
        # self.EB1toEB2 = [[22.3406, 21.2067],[22.2772, 19.8043],[25.5805, 21.3573],[24.888, 15.0913],[22.787, 11.1301],[19.1663, 6.2687],[14.8495, 2.7833],[9.4469, 0.5608],[4.2184, -0.0337],[2.9948, 0.0206],[0.0,0.0]]
        self.EB1toEB2 = [[1.23, 2.068], [1.179, 1.2414], [1.009, 0.6513], [0.6565, 0.2573], [0.0,0.0]]
        self.EB3toEB2 = [[-22.3406, 21.2067],[-22.2772, 19.8043],[-25.5805, 21.3573],[-24.888, 15.0913],[-22.787, 11.1301],[-19.1663, 6.2687],[-14.8495, 2.7833],[-9.4469, 0.5608],[-4.2184, -0.0337],[-2.9948, 0.0206],[0.0,0.0]]
        #self.EB1toEB2 = [[-2.1167, 1.981], [-2.1778, 2.7286], [-2.1228, 3.1375], [-1.6718, 3.8687], [-0.7452, 4.2302]]
        
        # self.EB1toEB2 = [[35.771549, -78.674697],[35.771598, -78.674656],[35.771687, -78.674549],[35.771715, -78.674491],[35.771724, -78.674432],[35.771726, -78.674371],[35.771728, -78.674302],[35.771704, -78.674220],[35.771678, -78.674161],[35.771654, -78.674104]]
        self.EB1toEB3 = [[35.771549, -78.674697],[35.771473, -78.674549],[35.771429, -78.674461],[35.771336, -78.674278],[35.771242, -78.674096],[35.771194, -78.674005],[35.771166, -78.673954]]
        self.EB1toFW = [[35.771549, -78.674697],[35.771484, -78.674745],[35.771405, -78.674809],[35.771334, -78.674866],[35.771260, -78.674927],[35.771170, -78.674986],[35.771105, -78.675043],[35.771022, -78.675102],[35.770953, -78.675150]]
        #self.EB3toEB2 = [[35.771166, -78.673954],[35.771231, -78.673913],[35.771294, -78.673878],[35.771364, -78.673867],[35.771433, -78.673873],[35.771520, -78.673902],[35.771566, -78.673945],[35.771612, -78.674015],[35.771654, -78.674104]]
        self.EB3toMID = [[35.771166, -78.673954],[35.771107, -78.674007],[35.771039, -78.674061],[35.770967, -78.674122],[35.770898, -78.674173],[35.770835, -78.674224],[35.770772, -78.674275],[35.770693, -78.674334],[35.770632, -78.674380],[35.770576, -78.674423]]
        self.FWtoMID = [[35.770953, -78.675150],[35.770917, -78.675074],[35.770856, -78.674962],[35.770800, -78.674852],[35.770746, -78.674742],[35.770682, -78.674624],[35.770624, -78.674508],[35.770576, -78.674423]]
        self.FWtoHUNT = [[35.770953, -78.675150],[35.770909, -78.675190],[35.770828, -78.675254],[35.770754, -78.675313],[35.770658, -78.675388],[35.770576, -78.675453],[35.770456, -78.675538],[35.770339, -78.675624],[35.770265, -78.675686],[35.770162, -78.675769],[35.770082, -78.675828],[35.770012, -78.675884],[35.769914, -78.675925]]
        self.OVALtoMID = [[35.770004, -78.674873],[35.770104, -78.674793],[35.770195, -78.674715],[35.770263, -78.674669],[35.770348, -78.674605],[35.770429, -78.674538],[35.770507, -78.674479],[35.770576, -78.674423]]
        self.OVALtoHUNT = [[35.770004, -78.674873],[35.770000, -78.674940],[35.769996, -78.675034],[35.769987, -78.675155],[35.769983, -78.675262],[35.769965, -78.675364],[35.769959, -78.675474],[35.769948, -78.675594],[35.769941, -78.675723],[35.769926, -78.675833],[35.769914, -78.675925]]

        # Initializing the coordinate system
        origin = [35.771650, -78.674103]
        pointY = [35.76926314060246, -78.67596498545836]
        pointX = [35.77257728014708, -78.67586909648608]

        # Making the coordinate system plane
        plane = CoordsToCartesian(origin, pointX, pointY)

        # Take the string of each node we need to reach and convert that to lat longs to make a route
        self.rough_route_coords = self.get_route_coords(route)

        # Forms of the route as cartesian and coordinates (lat/long)
        self.route_cart = []
        self.route_coords = []

        # Converting all the coordinates in the route to cartesian
        for i in range(len(self.rough_route_coords)):
            for j in range(len(self.rough_route_coords[i])):
                self.route_coords.append(self.rough_route_coords[i][j])
                self.route_cart.append(plane.get_cartesian(self.rough_route_coords[i][j]))
        
        # print(self.route_coords)
        # print(self.route_cart)

        if not SHOW_GUI:
            self.ros2_node()

        if SHOW_GUI:
            self.gui()

class Dijkstra(object):
    def __init__(self, start_node, target_node):
        nodes = ["EB2", "EB1", "EB3", "FW", "HUNT", "MID", "OVAL"]

        init_graph = {}
        for node in nodes:
            init_graph[node] = {}
            
        init_graph["EB2"]["EB1"] = 62.7
        init_graph["EB2"]["EB3"] = 64.17
        init_graph["EB1"]["FW"] = 77.6
        init_graph["EB1"]["EB3"] = 77.9
        init_graph["EB3"]["MID"] = 77.6
        init_graph["FW"]["MID"] = 77.9
        init_graph["FW"]["HUNT"] = 135.36
        init_graph["HUNT"]["OVAL"] = 90
        init_graph["OVAL"]["MID"] = 75.96

        graph = Graph( nodes, init_graph )
        previous_nodes, shortest_path = self.dijkstra_algorithm(graph, start_node)
        self.print_result(previous_nodes, shortest_path, start_node, target_node)


    def dijkstra_algorithm(self, graph, start_node):
        unvisited_nodes = list(graph.get_nodes())
    
        # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
        shortest_path = {}
    
        # We'll use this dict to save the shortest known path to a node found so far
        previous_nodes = {}
    
        # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
        max_value = sys.maxsize
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        # However, we initialize the starting node's value with 0   
        shortest_path[start_node] = 0
        
        # The algorithm executes until we visit all nodes
        while unvisited_nodes:
            # The code block below finds the node with the lowest score
            current_min_node = None
            for node in unvisited_nodes: # Iterate over the nodes
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
                    
            # The code block below retrieves the current node's neighbors and updates their distances
            neighbors = graph.get_outgoing_edges(current_min_node)
            for neighbor in neighbors:
                tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
                if tentative_value < shortest_path[neighbor]:
                    shortest_path[neighbor] = tentative_value
                    # We also update the best path to the current node
                    previous_nodes[neighbor] = current_min_node
    
            # After visiting its neighbors, we mark the node as "visited"
            unvisited_nodes.remove(current_min_node)
        
        return previous_nodes, shortest_path


    def print_result(self, previous_nodes, shortest_path, start_node, target_node):
        path = []
        node = target_node
        
        while node != start_node:
            path.append(node)
            node = previous_nodes[node]
    
        # Add the start node manually
        path.append(start_node)
        self.path = list(reversed(path))

    def get_shortest_path(self):
        return self.path
    
class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph):
        '''
        This method makes sure that the graph is symmetrical. In other words, if there's a path from node A to B with a value V, there needs to be a path from node B to node A with a value V.
        '''
        graph = {}
        for node in nodes:
            graph[node] = {}
        
        graph.update(init_graph)
        
        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
                    
        return graph
    
    def get_nodes(self):
        "Returns the nodes of the graph."
        return self.nodes
    
    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
    
    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.graph[node1][node2]
    
class CoordsToCartesian(object):

    def __init__(self, origin, referenceX, referenceY):

        # Initializing the coordinate system with lat long for origin, reference point on x axis and reference point on y axis
        self.OX = distance.distance(origin, referenceX).meters
        self.OY = distance.distance(origin, referenceY).meters
        self.referenceX = referenceX
        self.referenceY = referenceY
        self.origin = origin

    def polar_to_cartesian(self, radius, theta):
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        return x, y
    
    def cartesian_distance(self, x1, y1, x2, y2):
        dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
        return dist

    def get_cartesian(self, coords):

        # Getting the distances between coordinates
        OP = distance.distance(self.origin, coords).meters
        XP = distance.distance(coords, self.referenceX).meters
        YP = distance.distance(self.referenceY, coords).meters

        # Initializing the markerPoint (In cartesian form)
        markerPoint = []

        # Finding the angle between the x axis and the vector OP
        theta = math.acos((-(XP)**2 + self.OX**2 + OP**2)/(2*self.OX*OP))

        # Finding the points based on the distance from origin to point and theta found
        x, y = self.polar_to_cartesian(OP, theta)

        # Checks if theta is supposed to be positive or negative by checking if the distance from y to the point is the same as the expected y to point distance
        if abs(self.cartesian_distance(x, y, 0, self.OY) - YP) <= 50:
            markerPoint = [x, y]
        else:
            x, y = self.polar_to_cartesian(OP, -theta)
            markerPoint = [x, y]

        return markerPoint
                


if __name__ == "__main__":
    ui = UI()
    ui.main()
