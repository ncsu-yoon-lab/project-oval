from kivy_garden.mapview import MapView, MapMarker
from kivy.network.urlrequest import UrlRequest
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.button import Button
from kivy_garden.graph import Graph, MeshLinePlot
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.uix.checkbox import CheckBox
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from kivy.uix.dropdown import DropDown
from kivy.uix.image import Image
from kivy.uix.spinner import Spinner
from kivy.graphics import *
from kivy.core.window import Window

import requests
import random
import os
import csv
import sys
import math
from geopy import distance

Builder.load_file('design.kv')

class StartScreen(Screen):

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        pass

class ResearchScreen(Screen):

    # Initializing lat, long, and zoom
    zoom = 19
    lat = 35.770849
    long = -78.674706

    # File path
    file_path = 'C:\\Users\\malin\\Documents\\GitHub\\wolfwagen\\Server\\logged_data.csv'

    # The latitude and longitude points between each route
    EB1toEB2 = [[35.771549, -78.674697],[35.771598, -78.674656],[35.771687, -78.674549],[35.771715, -78.674491],[35.771724, -78.674432],[35.771726, -78.674371],[35.771728, -78.674302],[35.771704, -78.674220],[35.771678, -78.674161],[35.771654, -78.674104]]
    EB1toEB3 = [[35.771549, -78.674697],[35.771473, -78.674549],[35.771429, -78.674461],[35.771336, -78.674278],[35.771242, -78.674096],[35.771194, -78.674005],[35.771166, -78.673954]]
    EB1toFW = [[35.771549, -78.674697],[35.771484, -78.674745],[35.771405, -78.674809],[35.771334, -78.674866],[35.771260, -78.674927],[35.771170, -78.674986],[35.771105, -78.675043],[35.771022, -78.675102],[35.770953, -78.675150]]
    EB3toEB2 = [[35.771166, -78.673954],[35.771231, -78.673913],[35.771294, -78.673878],[35.771364, -78.673867],[35.771433, -78.673873],[35.771520, -78.673902],[35.771566, -78.673945],[35.771612, -78.674015],[35.771654, -78.674104]]
    EB3toMID = [[35.771166, -78.673954],[35.771107, -78.674007],[35.771039, -78.674061],[35.770967, -78.674122],[35.770898, -78.674173],[35.770835, -78.674224],[35.770772, -78.674275],[35.770693, -78.674334],[35.770632, -78.674380],[35.770576, -78.674423]]
    FWtoMID = [[35.770953, -78.675150],[35.770917, -78.675074],[35.770856, -78.674962],[35.770800, -78.674852],[35.770746, -78.674742],[35.770682, -78.674624],[35.770624, -78.674508],[35.770576, -78.674423]]
    FWtoHUNT = [[35.770953, -78.675150],[35.770909, -78.675190],[35.770828, -78.675254],[35.770754, -78.675313],[35.770658, -78.675388],[35.770576, -78.675453],[35.770456, -78.675538],[35.770339, -78.675624],[35.770265, -78.675686],[35.770162, -78.675769],[35.770082, -78.675828],[35.770012, -78.675884],[35.769914, -78.675925]]
    OVALtoMID = [[35.770004, -78.674873],[35.770104, -78.674793],[35.770195, -78.674715],[35.770263, -78.674669],[35.770348, -78.674605],[35.770429, -78.674538],[35.770507, -78.674479],[35.770576, -78.674423]]
    OVALtoHUNT = [[35.770004, -78.674873],[35.770000, -78.674940],[35.769996, -78.675034],[35.769987, -78.675155],[35.769983, -78.675262],[35.769965, -78.675364],[35.769959, -78.675474],[35.769948, -78.675594],[35.769941, -78.675723],[35.769926, -78.675833],[35.769914, -78.675925]]

    routes = (EB1toEB2, EB1toEB3, EB1toFW, EB3toEB2, EB3toMID, FWtoMID, FWtoHUNT, OVALtoMID, OVALtoHUNT)

    debug_mode = True

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Initialize the mapview
        self.mapview = MapView(zoom = self.zoom, lat = self.lat, lon = self.long)

        # Creating a plot for tracking the data
        self.graphview = Graph(xlabel='X', ylabel='Y', x_ticks_minor=5, x_ticks_major=25, y_ticks_minor=1,
                      y_ticks_major=10, y_grid_label=True, x_grid_label=True, padding=5,
                      xlog=False, ylog=False, xmin=-0, xmax=100, ymin=-1, ymax=1)
        plot = MeshLinePlot(color=[1, 0, 0, 1])
        plot.points = [(x, 0.5 * x * 0.01) for x in range(0, 101)]
        self.graphview.add_plot(plot)

        # Add the map to the widget
        map = self.ids.research_widget
        map.add_widget(self.mapview)

        # Initialize each of the check boxes initialized
        self.rtk_check = self.ids.rtk_cb.active
        self.gps_check = self.ids.gps_cb.active
        self.ml_check = self.ids.ml_cb.active

        # Initialize the lists for each 1D list of data
        self.rtk_data = []
        self.gps_data = []
        self.ml_data = []
        self.times = []

        # Initialize the lists for each 1D list of statistics
        # Percent error
        self.gps_pe = []
        self.ml_pe = []

        # Markers added to the map
        self.markers = []

        # Initializes if the two routes are already generated
        self.routes_found = False

        # Initializes the past, current, and next routes
        self.current_route = self.next_route = self.full_route = None

        # Create the research widget that holds either the plot or the map
        self.research_widget = self.ids.research_widget

        # Set the initial switch to 0
        self.widget_switch_state = 0
        self.log_switch_state = 0
        self.route_switch_state = 0

        # Constant loop of every 1 second
        Clock.schedule_interval(self.main_loop, 1)

    # Main loop that is repeats every second
    def main_loop(self, instance):

        # Receive the current data from the web server
        if self.debug_mode:
            self.current_lat, self.current_lon = 35.771260, -78.674927
        else:
            self.current_lat, self.current_lon = self.receive_data()

        self.calculate_stats()

        # Log data if it is turned on
        if self.log_switch_state == 1:
            self.log_data()

        # Clears all markers before updating the current location of the vehicle
        self.clear_markers()

        # Adds the current location of the vehicle
        self.add_marker(self.current_lat, self.current_lon, False)
        
        # Constantly update the route on the map if needed
        if self.route_switch_state:
            self.generate_route()

    # Calculate the statistics of the data
    def calculate_stats(self):

        # Calculate the percent difference between the points
        if len(self.gps_data) > 0 and len(self.ml_data) > 0 and len(self.rtk_data) > 0:
            self.gps_pe[len(self.gps_pe)] = self.percent_error(self.rtk_data[len(self.rtk_data) - 1], self.gps_data[len(self.gps_data) - 1])
            self.ml_pe[len(self.ml_pe)] = self.percent_error(self.rtk_data[len(self.rtk_data) - 1], self.ml_data[len(self.ml_data) - 1])

    # Calculates the percent error between expected and measured values
    def percent_error(self, expected, measured):
        pe = abs(float(expected - measured)) / float(measured)
        return pe

    # Logs the data to a csv
    def log_data(self):

        # Combine all the data into a 2D array
        data = [self.times, self.rtk_data, self.gps_data, self.ml_data, self.gps_pe, self.ml_data]

        # Writes the data to the csv of the file path and overwrites it if it already exists
        with open(self.file_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(data)

    # Clears all the markers on the map
    def clear_markers(self):
        for marker in self.markers:
            self.mapview.remove_marker(marker)
    
    # Creates a marker when given a lat and long
    def add_marker(self, latitude, longitude, isRoute):

        # Creating a map marker at the specified lat and long
        if isRoute:
            marker = MapMarker(lat=latitude, lon=longitude, source='Images/red_dot.png')
        else:
            marker = MapMarker(lat=latitude, lon=longitude)

        # Add the marker to the map
        self.markers.append(marker)
        self.mapview.add_widget(marker)

    # Receives data from the server to then be logged to a csv and displayed
    def receive_data(self):

        # URL of the AWS server being used
        url = 'http://3.16.149.178/download/data.csv'

        # Get the response from the url
        response = requests.get(url)

        # Decode the byte string
        csv_content = response.content.decode('utf-8')

        # Split the content by lines
        lines = csv_content.splitlines()

        # Parse the csv to get the times and data 
        data = [line.split(',') for line in lines]
        size = len(self.times)
        if len(data) > 3:
            self.times[size] = data[0].pop()
            self.rtk_data[size] = data[1].pop()
            self.gps_data[size] = data[2].pop()
            self.ml_data[size] = data[3].pop()
        
        return self.rtk_data[size][0], self.rtk_data[size][1]
        
    # When a checkbox is clicked, the data is updated to reflect which checkboxes are currently active
    def checkbox_clicked(self, instance, value):
        self.rtk_check = self.ids.rtk_cb.active
        self.gps_check = self.ids.gps_cb.active
        self.ml_check = self.ids.ml_cb.active

    # Finds the current route that it is on, and finds the next one
    # Checks that when it leaves a route, a new one is added
    # Two routes shown at a time, current and next
    def generate_route(self):
        
        # Checks if this is the first time generating a route and finds the current route it is on if it is
        if self.current_route == None:
            self.current_route = self.get_closest_route()

        # Initializes the shortest distance and closest point
        shortest_distance = sys.maxsize
        closest_point = None

        # Goes through all the points in the current route to find which point on the route is closest to the vehicle
        for i in range(len(self.current_route)):
            d = distance.distance(self.current_route[i], (self.current_lat, self.current_lon))
            if d < shortest_distance:
                shortest_distance = d
                closest_point = i

        # Initializes the full route as the current route from position of closest point to end of route
        self.full_route = self.current_route[closest_point:]
        
        # Checks if the next route has already been created
        if self.next_route is not None:

            # Checks if the closest point on the current route is the beginning of the next route and sets the current route as next route and makes next route None
            if self.current_route[closest_point] == self.next_route[0]:
                self.current_route = self.next_route
                self.next_route = None

        # Checks if the next route was set to None
        if self.next_route == None:

            # Gets the last point on the current_route
            last_point = self.current_route[len(self.current_route) - 1]

            # Initializes the possible routes
            possible_routes = []

            # Goes through the routes and finds the points that are the same as the end of the current route
            for route in self.routes:

                # Makes a reverse of the route
                reverse_route = list(route[::-1])

                # Checks if the current route is not the same as the route or the reverse of the route
                if self.current_route != route and self.current_route != reverse_route:

                    for point in route:

                        # Checks if the point is the same as the last point of the current route
                        if point == last_point:

                            # Checks if the point is at the beginning or end of the list and adds the correct orientation
                            if (point != route[0]):
                                possible_routes.append(reverse_route)
                            else:
                                possible_routes.append(route)
            
            # Randomly chooses the next route from the possible options
            self.next_route = possible_routes[random.randint(0,len(possible_routes) - 1)]
        
        # Updates the full route to include the next route and adjusted current route
        self.full_route += self.next_route
        
        # Goes through all the points 
        for point in self.full_route:
            self.add_marker(point[0], point[1], True)


    # Finds the route that the vehicle is closest to
    def get_closest_route(self):

        # Initializes the min_distance as the largest possible number
        shortest_distance = sys.maxsize

        # Initializes the shortest route and closest point
        closest_route = None

        # Goes through the list of routes and finds the point that is the closest to the current point
        for route in self.routes:
            for point in route:

                # Gets the distance from the point and current position
                d = distance.distance(point, (self.current_lat, self.current_lon))

                # Checks if the distance is less than the current shortest distance
                if d < shortest_distance:
                    closest_route = route
                    shortest_distance = d

        
        return closest_route


    def route_gen_switch(self):
        if self.generating_routes:
            self.generating_routes = False
        else:
            self.generating_routes = True
        
    # Allows the user to switch between views of the current tracked positions 
    # and the statistics between each position tracked (Percent error)
    def switch_widgets(self):
        
        # Get the current widget being displayed
        widget = self.research_widget.children[0]

        # Remove that current widget
        self.research_widget.remove_widget(widget)

        # If the switch state is 0, change it to 1 then display the graph
        # Else change it to 1 then display the map
        if self.widget_switch_state == 0:
            self.widget_switch_state += 1
            self.ids.switch_widget.text = 'Display Map'
            self.research_widget.add_widget(self.graphview)
        else:
            self.widget_switch_state -= 1
            self.ids.switch_widget.text = 'Display Statistics'
            self.research_widget.add_widget(self.mapview)

    # Logs the data of the GPS signals received via communication with ESP
    def switch_log(self):

        # If the log switch state is 1, the data will be logged consistently to a local csv file
        # Else the logging of the data is stopped
        if self.log_switch_state == 0:
            self.log_switch_state += 1
            self.ids.switch_log.text = 'End Logging'
            self.rtk_data = []
            self.gps_data = []
            self.ml_data = []
            self.times = []
            self.gps_pe = []
            self.ml_pe = []

            # Num initialize
            num = 1

            # Continue until a new file path is created
            while os.path.exists(self.file_path):
                self.file_path = self.file_path + str(num)
                num += 1
        else:
            self.log_switch_state -= 1
            self.ids.switch_log.text = 'Begin Logging'

    # Generate a constant set of waypoints for the car to travel and be displayed on the map
    def switch_route_gen(self):

        # If the route switch is 1, it will begin to generate a random route for the 
        if self.route_switch_state == 0:
            self.route_switch_state += 1
            self.ids.switch_route.text = 'End Route Generation'
        else:
            self.route_switch_state -= 1
            self.ids.switch_route.text = 'Begin Route Generation'

class PrototypeScreen(Screen):

    # Initializing lat, long, and zoom
    lat = long = zoom = 19
    lat = 35.770849
    long = -78.674706

    # The latitude and longitude points between each route
    EB1toEB2 = [[35.771549, -78.674697],[35.771598, -78.674656],[35.771687, -78.674549],[35.771715, -78.674491],[35.771724, -78.674432],[35.771726, -78.674371],[35.771728, -78.674302],[35.771704, -78.674220],[35.771678, -78.674161],[35.771654, -78.674104]]
    EB1toEB3 = [[35.771549, -78.674697],[35.771473, -78.674549],[35.771429, -78.674461],[35.771336, -78.674278],[35.771242, -78.674096],[35.771194, -78.674005],[35.771166, -78.673954]]
    EB1toFW = [[35.771549, -78.674697],[35.771484, -78.674745],[35.771405, -78.674809],[35.771334, -78.674866],[35.771260, -78.674927],[35.771170, -78.674986],[35.771105, -78.675043],[35.771022, -78.675102],[35.770953, -78.675150]]
    EB3toEB2 = [[35.771166, -78.673954],[35.771231, -78.673913],[35.771294, -78.673878],[35.771364, -78.673867],[35.771433, -78.673873],[35.771520, -78.673902],[35.771566, -78.673945],[35.771612, -78.674015],[35.771654, -78.674104]]
    EB3toMID = [[35.771166, -78.673954],[35.771107, -78.674007],[35.771039, -78.674061],[35.770967, -78.674122],[35.770898, -78.674173],[35.770835, -78.674224],[35.770772, -78.674275],[35.770693, -78.674334],[35.770632, -78.674380],[35.770576, -78.674423]]
    FWtoMID = [[35.770953, -78.675150],[35.770917, -78.675074],[35.770856, -78.674962],[35.770800, -78.674852],[35.770746, -78.674742],[35.770682, -78.674624],[35.770624, -78.674508],[35.770576, -78.674423]]
    FWtoHUNT = [[35.770953, -78.675150],[35.770909, -78.675190],[35.770828, -78.675254],[35.770754, -78.675313],[35.770658, -78.675388],[35.770576, -78.675453],[35.770456, -78.675538],[35.770339, -78.675624],[35.770265, -78.675686],[35.770162, -78.675769],[35.770082, -78.675828],[35.770012, -78.675884],[35.769914, -78.675925]]
    OVALtoMID = [[35.770004, -78.674873],[35.770104, -78.674793],[35.770195, -78.674715],[35.770263, -78.674669],[35.770348, -78.674605],[35.770429, -78.674538],[35.770507, -78.674479],[35.770576, -78.674423]]
    OVALtoHUNT = [[35.770004, -78.674873],[35.770000, -78.674940],[35.769996, -78.675034],[35.769987, -78.675155],[35.769983, -78.675262],[35.769965, -78.675364],[35.769959, -78.675474],[35.769948, -78.675594],[35.769941, -78.675723],[35.769926, -78.675833],[35.769914, -78.675925]]

    # Initialize the start screen
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Initialize the mapview
        self.mapview = MapView(zoom = self.zoom, lat = self.lat, lon = self.long)
        map = self.ids.map_widget
        map.add_widget(self.mapview)

        self.routes = [self.EB1toEB2, self.EB1toEB3, self.EB1toFW, self.EB3toEB2, self.EB3toMID, self.FWtoMID, self.FWtoHUNT, self.OVALtoMID, self.OVALtoHUNT]

        # Get the current position of each inactive vehicle
        self.vehicles = self.get_vehicle_positions()

        self.markers = []

        # Initializes the selected marker
        self.selected_marker = None

        self.set_markers()

    # Creates a marker when given a lat and long
    def add_marker(self, latitude, longitude, isRoute):

        # Creating a map marker at the specified lat and long
        if isRoute:
            marker = MapMarker(lat=latitude, lon=longitude, source='Images/red_dot.png')
        else:
            marker = MapMarker(lat=latitude, lon=longitude)
            self.markers.append(marker)

        # Add the marker to the map
        self.mapview.add_widget(marker)

    # Clears all the markers on the map
    def clear_markers(self):
        for marker in self.markers:
            self.mapview.remove_marker(marker)

    # Sets the markers up
    def set_markers(self):
        for vehicle in self.vehicles:
            self.add_marker(vehicle[0], vehicle[1], False)

    # Gets the position of each vehicle active
    # Currently randomly generated as points on current paths
    def get_vehicle_positions(self):

        points = []

        random_routes = [random.randint(0,8), random.randint(0,8), random.randint(0,8), random.randint(0,8), random.randint(0,8)]

        for i in random_routes:
            points.append(self.routes[i][random.randint(0, len(self.routes[i]) - 1)])
        
        return points
    
    # Generates a route by finding the closest vehicle and going to the start then to the finish
    def generate_route(self):

        # Gets the children boxes of the start and end 
        start_cb = self.ids.start_cb.children
        end_cb = self.ids.end_cb.children

        # Initializing start and end node
        start_node_coords = None
        end_node_coords = None
        start_node = None
        end_node = None

        # Goes through all the boxes of the start and end to locate which nodes are the start and end selected
        for box in start_cb:
            if box.active:
                start_node_coords = (box.lat, box.lon)
                start_node = box
                break
        
        for box in end_cb:
            if box.active:
                end_node_coords = (box.lat, box.lon)
                end_node = box
                break

        # Check that there is a valid input
        if start_node is not None and end_node is not None:

            # Initialize the closest distance and vehicle
            closest_distance = distance.distance((self.markers[0].lat, self.markers[1].lon), start_node_coords)
            closest_vehicle = self.markers[0]

            # Find the vehicle that is closest to the start
            for vehicle in self.markers:
                d = distance.distance((vehicle.lat, vehicle.lon), start_node_coords)
                if closest_distance > d:
                    closest_distance  = d
                    closest_vehicle = vehicle

            # Re-initializes the closest distance to a node and second closest node
            closest_node = 0
            second_closest_node = 1
            closest_distance = distance.distance((closest_vehicle.lat, closest_vehicle.lon), (start_cb[1].lat, start_cb[1].lon))
            second_closest_distance = distance.distance((closest_vehicle.lat, closest_vehicle.lon), (start_cb[2].lat, start_cb[2].lon))

            # Goes through every node finding the closest and second closest node to find the segment
            for i in range(len(start_cb) - 1):
                lat = start_cb[i + 1].lat
                lon = start_cb[i + 1].lon
                d = distance.distance((lat, lon), (closest_vehicle.lat, closest_vehicle.lon))
                if closest_distance > d:
                    closest_distance = d
                    closest_node = i
                elif second_closest_distance > d > closest_distance:
                    second_closest_distance = d
                    second_closest_node = i

            # Initialize the route list
            route_coords = []
            
            self.clear_markers()

            self.add_marker(closest_vehicle.lat, closest_vehicle.lon, False)

            # Checks if closest or second closest node is closer to the start to find the direction
            distance_from_closest = distance.distance((start_cb[closest_node].lat, start_cb[closest_node].lon), (start_node.lat, start_node.lon))
            distance_from_second_closest = distance.distance((start_cb[closest_node].lat, start_cb[closest_node].lon), (start_node.lat, start_node.lon))
            if distance_from_closest > distance_from_second_closest:

                # Get the route for the shortest path from the 
                path = Dijkstra(start_cb[closest_node + 1].text, start_node.text)
                route = path.get_shortest_path()

                # Take the string of each node we need to reach and convert that to lat longs to make a route
                self.rough_route_coords = self.get_route_coords(route)
            else:
                # Get the route for the shortest path from the 
                print(start_cb[second_closest_node + 1].text)
                print(start_node.text)
                path = Dijkstra(start_cb[second_closest_node + 1].text, start_node.text)
                route = path.get_shortest_path()

                # Take the string of each node we need to reach and convert that to lat longs to make a route
                self.rough_route_coords = self.get_route_coords(route)

            # Putting the route into a list of points from current point to start
            for i in range(len(self.rough_route_coords)):
                for j in range(len(self.rough_route_coords[i])):
                    route_coords.append(self.rough_route_coords[i][j])

            # Adding the path from the start to end
            path = Dijkstra(start_node.text, end_node.text)
            route = path.get_shortest_path()

            self.rough_route_coords = self.get_route_coords(route)

            print(route_coords)

            for i in range(len(self.rough_route_coords)):
                for j in range(len(self.rough_route_coords[i])):
                    route_coords.append(self.rough_route_coords[i][j])

            print(route_coords)

            self.add_points_to_map(route_coords)

    def add_points_to_map(self, coords):
        for i in coords:
            self.add_marker(i[0], i[1], True)
    
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


class GraphClass(object):
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


class WW_MonitoringApp(App):

    def build(self):
        Window.fullscreen = 'auto'
        sm = ScreenManager()
        Window.clearcolor = (230/255, 230/255, 230/255, 1)
        sm.add_widget(StartScreen(name='start_screen'))
        sm.add_widget(ResearchScreen(name='research_screen'))
        sm.add_widget(PrototypeScreen(name='prototype_screen'))

        return sm

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

        graph = GraphClass( nodes, init_graph )
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

if __name__ == '__main__':
    WW_MonitoringApp().run()