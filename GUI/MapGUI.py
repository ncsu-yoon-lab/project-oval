import tkinter
import tkintermapview
from geopy import distance
import math
import csv

# create tkinter window
root_tk = tkinter.Tk()
root_tk.geometry(f"{1000}x{700}")
root_tk.title("map_view_simple_example.py")

# Storing Markers
waypoints = []

# Setting the radius
radius = 12

# create map widget
map_widget = tkintermapview.TkinterMapView(root_tk, width=1000, height=700, corner_radius=0)
map_widget.pack(fill="both", expand=True)

# set other tile server (standard is OpenStreetMap)
map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)  # google satellite

# set current position and zoom
map_widget.set_position(35.770743,-78.674817, marker=False)  # Chapel Hill, NC
# map_widget.set_zoom(17)

# set current position with address
map_widget.set_address("Starting Location", marker=True)

# Initializing the coordinate system
origin = [35.772101, -78.673756]
pointY = [35.76926314060246, -78.67596498545836]
pointX = [35.77099489979461, -78.6714045595808]

# origin = [35.7716591, -78.6740835]
# pointY = [35.76926314060246, -78.67596498545836]
# pointX = [35.7716591, -78.6714045595808]

def polar_to_cartesian(radius, theta):
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    return x, y

def cartesian_distance(x1, y1, x2, y2):
    dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
    return dist

# Adding markers
def add_marker_event(coords):
    global waypoints

    # Getting the latitude and longitude from the coordinates
    lat = coords[0]
    lon = coords[1]
    print(lat)
    print(lon)

    # Creating a new marker at that lat lon
    new_marker = map_widget.set_marker(lat, lon)

    # Getting the distances between coordinates
    OX = distance.distance(origin, pointX).meters
    OY = distance.distance(origin, pointY).meters
    OP = distance.distance(origin, coords).meters
    XP = distance.distance(coords, pointX).meters
    YP = distance.distance(pointY, coords).meters

    # Initializing the markerPoint (In cartesian form)
    markerPoint = []

    # Finding the angle between the x axis and the vector OP
    theta = math.acos((-(XP)**2 + OX**2 + OP**2)/(2*OX*OP))

    # Finding the points based on the distance from origin to point and theta found
    x, y = polar_to_cartesian(OP, theta)
    # print(cartesian_distance(x, y, 0, OY), " ", YP)

    # Checks if theta is supposed to be positive or negative by checking if the distance from y to the point is the same as the expected y to point distance
    if abs(cartesian_distance(x, y, 0, OY) - YP) <= 10:
        markerPoint = [x, y]
    else:
        x, y = polar_to_cartesian(OP, -theta)
        markerPoint = [x, y]
    
    waypoints.append([lat , lon])
    path_maker()
    print( "X" , markerPoint[0] )
    print( "Y" , markerPoint[1] )
    
# Left click
map_widget.add_left_click_map_command(add_marker_event)

# set a path
def path_maker():
    global waypoints
    if len(waypoints) > 1:
        path = map_widget.set_path(waypoints)
        print( path )
input_csv_file1 = "gps_data1.csv"
data_array1 = []
with open(input_csv_file1, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
                data_array1.append(row)

for row in data_array1:
    lat = row[0]
    long = row[1]
    add_marker_event([lat, long])

# input_csv_file2 = "gps_data2"
# data_array2 = []
# with open(input_csv_file2, 'r') as csv_file:
#         csv_reader = csv.reader(csv_file)
#         for row in csv_reader:
#                 data_array2.append(row)

root_tk.mainloop()
