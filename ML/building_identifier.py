import requests
import math

STEP_SIZE = 20 # meters
EARTH_RADIUS = 6371000 # meters

class BuildingIdentifier():
    """This class is used to identify the name of the buildings based on a given position and angle of building sighted"""

    def __init__(self, origin_lat, origin_lon):
        """Initializes the coordinate system and positions"""
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon

        self.current_latitude = self.current_longitude = self.current_heading = None

    def building_finder(self, lat, lon):
        """Uses the latitude and longitude point along with openstreetmap to get the name of the building at that point"""

        # URL for openstreetmap
        url = f"https://nominatim.openstreetmap.org/reverse"

        # The parameters of the website being looked at and the header of the app
        params = {
            'format': 'json',
            'lat': lat,
            'lon': lon,
            'zoom': 18,
            'addressdetails': 1
        }
        headers = {
            'User-Agent': 'BuildingIdentifier'
        }
        
        # Gets the response from the request to the specific url and parameters and headers
        response = requests.get(url, params=params, headers=headers)
        
        # Checks if the response is received correctly
        # Else returns an error
        if response.status_code == 200:

            # Gets the data as the json file of the data
            data = response.json()

            # Gets the address from the data
            address = data.get('address', {})

            # Gets the name of the building or amenity of the address at that point
            building_name = address.get('building')
            amenity_name = address.get('amenity')

            # Checks if the building name is not None and returns that as the name
            # Else if the amenity name is not None and it is not North Carolina State University (which by default is the overlying amenity for the area) returns that name
            # Else returns that there is no recognizable place found
            if building_name is not None:
                return building_name
            elif amenity_name is not None and amenity_name != 'North Carolina State University':
                return amenity_name
            else:
                return 'No recognizable place found'
        else:
            return f"Error {response.status_code}: Unable to fetch data"

    def latlon_to_xy(self, lat, lon):
        """Converts the latitude, longitude point to an x, y point with X axis going North and Y axis going East"""

        # Uses the haversine distance function to get the x, y coordinate
        x = self.haversine_distance(lat, self.origin_lon)
        y = self.haversine_distance(self.origin_lat, lon)
        
        # Checks if x and y are supposed to be negative if they are below the origin
        if lon < self.origin_lon:
            x = -x
        if lat < self.origin_lat:
            y = -y

        return x, y

    def haversine_distance(self, lat, lon):
        """Haversine distance function that calculates the distance x and y position when the latitude and longitude origin and current latitude and longitude"""

        # Converts the latitude and longitude to radians for ease of use
        dlat = math.radians(lat - self.origin_lat)
        dlon = math.radians(lon - self.origin_lon)

        # Calculates the distance the latitude and longitude are from the origin
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(self.origin_lat)) * math.cos(math.radians(lat)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return EARTH_RADIUS * c

    def xy_to_latlon(self, x, y):
        """Converts the x, y point to a latitude, longitude point with X axis going North and Y axis going East"""

        lat = self.origin_lat + (x / EARTH_RADIUS) * (180 / math.pi)
        lon = self.origin_lon + (y / (EARTH_RADIUS * math.cos(math.pi * self.origin_lat / 180))) * (180 / math.pi)

        return lat, lon

    def get_building_name(self, current_latitude, current_longitude, current_heading, building_angle):
        """Main function that gets the name of the building when given a current latitude, longitude, heading, and the angle at which the building is spotted"""

        # Sets the current position
        self.current_latitude = current_latitude
        self.current_longitude = current_longitude
        self.current_heading = current_heading

        # Converts the latitude, longitude point to an x, y point
        current_x, current_y = self.latlon_to_xy(self.current_latitude, self.current_longitude)

        # Gets the direction from the current heading and is rotated based on the angle at which the building is spotted
        direction = self.current_heading + building_angle

        # Initialize variables before while loop
        temp_x, temp_y = current_x, current_y

        # Loops until a building is found or there is an error
        while True:

            # Gets the temporary latitude and longitude from the temporary x and y
            temp_lat, temp_lon = self.xy_to_latlon(temp_x, temp_y)

            # Gets the building name if there is one at that latitude and longitude
            building_name = self.building_finder(temp_lat, temp_lon)

            # Checks if the building name is a recognizable place found and breaks the while loop
            if building_name != 'No recognizable place found':
                break
                
            # Changes the temp x and y by a step size in the direction that it is looking
            temp_x = temp_x + STEP_SIZE * math.cos(math.radians(direction))
            temp_y = temp_y + STEP_SIZE * math.sin(math.radians(direction))

        # Returns the name of the building or an error if there is a problem
        return building_name