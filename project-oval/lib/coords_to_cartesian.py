from geopy import distance
import math

class CoordsToCartesian(object):

    # The radius of Earth in meters
    R = 6378137
    
    # origin = (35.7713528, -78.673756)

    def __init__(self, lat_origin, lon_origin):

        # Sets the origin of the coordinate system
        self.origin = (lat_origin, lon_origin)

    def haversine(self, lat, lon):

        # Finds the difference between the lat longs and converts them to radians
        d_lat = math.radians(lat - self.origin[0])
        d_lon = math.radians(lon - self.origin[1])

        a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(self.origin[0])) * math.cos(math.radians(lat)) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = self.R * c

        return distance

    def latlon_to_xy(self, lat, lon):

        x = self.haversine(self.origin[0], lon)
        y = self.haversine(lat, self.origin[1])

        if lon < self.origin[1]:
            x = -x
        if lat < self.origin[0]:
            y = -y

        # Originally returned as x, y (with y as north and x as east) but we want to flip the coordinate system so x is north and y is east to match heading
        # return (x, y)
        return (y, x)
    
    def heading_to_yaw(self, heading) -> float:
        
        # Measured by finding heading pointing the x direction (parallel to vector from EB1 to EB3)
        # Degrees
        OFFSET = 0

        # Subtract the offset from the heading
        yaw = heading - OFFSET

        # If the yaw is still greater than 180, subtract 360 from it so that it can be converted from a range of (0, 360) to (-180, 180) which is required by pure pursuit
        if yaw > 180:
            yaw -= 360

        return yaw
