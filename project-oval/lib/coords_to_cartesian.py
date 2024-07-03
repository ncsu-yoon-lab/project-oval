from geopy import distance
import math

class CoordsToCartesian(object):

    # The reference coordinates for the OVAL
    origin = (35.7713528, -78.673756)
    referenceX = (35.7706063, -78.6728862)
    referenceY = (35.7692556, -78.6743267)

    def __init__(self):
        # Initializing the coordinate system with lat long for origin, reference point on x axis and reference point on y axis
        self.OX = distance.distance(self.origin, self.referenceX).meters
        self.OY = distance.distance(self.origin, self.referenceY).meters

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
    
    def heading_to_yaw(self, heading) -> float:

        # Not sure about the offset, needs to be tested
        OFFSET = 58
        yaw = heading + OFFSET

        return yaw
