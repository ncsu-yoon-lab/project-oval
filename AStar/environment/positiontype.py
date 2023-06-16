import positiontypestatus as rs

"""
This is a simple object to represent the road and surrounding environment on the track
@author: Sarvesh
"""


class PositionType:

    # this initializes the road with a given Road Status
    def __init__(self, status):
        self.status = status

    # this gets the status of the road
    def get_status(self):
        return self.status

    # checks if road is an intersection
    def is_intersection(self):
        return self.status is rs.PositionTypeStatus.INTERSECTION

    # checks if the given position is not an edge
    def is_valid(self):
        return self.status is not rs.PositionTypeStatus.EDGE

    # this prints the current status of the Road tile out
    def __repr__(self):
        return f"Tile({self.status})"
