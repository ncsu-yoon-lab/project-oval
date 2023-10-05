from enum import Enum

"""
 Enumeration class to contain types of road environment
 @author: Sarvesh
"""


class PositionTypeStatus(Enum):
    # Enum to contain the status of the Road
    EDGE = 1
    INTERSECTION = 2
    CURVE = 3
