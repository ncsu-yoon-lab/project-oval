from enum import Enum

"""
An enumeration for the action items, left, up, down, right, stop, or turn (used to travel through curve)
We need to use these values to figure out how the robot needs to move in Lane Detection
@author: Sarvesh
"""


class Action(Enum):
    # Enum to contain the status of the Road
    LEFT = 1
    RIGHT = 2
    UP = 3
    DOWN = 4
    STOP = 5
    TURN = 6
