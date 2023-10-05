"""road
This is the position coordinate, (row, col), represented as a node within the track environment
creates a graph-like representation of the positions of the intersection in any given map

@author: Sarvesh
"""


class Position:

    # This initializes the Position object with row, col, and neighbor positions which
    # are initially set to None
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.above = None
        self.below = None
        self.right = None
        self.left = None

    """
    Traditional getters and setters
    """
    # get row
    def get_row(self):
        return self.row

    # get col
    def get_col(self):
        return self.col

    # set above position
    def set_above(self, neighbor):
        self.above = neighbor
        neighbor.below = self

    # set below position
    def set_below(self, neighbor):
        self.below = neighbor
        neighbor.above = self

    # set right position
    def set_right(self, neighbor):
        self.right = neighbor
        neighbor.left = self

    # set left position
    def set_left(self, neighbor):
        self.left = neighbor
        neighbor.right = self

    # get above
    def get_above(self):
        return self.above

    # get below
    def get_below(self):
        return self.below

    # get right
    def get_right(self):
        return self.right

    # get left
    def get_left(self):
        return self.left

    """
    These are functions that are needed to make the position object comparable, so that it can be used in ordered lists,
    dicts, and queues/stacks
    """

    # this is a print function
    def __repr__(self):
        return f"Position({self.row}, {self.col})"

    # this is an equals function that check if two position objects equal each other
    def __eq__(self, other):
        if isinstance(other, Position):
            return (self.row, self.col) == (other.row, other.col)
        return False

    # this is a hash function that overrides the default hash function, we hash the row and col together decided to
    # use row and col because two positions that are hashed that have the same row and col need to equal each other
    def __hash__(self):
        return hash((self.row, self.col))

