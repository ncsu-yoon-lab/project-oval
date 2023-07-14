"""
This is a class that keeps track of the robot orientation and saves to a txt file that can be loaded to save current
orientation, given that starting from the bottom left of the field is origin and x axis is across and y axis is up
"""

import action

class RobotOrientation:

    def __init__(self):
        # Initialize by loading text file, if it doesn't exist make one
        self.curr_orientation = []
        self.load_orientation()

    # load orientation and position information from the curr_orientation file, if the filename is not specified,
    # use the default value
    def load_orientation(self, filename="../files/curr_orientation.txt"):
        # store the orientation information in an array
        curr_orientation = []
        try:
            # open the file, create if it doesn't exist
            # file format: ORIENTATION
            #              x_coord
            #              y_coord
            f = open(filename, 'r')
            lines = f.readlines()
            # add value to the array
            for line in lines:
                curr_orientation.append(line.strip())
            f.close()

            # if the file is not found, then create the file and add the default values to the orientation and the file
        except FileNotFoundError:
            f = open(filename, 'w+')
            f.write("UP\n0\n0")
            curr_orientation = ["UP", "0", "0"]
            f.close()
        self.curr_orientation = curr_orientation

    # this function saves the orientation to the orientation file
    def save_orientation(self, filename='../files/curr_orientation.txt'):
        try:
            # open the file, create if it doesn't exist
            # file format: ORIENTATION
            #              x_coord
            #              y_coord
            f = open(filename, 'w')
            for val in self.curr_orientation:
                f.write(str(val))
                f.write('\n')
            f.close()
        except FileNotFoundError:
            print("File cannot be found")

    # this function gets the current orientation from the object
    def get_orientation(self):
        return self.curr_orientation

    # this function gets the new potential orientation given a astar movement and the current orientation,
    # x, and y coords
    @staticmethod
    def get_new_orientation(astar_move, pos, x, y):

        # this is logic to figure out the new orientation after a given move
        if pos == "UP":
            if astar_move == action.Action.UP:
                return "UP"

            elif astar_move == action.Action.RIGHT:
                return "RIGHT"

            elif astar_move == action.Action.LEFT:
                return "LEFT"

            elif astar_move == action.Action.TURN:
                if int(x) == 0 and int(y) == 0:
                    return "RIGHT"
                return "DOWN"

        elif pos == "RIGHT":
            if astar_move == action.Action.UP:
                return "UP"

            elif astar_move == action.Action.RIGHT:
                return "RIGHT"

            elif astar_move == action.Action.DOWN:
                return "DOWN"

        elif pos == "LEFT":
            if astar_move == action.Action.UP:
                return "UP"

            elif astar_move == action.Action.LEFT:
                return "LEFT"

            elif astar_move == action.Action.DOWN:
                return "DOWN"

            elif astar_move == action.Action.TURN:
                if int(x) == 0 and int(y) == 0:
                    return "RIGHT"
                return "DOWN"

        elif pos == "DOWN":
            if astar_move == action.Action.RIGHT:
                return "RIGHT"

            elif astar_move == action.Action.LEFT:
                return "LEFT"

            elif astar_move == action.Action.DOWN:
                return "DOWN"

    # this function is the movement interpreter, which actually actuates the movement and updates the current
    # orientation with new coords and new orientations
    def movement_interpreter(self, astar_move):
        pos = self.curr_orientation[0]
        x = int(self.curr_orientation[2])
        y = int(self.curr_orientation[1])

        if pos == "UP":
            if astar_move == action.Action.UP:
                self.curr_orientation[1] = y + 1
                return "straight"

            elif astar_move == action.Action.RIGHT:
                self.curr_orientation[0] = "RIGHT"
                self.curr_orientation[2] = x + 1
                return "right"

            elif astar_move == action.Action.LEFT:
                self.curr_orientation[0] = "LEFT"
                self.curr_orientation[2] = x - 1
                return "left"

            elif astar_move == action.Action.TURN:
                if int(x) == 0 and int(y) == 0:
                    self.curr_orientation[0] = "RIGHT"
                    self.curr_orientation[2] = x + 1
                    self.curr_orientation[1] = y + 1
                    return "straight"

                self.curr_orientation[0] = "DOWN"
                self.curr_orientation[2] = x - 1
                self.curr_orientation[1] = y - 1
                return "left"

            elif astar_move == action.Action.DOWN:
                return "back"

        elif pos == "RIGHT":
            if astar_move == action.Action.UP:
                self.curr_orientation[0] = "UP"
                self.curr_orientation[1] = y + 1
                return "left"

            elif astar_move == action.Action.RIGHT:
                self.curr_orientation[2] = x + 1
                return "straight"

            elif astar_move == action.Action.DOWN:
                self.curr_orientation[0] = "DOWN"
                self.curr_orientation[1] = y - 1
                return "right"

            elif astar_move == action.Action.LEFT:
                return "back"

        elif pos == "LEFT":
            if astar_move == action.Action.UP:
                self.curr_orientation[0] = "UP"
                self.curr_orientation[1] = y + 1
                return "right"

            elif astar_move == action.Action.LEFT:
                self.curr_orientation[2] = x - 1
                return "straight"

            elif astar_move == action.Action.DOWN:
                self.curr_orientation[0] = "DOWN"
                self.curr_orientation[1] = y - 1
                return "left"

            elif astar_move == action.Action.TURN:
                if int(x) == 0 and int(y) == 0:
                    self.curr_orientation[0] = "RIGHT"
                    self.curr_orientation[2] = x + 1
                    self.curr_orientation[1] = y + 1
                    return "right"
                self.curr_orientation[0] = "DOWN"
                self.curr_orientation[2] = x - 1
                self.curr_orientation[1] = y - 1
                return "straight"

            elif astar_move == action.Action.RIGHT:
                return "back"

        elif pos == "DOWN":
            if astar_move == action.Action.RIGHT:
                self.curr_orientation[0] = "RIGHT"
                self.curr_orientation[2] = x + 1
                return "left"

            elif astar_move == action.Action.LEFT:
                self.curr_orientation[0] = "LEFT"
                self.curr_orientation[2] = x - 1
                return "right"

            elif astar_move == action.Action.DOWN:
                self.curr_orientation[1] = y - 1
                return "straight"

            elif astar_move == action.Action.UP:
                return "back"

    # this function is used to check if a move is valid, it returns True if the move is invalid and False if the
    # move is valid. A move is invalid if the current orientation of the robot and the next astar move causes the
    # robot to move backward, which is not allowed
    @staticmethod
    def move_check(astar_move, pos):

        if pos == "UP":
            if astar_move == action.Action.DOWN:
                return True

        elif pos == "RIGHT":
            if astar_move == action.Action.LEFT:
                return True

        elif pos == "LEFT":
            if astar_move == action.Action.RIGHT:
                return True

        elif pos == "DOWN":
            if astar_move == action.Action.UP:
                return True

        return False
