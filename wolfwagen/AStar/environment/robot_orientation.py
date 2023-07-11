'''
This is a class that keeps track of the robot orientation and saves to a txt file that can be loaded to save current
orientation, given that starting from the bottom left of the field is origin and x axis is across and y axis is up
 ___
/___|
'''

import action
import position

class RobotOrientation:

    def __init__(self):
        '''
        Initialize by loading text file, if it doesn't exist make one
        '''
        self.curr_orientation = []
        self.load_orientation()

    def load_orientation(self, filename="../files/curr_orientation.txt"):
        curr_orientation = []
        try:
            # open the file, create if it doesn't exist
            # file format: ORIENTATION
            #              x_coord
            #              y_coord
            f = open(filename, 'r')
            lines = f.readlines()
            for line in lines:
                curr_orientation.append(line.strip())
            f.close()
        except FileNotFoundError:
            f = open(filename, 'w+')
            f.write("UP\n0\n0")
            curr_orientation = ["UP", "0", "0"]
            f.close()
        self.curr_orientation = curr_orientation

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

    def update_orientation(self, pos, x, y):
        new_orientation = [pos, x, y]
        self.curr_orientation = new_orientation

    def get_orientation(self):
        return self.curr_orientation

    @staticmethod
    def get_new_orientation(astar_move, pos, x, y):

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
