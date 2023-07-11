'''
This is a class that keeps track of the robot orientation and saves to a txt file that can be loaded to save current
orientation, given that starting from the bottom left of the field is origin and x axis is across and y axis is up
 ___
/___|
'''

import action

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

    def update_orientation(self, pos, x, y):
        new_orientation = [pos, x, y]
        self.curr_orientation = new_orientation

    def get_orientation(self):
        return self.curr_orientation

    # def movement_interpreter(self, astar_move):
    #     pos = self.curr_orientation[0]
    #     x = self.curr_orientation[1]
    #     y = self.curr_orientation[2]
    #
    #     if pos == "UP":
    #         if astar_move == action.Action.UP:
    #             self.curr_orientation[2] = y + 1
    #             return "straight"
    #
    #         elif astar_move == action.Action.RIGHT:
    #             self.curr_orientation[0] = "RIGHT"
    #             self.curr_orientation[1] = x + 1
    #             return "right"
    #
    #         elif astar_move == action.Action.LEFT:
    #             self.curr_orientation[0] = "LEFT"
    #             self.curr_orientation[1] = x - 1
    #             return "left"
    #
    #         elif astar_move == action.Action.DOWN:
    #             self.curr_orientation[0] = "DOWN"
    #             self.curr_orientation[1] = y - 1
    #             return "back"
    #
    #         elif astar_move == action.Action.TURN:
    #             self.curr_orientation[0] = "RIGHT"
    #             self.curr_orientation[1] = x + 1
    #             self.curr_orientation[2] = y + 1
    #             return "straight"
    #
    #
    #     elif pos == "RIGHT":
    #         if astar_move == action.Action.UP:
    #             self.curr_orientation[0] = "UP"
    #             self.curr_orientation[2] = y + 1
    #             return "left"
    #
    #         elif astar_move == action.Action.RIGHT:
    #             self.curr_orientation[1] = x + 1
    #             return "straight"
    #
    #         elif astar_move == action.Action.LEFT:
    #             self.curr_orientation[0] = "LEFT"
    #             self.curr_orientation[1] = x - 1
    #             return "back"
    #
    #         elif astar_move == action.Action.DOWN:
    #             self.curr_orientation[0] = "DOWN"
    #             self.curr_orientation[1] = y - 1
    #             return "right"
    #
    #         elif astar_move == action.Action.TURN:
    #             self.curr_orientation[0] = "RIGHT"
    #             self.curr_orientation[1] = x + 1
    #             self.curr_orientation[2] = y + 1
    #             return "straight"
    #
    #     elif pos == "LEFT":
    #         if astar_move == action.Action.UP:
    #
    #         elif astar_move == action.Action.RIGHT:
    #
    #         elif astar_move == action.Action.LEFT:
    #
    #         elif astar_move == action.Action.DOWN:
    #
    #         elif astar_move == action.Action.TURN:
    #
    #     elif pos == "DOWN":
    #         if astar_move == action.Action.UP:
    #
    #         elif astar_move == action.Action.RIGHT:
    #
    #         elif astar_move == action.Action.LEFT:
    #
    #         elif astar_move == action.Action.DOWN:
    #
    #         elif astar_move == action.Action.TURN:
