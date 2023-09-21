"""
This class loads costs from a filename into a list of costs
"""


class CostLoader:

    @staticmethod
    def load_map(filename):
        cost_vals = []
        try:
            # open the file and for each line in the file add the whole line as a string without whitespace or new
            # line characters
            f = open(filename, 'r')
            for line in f.readlines():
                for val in line.strip().split(','):
                    try:
                        cost_vals.append(float(val))
                    except ValueError:
                        if val == "\n":
                            cost_vals.append("N")
            f.close()
        except FileNotFoundError:
            print("File not found " + filename)
        return cost_vals

    # gets straightline distances as a map from file
    @staticmethod
    def load_straight_line(filename, positions):
        straight_line = dict()
        i = 0
        try:
            f = open(filename, 'r')
            lines = f.readlines()
            for line in lines:
                parent_pos = positions[i]
                j = i
                next_pos = positions[j]
                straight_line[(parent_pos, next_pos)] = 0.0
                for val in line.strip().split(','):
                    j = j + 1
                    next_pos = positions[j]
                    try:
                        straight_line[(parent_pos, next_pos)] = float(val)
                    except ValueError:
                        print("error")
                i += 1
            straight_line[(positions[i], positions[i])] = 0.0
            f.close()
        except FileNotFoundError:
            print("File not found " + filename)
        return straight_line
