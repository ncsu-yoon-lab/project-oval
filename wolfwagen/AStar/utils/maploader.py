"""
This class loads map files from a filename into a list of strings
"""


class MapLoader:

    @staticmethod
    def load_map(filename):
        map_strings = []
        try:
            # open the file and for each line in the file add the whole line as a string without whitespace or new
            # line characters
            f = open(filename, 'r')
            lines = f.readlines()
            for line in lines:
                map_strings.append(line.rstrip('\n'))
            f.close()
        except FileNotFoundError:
            print("File not found " + filename)
        return map_strings
