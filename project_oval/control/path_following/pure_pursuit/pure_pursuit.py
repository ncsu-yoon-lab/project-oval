import math
'''
Pure Pursuit Algorithm:

Current considerations:
- Pure pursuit should work in any frame. odometry or map (gps)
- Webots publishes odometry which will work for simple testing
- Webots also supports a gps sensor
'''

# Point along the path will be a 2D tuple of floats
Point2D = (float, float)
class PurePursuit:
    
    def __init__(self, lookahead_distance):
        self.lookahead_distance = float(lookahead_distance)

        self.path = []
        self.arc_lengths = []

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

    def set_path(self, path):

        # init the path.
        self.path = []
        for point in path:
            x = float(point[0])
            y = float(point[1])
            self.path.append((x, y))
        
        # Init the arc lengths along the path
        self.arc_lengths = []
        self.arc_lengths.append(0.0)

        # Compute the cumulative distance along the path
        for i in range(1, len(self.path)):
            x0, y0 = self.path[i - 1]
            x1, y1 = self.path[i]

            dx = x1 - x0
            dy = y1 - y0

            distance = math.hypot(dx, dy)
            cumulative = self.arc_lengths[-1] + distance
            self.arc_lengths.append(cumulative)

        self.last_closest_index = 0
    

   
    def update_state(self, x, y, yaw, velocity):
         # Each iteration algorithm will need vehicles x,y coordinates, heading (yaw) and velocity
         # 
        pass

    def compute_target_point(self):
        pass

    def reset(self):
        pass