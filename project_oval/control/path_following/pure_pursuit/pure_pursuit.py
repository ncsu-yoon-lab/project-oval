import math
'''
Pure Pursuit Algorithm:

Current considerations:
- Pure pursuit should work in any frame. odometry or map (gps)
- Webots publishes odometry which will work for simple testing
- Webots also supports a gps sensor
'''

# Point along the path will be a 2D tuple of floats
class PathPoint2D:
    def __init__(self, x, y, cal=0):
        self.x = x
        self.y = y
        
        # cumulative distance the point is along the path
        self.cumulative_arc_length = cal
    
    def set_arc_length(self, cal):
        self.cumulative_arc_length = cal

    def get_arc_length(self):
        return self.cumulative_arc_length

    def get_point(self):
        return (self.x, self.y)


class PurePursuit:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = float(lookahead_distance)

        self.path = []

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

        self.last_closest_index = 0

    def init_path(self, path):

        # init the path.
        self.path = []
        for point in path:
            x = float(point[0])
            y = float(point[1])
            current_point = PathPoint2D(x, y)

            self.path.append(current_point)

        # naturally, the first point in the path hase zero cumulative distance
        self.path[0].set_arc_length(0.0)

        # pre-compute each point's cumulative distance along the path
        for i in range(1, len(self.path)):
            current_point = self.path[i]
            previous_point = self.path[i-  1]

            x1, y1 = current_point.get_point()
            x0, y0 = previous_point.get_point()

            dx = x1 - x0
            dy = y1 - y0

            distance = math.hypot(dx, dy)

            # add distance to last arc length
            current_total_arc_length = previous_point.get_arc_length() + distance
            
            current_point.set_arc_length(current_total_arc_length)

        self.last_closest_index = 0
    

   
    def update_state(self, x, y, yaw, velocity):
         # Each iteration algorithm will need vehicles x,y coordinates, heading (yaw) and velocity
         # 
        pass

    def compute_target_point(self):
        pass

    def reset(self):
        pass