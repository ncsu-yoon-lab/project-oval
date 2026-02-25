import math
import numpy as np
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
        self.point = np.array([float(x), float(y)])
        
        # cumulative distance the point is along the path
        self.cumulative_arc_length = cal
    
    def set_arc_length(self, cal):
        self.cumulative_arc_length = cal

    def get_arc_length(self):
        return self.cumulative_arc_length

    def get_point(self):
        return self.point



class PurePursuit:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = float(lookahead_distance)

        self.path = []

        # May remove points from path as it is traversed for efficency
        # Will store untraversed points in this array.
        self.untraversed_path = []

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
        
        pass

    # there are two ways to find the target point. 
    # One uses vector projection, and the other uses circle-line intersection
    # For this implementation I'm going to use vector circle-line intersection. This is how the original pure pursuit works.
    def compute_target_point(self):
        # search for the closest point
    

# helper function: sgn(num)
# returns -1 if num is negative, 1 otherwise
def sgn (num):
    if num >= 0:
        return 1
    else:
        return -1
    

def line_circle_intersection (robot_pose, segment_start, segment_end, lookahead):

    center = robot_pose.get_point()
    start = segment_start.get_point()
    end = segment_end.get_point()

     # the line will be p(t) = segment_start + d*t where t is between 0 and 1
     # thank you calc 3. I guess.

    direction = end - start
    startToCenter = start - center

    a = float(np.dot(direction, direction))
    b = 2.0 * float(np.dot(startToCenter, direction))
    c = float(np.dot(startToCenter, startToCenter)) - float(lookahead * lookahead)

    floating_point_error = 1.0e-12

    discriminant = b**2 - 4.0 * a * c
    intersections = ()

    if discriminant < -floating_point_error:  # no intersection for target point
        return None
    
    elif math.abs(discriminant) <= floating_point_error: # equals zero, one intersection
        t = -b / (2.0 * a)
        if 0.0 <= t <= 1.0:
            intersections.append(start + t * direction)
        return intersections
    
    elif discriminant > 0: # two intersections
        root = float(np.sqrt(discriminant))
        t1 = (-b - root) / (2.0 * a)
        t2 = (-b + root) / (2.0 * a)
        if 0.0 <= t1 <= 1.0:
            intersections.append(start + t1 * direction)
        if 0.0 <= t2 <= 1.0:
            intersections.append(start + t2 * direction)
        
        return intersections
    else:
        print("[WARNING] line circle intersection returned an unusual value: " + str(discriminant))
        return None

   


    def reset(self):
        pass