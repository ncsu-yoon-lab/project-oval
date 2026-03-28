import math
import numpy as np

import pdb
'''
Pure Pursuit Algorithm:

Current considerations:
- Pure pursuit should work in any frame. odometry or map (gps)
- Webots publishes odometry which will work for simple testing
- Webots also supports a gps sensor.
- For odometry on car, we will use RTK gps and onboard IMU
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

class Segment2D:
    def __init__(self, point1, point2, t=None, index=0):
        self.point1 = point1
        self.point2 = point2
        self.t = None
        self.index = index

class PurePursuit:
    def __init__(self, lookahead_distance, velocity):
        self.lookahead_distance = float(lookahead_distance)   # current runtime value
        self.base_lookahead_distance = float(lookahead_distance)  # fixed base
        self.lookahead_scaling = 0

        self.path = []
        self.path_length = 0
        self.last_closest_index = 0
        # tuning parameter for how much we want to slow down around turns
        self.alpha = 0.3

        # tuning parameter for what percentage along the path we want to start slowing down before goal.
        # should always be between 0 and 1
        self.beta = 0.85 # start scaling down velocity when we are 85% of the way to the goal.

        # Tuning parameter for how aggressively we slow down as we reach the goal point.
        # Higher zeta = much slower approach to goal.
        # should always be between 0 and 1
        self.zeta = 0.65

        self.velocity = velocity # maximum operating speed for pure pursuit

        # end condition tolerance (meters)
        self.goal_tolerance = 0.10

        self.current_segment = None

    def init_path(self, path):
        # init the path.
        self.path = []
    
        for point in path:
            p += 1
            x = float(point[0])
            y = float(point[1])
            current_point = PathPoint2D(x, y)
        
            self.path.append(current_point)
        
        for i in range(0, len(path) - 1):
            


        
        
        self.path_length = len(self.path)

        # if path is empty, nothing to do
        if self.path_length == 0:
            self.last_closest_index = 0
            return

        # naturally, the first point in the path hase zero cumulative distance
        self.path[0].set_arc_length(0.0)

        # pre-compute each point's cumulative distance along the path
        for i in range(1, len(self.path)):
            current_point = self.path[i]
            previous_point = self.path[i-1]

            x1, y1 = current_point.get_point()
            x0, y0 = previous_point.get_point()

            dx = x1 - x0
            dy = y1 - y0

            distance = math.hypot(dx, dy)

            # add distance to last arc length
            current_total_arc_length = previous_point.get_arc_length() + distance
            
            current_point.set_arc_length(current_total_arc_length)

        self.last_closest_index = 0

   
    def update_state(self, robot_pose, yaw, velocity):
        # velocity value is not required, some pure pursuit implementations
        # use current velocity to choose lookahead distance, we can have that too.
        # velocity-scaled lookahead (no accumulation)
        self.lookahead_distance = self.base_lookahead_distance + self.lookahead_scaling * velocity

       
        if self.path_length == 0:
            return (0.0, 0.0)


        robot_xy = robot_pose.get_point()
        goal_xy = self.path[-1].get_point()

         # end condition (close enough to final waypoint)
        if np.linalg.norm(robot_xy - goal_xy) <= self.goal_tolerance:
            return (0.0, 0.0)

        target_point = self.compute_target_point(robot_pose)
        print("TARGET POINT: ")
        if target_point is None:
            print("No target point found")
            # fallback when no point is found, follow the last closest index
            target_point = self.path[self.last_closest_index].get_point()
        
        print("x: ", target_point[0])
        print("y: ", target_point[1])

        # Special case, pure pursuit does not handle it well when the target point is behind (~180) the robot, you end up with very wide turns
        # that could take the car into the grass, or a wall. It is better, and safer to rotate in place until we are facing the target point.
        # heading_error = self.get_heading_error(yaw)
        # if heading_error > math.radians(90): # if car is facing too far in the other direction, rotate

        #     return (0, self.velocity) # no, forward velocity, but angular velocity 

        kappa = self.curvature_from_target(robot_pose, yaw, target_point, self.lookahead_distance)
        print("Kappa: ", kappa)

        # curvature based speed scaling. computes desired linear velocity
        vcmd = float(self.velocity) / (1.0 + float(self.alpha) * abs(float(kappa)))

        # When getting to the goal, the car occasionally overshoots the target due to momentum.
        # This will scale the velocity commands down as we get closer to teh goal

        # get total distance of path and the total distance traveled so far along the path
        distance_traveled, total_distance = self.get_estimated_travel_distance(robot_xy)

        ratio = distance_traveled / total_distance
        if ratio >= self.beta: # if we are in that range of scaling down, lets scale down velocity
            vcmd = vcmd - (vcmd * ratio * self.zeta) # scal back velocity by how close we are to the goal, and parameter zeta
        
        # 

        # differential drive relation omega = v * kappa
        wcmd = vcmd * kappa

        # we will now take this value to a controller for robot control.
        return (vcmd, wcmd)
    
    # When getting to the goal, the car occasionally overshoots the target due to momentum.
    # This method will be used for getting an estimation of how far along the path the car is.
    # This way, we can scale velocity down as the car gets closer to the goal.
    def get_estimated_travel_distance(self, robot_xy):
        seg_start = self.path[self.last_closest_index].get_point()
        seg_end = self.path[self.last_closest_index + 1].get_point()

        segment_vector = seg_end - seg_start

        seg_length = np.linalg.norm(segment_vector)

        # we can use vector projection to estimate how far along the robot is for the current segment
        # project robot's position onto current segment
        robot_vector = robot_xy - seg_start # line from the segment start to the robot.

        # (a dot b) / (b^2) is the mag of the vector projected. Assuming the car is not past this segment, t between [0, 1]
        t = np.dot(robot_vector, segment_vector) / (seg_length * seg_length)

        # Only time when this should be a problem is if we are moving fast and move beyond the segment before finishing this calc.
        t = np.clip(t, 0.0, 1.0) # clip it just in case. f

        # Estimated distance traveled along the path. Not perfect, but a really good estimate.
        distance_traveled = self.path[self.last_closest_index].get_arc_length() + t * seg_length
        distance_total = self.path[-1].get_arc_length()

        return distance_traveled, distance_total
    
    def get_heading_error(self, yaw):
        seg_start = self.path[self.last_closest_index].get_point()
        seg_end = self.path[self.last_closest_index + 1].get_point()

        dx = seg_end[0] - seg_start[0]
        dy = seg_end[1] - seg_start[1]

        segment_angle = math.atan2(dy, dx)

        heading_error = abs(yaw - segment_angle)
        return heading_error

    # compute the curbature
    def curvature_from_target(self, robot_pose, robotyaw, target, lookahead):
        robot_xy = robot_pose.get_point()
        dx = target[0] - robot_xy[0]
        dy = target[1] - robot_xy[1]

        c = math.cos(robotyaw)
        s = math.sin(robotyaw)

        xprime = c * dx + s * dy
        yprime = -s * dx + c * dy

        kappa = 2.0 * yprime / (lookahead * lookahead)
        return kappa

    # there are two ways to find the target point. 
    # One uses vector projection, and the other uses circle-line intersection
    # For this implementation I'm going to use vector circle-line intersection. This is how the original pure pursuit works.
    def compute_target_point(self, robot_pose):
        for i in range(self.last_closest_index, self.path_length - 1):
            segment_start = self.path[i]
            segment_end = self.path[i+1]
            target_points, t = self.line_circle_intersection(robot_pose, segment_start, segment_end, self.lookahead_distance)
            if target_points and len(target_points) > 0:
                # check current segment if exists
                if self.current_segment is not None:

                      
                else:
                    # update current segment
                    self.current_segment = Segment2D(segment_start, segment_end, target_points[0][1], i)
                    
                self.last_closest_index = i
                return target_points[0]
        return None
                
    

    def line_circle_intersection(self, robot_pose, segment_start, segment_end, lookahead):

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
        # handle degenerate segment start==end
        if abs(a) <= floating_point_error:
            return None

        discriminant = b**2 - 4.0 * a * c
        intersections = [] # this appearing as a list is not needed for how I choose between two possible target points

        if discriminant < -floating_point_error:  # no intersection for target point
            return None
        # t should be between 0 and 1 so that the point is on the segment
        elif abs(discriminant) <= floating_point_error: # equals zero, one intersection
            t = -b / (2.0 * a)
            if 0.0 <= t <= 1.0:
                intersections.append(start + t * direction)
            return intersections, t if len(intersections) > 0 else None
        
        elif discriminant > 0.0:
            root = float(np.sqrt(discriminant))
            t1 = (-b - root) / (2.0 * a)
            t2 = (-b + root) / (2.0 * a)

            # eliminate roots not on the segment
            if not (0.0 <= t1 <= 1.0):
                t1 = -1.0
            if not (0.0 <= t2 <= 1.0):
                t2 = -1.0
            
            # if no valid roots, return none
            if t1 < 0.0 and t2 < 0.0:
                return None

            # Select the root farther along the line segment
            t = max(t1, t2)
            
            intersections = start + t * direction
            return intersections, t
        else:
            print("[WARNING] line circle intersection returned an unusual value: " + str(discriminant))
            return None

    def reset(self):
        # reset progress
        self.last_closest_index = 0

        # clear path
        self.path = []
        self.path_length = 0