'''
Pure Pursuit Algorithm:

Current considerations:
- Pure pursuit should work in any frame. odometry or map (gps)
- Webots publishes odometry which will work for simple testing
- Webots also supports a gps sensor
'''
class PurePursuit:
    
    def __init__(self, lookahead_distance):
        pass

    def set_path(self, path):
        pass

   
    def update_state(self, x, y, yaw, velocity):
         # Each iteration algorithm will need vehicles x,y coordinates, heading (yaw) and velocity
         # 
        pass

    def compute_target_point(self):
        pass

    def reset(self):
        pass