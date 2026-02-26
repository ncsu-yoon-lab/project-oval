class PurePursuitPid:
    def __init__(self, kp=0.5, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.prev_error = 0
    
    def update(self, error, dt):
        # Compute I
        self.integral += error * dt
        
        # Compute D
        derivative = 0
        if dt > 0:
            derivative = (error - self.prev_error) / dt

        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative

        output_throttle = p + i + d

        self.prev_error = error

        # output of this controller is throttle
        return output_throttle

    
    def set_constants(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        

    def reset(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.prev_error = 0
        self.integral = 0