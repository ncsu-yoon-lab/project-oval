import math
'''
Actual motors on car: https://flipsky.net/products/63100-motor-190kv-5000w
datasheet for motor, used to inform some of the sim decision: https://cdn.shopify.com/s/files/1/0011/4039/1996/files/63100-190KV-48V.pdf?v=1735636733
Contains methods for driving the sim car.

Torque constant = 48.1 Nm/A

max_motor_torque=25450
max_motor_torque = max_motor_torque / 1000
'''
class SimDriverLib():
    # gear ratio and drivetrain efficiency should be tuned.
    def __init__(self, stall_torque=10, motor_torque_const=48.1, gear_ratio=2, drivetrain_eff=0.95):
        
        self.gear_ratio = gear_ratio
        # make sure to convert out of mNm
        self.stall_torque = stall_torque
        self.motor_torque_const = motor_torque_const / 1000
    
        self.drivetrain_eff = drivetrain_eff
        

    
    
    # Pedal Shaping functions. gamma > 1
    def soft_pedal(self, throttle, gamma=1.5):
        if gamma <= 1:
            print("[WARNING] - using soft pedal method with gamma <= 1")
        return throttle**gamma

    def linear_pedal(self, throttle, gamma=1):
        return throttle*gamma

    # Get up to speed quick, but levels out
    def aggressive_pedal(self, throttle, gamma=0.5):
        if gamma >= 1:
            print("[WARNING] - using aggressive pedal method with gamma >= 1")
        return throttle**gamma

    def rpm_to_rads(self, rpm):
        return rpm*(2*math.pi/60)
    
    # 
    def efficiency_eta(self, current_torque):
        return (95 -  0.03958333333 * current_torque)/100

    def get_torque_input(self, throttle, current_wheel_speeds, pedal_function):

        """
        modified to support direction
        """

        # Average wheel speed (signed)
        avg_wheel_speed = sum(current_wheel_speeds) / len(current_wheel_speeds)

        # 2) Motor speed from gearing (signed)
        motor_speed = avg_wheel_speed * self.gear_ratio

        #  Motor constants
        no_load_speed = self.rpm_to_rads(9475)   # rad/s, positive
        stall_torque = self.stall_torque         # Nm, positive magnitude

        # Available torque depends on speed magnitude (NOT direction)
        speed_ratio = abs(motor_speed) / no_load_speed
        torque_avail = stall_torque * (1.0 - speed_ratio)
        torque_avail = max(0.0, torque_avail)

        # Throttle sign controls direction; pedal map shapes magnitude only
        #    Clamp throttle to [-1, 1] for safety.
        throttle = max(-1.0, min(1.0, float(throttle)))
        throttle_sign = 1.0 if throttle > 0 else (-1.0 if throttle < 0 else 0.0)
        throttle_mag = abs(throttle)

        adjusted_throttle_mag = pedal_function(throttle_mag)  # expects [0,1] to [0,1]
        adjusted_throttle_mag = max(0.0, min(1.0, float(adjusted_throttle_mag)))

        #  Motor torque (signed)
        motor_torque = throttle_sign * adjusted_throttle_mag * torque_avail

        # Axle torque (signed). gear_ratio and drivetrain_eff are positive scalars.
        axle_torque = motor_torque * self.gear_ratio * self.drivetrain_eff

        return axle_torque

