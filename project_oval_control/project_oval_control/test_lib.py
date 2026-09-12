from sim_driver_lib import SimDriverLib

driver = SimDriverLib()

wheel_speeds = [-10, -10]  # rad/s
throttle = -1

# Pass a pedal function. If you want a custom gamma, wrap it in a lambda.
axle_torque = driver.get_torque_input(throttle, wheel_speeds, lambda x: driver.soft_pedal(x))

print(axle_torque / 2)