"""Module 5 Controller"""

from controller import Robot
import numpy as np

# Constants
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.02
TIRE_RADIUS = 0.0201
ROBOT_DIAMETER = 0.052

# Position and rotation in world coordinates
xw = 0.028 # starting just ahead of the start line
yw = 0
omegaz = np.pi / 2 # start rotated +90 degrees to world coords

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get the motor devices
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

# turn off position control
motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

# get the ground sensors
gs = []

for g in range(3):

    # get the gth ground sensor and enable it
    sensor_name = f'gs{g}'
    gs.append(robot.getDevice(sensor_name))
    gs[g].enable(timestep)

# these are from the previous module, but keeping them for some independent checking
total_displacement = 0
total_rotation = 0

# This flag gets set to true when the robot returns to the start line, ending the while loop 
is_complete = False

while robot.step(timestep) != -1 and not is_complete:

    # Do some calculations from the timestep that just concluded
    
    # get the old velocity for computing displacement in the timestep that just ended
    v_l = motor_left.getVelocity()
    v_r = motor_right.getVelocity()
    
    # calculate incremental and total displacement
    displacement = ((TIRE_RADIUS * v_l + TIRE_RADIUS * v_r) / 2)  * timestep / 1000
    total_displacement += displacement
    
    # calculate incremental and total rotation
    rotation = ((TIRE_RADIUS * v_r - TIRE_RADIUS * v_l) / ROBOT_DIAMETER) * timestep / 1000
    total_rotation += rotation

    # Update the world x, y, and rotation estimates
    xw += np.cos(omegaz) * displacement
    yw += np.sin(omegaz) * displacement
    omegaz += rotation
    
    print(f"xw: {xw:.4f}, yw: {yw:.4f}, omegaz: {omegaz * 180 / np.pi:.4f}")

    # Read the ground sensors
    g = []

    for gsensor in gs:
        g.append(gsensor.getValue())

    # stopping. this is hacky. All sensors can be over black at the corners,
    # so only check for the stopping condition after 3 meters.
    if total_displacement > 3.0 and g[0] < 500 and g[1] < 500 and g[2] < 500:
        phildot = phirdot = 0.0
        is_complete = True

    elif g[0] > 500 and g[1] < 350 and g[2] > 500: # drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED

    elif g[2] < 550: # turn right
        phildot, phirdot = 0.25 * MAX_SPEED, -0.1 * MAX_SPEED

    elif g[0] < 550: # turn left
        phildot, phirdot = -0.1 * MAX_SPEED, 0.25 * MAX_SPEED

    # Update motor speeds
    motor_left.setVelocity(phildot)
    motor_right.setVelocity(phirdot)

print("---------------")
print("All done.")
print(f"xw:           {100 * xw:.2f}cm")
print(f"yw:           {100 * yw:.2f}cm")
print(f"error:        {100 * np.sqrt(xw**2 + yw**2):.1f}cm")
print(f"omegaz (rad): {omegaz:.4f}")
print(f"omegaz (deg): {omegaz * 180 / np.pi:.4f}")
