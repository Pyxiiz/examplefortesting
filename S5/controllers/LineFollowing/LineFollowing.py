from controller import Robot
import math
import numpy as np

robot = Robot()

timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

MAX_SPEED = 6.28

gs = []
for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)

FORWARD_SPEED = 0.5 * MAX_SPEED
TURN_FAST = 0.25 * MAX_SPEED
TURN_SLOW = -0.05 * MAX_SPEED
STOP = 0 * MAX_SPEED

phildot, phirdot = FORWARD_SPEED, FORWARD_SPEED

#Odometry
WHEEL_RADIUS = 0.0201   
AXLE_LENGTH = 0.052     
dt = timestep / 1000.0  

x_total = 0.0
theta_total = 0.0

#Initialize
xw = 0.0
yw = 0.0
alpha = 0.0

#Start/finish line detection (based on odometry return-to-origin)
LEAVE_RADIUS = 0.20     #Robot must get at least this far from start before a "return" counts
RETURN_RADIUS = 0.04    #Close enough to (0,0) again. .04 seems to be good value
left_start = False
finished = False

while robot.step(timestep) != -1:
    #Read sensors
    g = [gsensor.getValue() for gsensor in gs]
    print(g)

    if not finished:
        if g[0] > 500 and g[1] < 305 and g[2] > 500:
            phildot, phirdot = FORWARD_SPEED, FORWARD_SPEED
        elif g[2] < 550:
            phildot, phirdot = TURN_FAST, TURN_SLOW
        elif g[0] < 305 and g[1] < 305:
            phildot, phirdot = TURN_SLOW, TURN_FAST
        elif g[0] < 305 and g[1] > 500 and g[2] > 500:
            phildot, phirdot = TURN_SLOW, TURN_FAST
        elif g[0] > 500 and g[1] > 500 and g[2] > 500:
            pass

    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)

    dx = WHEEL_RADIUS * (phildot + phirdot) / 2.0 * dt
    dtheta = WHEEL_RADIUS * (phirdot - phildot) / AXLE_LENGTH * dt

    x_total += dx
    theta_total += dtheta
    heading_deg = (theta_total / math.pi) * 180.0

    #Calculations
    xw = xw + np.cos(alpha) * dx
    yw = yw + np.sin(alpha) * dx
    alpha = alpha + dtheta

    error = np.sqrt(xw**2 + yw**2)

    print(f"Odometry: Distance = {x_total:.4f} m, Heading = {heading_deg:.2f} deg")
    print(f"Pose: xw = {xw:.4f} m, yw = {yw:.4f} m, alpha = {math.degrees(alpha):.2f} deg")
    print("Error:", error)

    if not finished:
        if not left_start and error > LEAVE_RADIUS:
            # robot has moved away from the start -> now watch for it coming back
            left_start = True
        elif left_start and x_total > 3 and (g[0] < 450 and g[1] < 450 and g[2] < 450):
            # Must have traveled > 2.5m (near the end of the ~2.72m loop) and hit the start line
            finished = True
            phildot, phirdot = STOP, STOP
            leftMotor.setVelocity(STOP)
            rightMotor.setVelocity(STOP)
            print(f"Loop complete! Stopped at start line, final error = {error:.4f} m")