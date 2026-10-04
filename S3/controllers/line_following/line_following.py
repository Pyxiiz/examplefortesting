# line_following controller
# danielle drugan 09/27/2026

from controller import Robot
import math
import numpy as np

# create the Robot instance.
robot = Robot()

#initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# get the time step of world
timestep = int(robot.getBasicTimeStep())

gs = [] # enable ground sensors
for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)
    
MAX_SPEED = 6.28

r = 0.0201 # wheel radius
d = 0.052 # wheel distance
delta_t = timestep / 1000.0  # time step in second

distance = 0.0 # initialize distance counter
orientation_rad = 0.0 # initialize orientation
startLine = False # for printing purposes only, not a state

# initialize world positions and alpha angle
xw = 0.0
yw = 0.028 # start position per video
alpha = (math.pi / 2) # start at 90 degrees

while robot.step(timestep) != -1:
    # read sensors
    g=[]
    phildot = 0.0 # initialize motors at 0
    phirdot = 0.0 # initialize motors at 0
    
    for gsensor in gs:
        g.append(gsensor.getValue())#get ground sensor data
    

    if (g[0] < 350 and g[1] < 350 and g[2] < 350) and (orientation_deg <= -340.0):
        # ground sensors see all black and epuck has down a 360 degree rotation
        phildot, phirdot = 0.0, 0.0
        error = math.sqrt(xw**2 + yw**2)
        if not startLine: #otherwise keeps printing 
            print("Start line reached!", g)
            print(f"\nDistance: {distance:.4f}m  \tOrientation: {orientation_deg:.2f}°")
            print(f"Final pose: \nxw: {xw}, yw: {yw}")
            print(f"Error: \t{error* 100:.4f}cm")
        startLine = True
    elif (g[0] > 500 and g[1]<350 and g[2]>500): # drive straight
        #ground sensors see white on left and right, black in middle
       phildot, phirdot = 0.8*MAX_SPEED, 0.8*MAX_SPEED
       # print("Driving straight", g)
    elif (g[2]<550): # turn right
        #ground sensor sees black on right
        phildot, phirdot = 0.25 * MAX_SPEED, -0.1*MAX_SPEED
        # print("Turning right", g)
    elif (g[0]<550): # turn left
        #ground sensor sees black on left
        phildot, phirdot = -0.1*MAX_SPEED, 0.25 * MAX_SPEED
        # print("Turning left", g)
    else: # get around corners slowly
        phildot, phirdot = 0.2 * MAX_SPEED, 0.2 * MAX_SPEED
        # print("Driving slowly", g)

    # change in distance and orientaion
    deltaX = ((r * phildot) + (r * phirdot)) / 2.0 * delta_t
    deltaomegaz = ((r * phirdot) - (r * phildot)) / d * delta_t
    
    # track distance and orientation
    distance += deltaX
    orientation_rad += deltaomegaz
    
    # odometry localization tracking
    xw = xw + np.cos(alpha) * deltaX
    yw = yw + np.sin(alpha) * deltaX
    alpha = alpha + deltaomegaz
    
    # convert orientation to degrees
    orientation_deg = (orientation_rad / math.pi) * 180.0
    if not startLine:#otherwise keeps printing
        print(f"\nDistance: {distance:.4f}m  \tOrientation: {orientation_deg:.2f}°")
        print(f"xw: {xw}, yw: {yw}")
    leftMotor.setVelocity(phildot) # set motors once
    rightMotor.setVelocity(phirdot) # set motors once

    pass