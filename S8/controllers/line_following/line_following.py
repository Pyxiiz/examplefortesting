"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize ground sensors
gs=[]
for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)

#initialize motors and set positions + starting velocity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)
   
totalDistance=0
curOrientation=1.5708
xw=0
yw=0.028
alpha=1.5708
done = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    g=[]
    for gsensor in gs:
        g.append(gsensor.getValue())
        
    print(g)
    # Process sensor data here.
    if(g[0]>500 and g[1]<350 and g[2]>500): #drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED
    elif(g[2]<550): # turn right
        phildot, phirdot = 0.3 * MAX_SPEED, -0.1 * MAX_SPEED,
    elif(g[0]<550): # turn left
        phildot, phirdot = -0.1 * MAX_SPEED, 0.3 * MAX_SPEED,
    else:
        phildot, phirdot = 0.0, 0.0
    
    # Perform odometer calculations
    changeInDist = (.0201 * (phirdot + phildot))/2 * 32/1000
    changeInOrientation = (.0201 * (phirdot - phildot))/.052 * 32/1000
    
    totalDistance += changeInDist
    curOrientation += changeInOrientation
    
    xw=xw + np.cos(alpha) * changeInDist
    yw=yw + np.sin(alpha) * changeInDist
    omegaz = changeInOrientation
    alpha = alpha + omegaz
    error = np.sqrt(xw**2+yw**2)
    print(f"xw: {xw} ya: {yw} omegaz: {omegaz}")
    print(f"error from (0,0): {error}")
    
    # Enter here functions to send actuator commands:
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)

# Enter here exit cleanup code.
