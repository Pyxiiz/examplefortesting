"""line_follow controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np
# create the Robot instance.
robot = Robot()
TIME_STEP = 32
MAX_SPEED = 6.28
totaldistance = 0
totalrotation = 0 
TURN_SPEED = MAX_SPEED/ 3
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
lspeed = 0
rspeed = 0
wheel_dist = .052
wheel_dia = .0201
deltaX = 0
deltaW = 0
cumulativeX = 0
xw= 0 
yw= 0
alpha = 0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
gs = [] 
for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())
            
   
    # Process sensor data here.
    #print(gsValues)
    if(gsValues[0] > 500 and gsValues[1] < 350 and gsValues[2] > 500):
        lspeed = MAX_SPEED
        rspeed = MAX_SPEED
    
    elif(gsValues[0]< 500): #turnleft
        lspeed = -TURN_SPEED /3
        rspeed = TURN_SPEED
    elif(gsValues[2] < 500): #turn right
        rspeed = -TURN_SPEED / 3
        lspeed = TURN_SPEED
    
    leftMotor.setVelocity(lspeed)
    rightMotor.setVelocity(rspeed)
    
    #timestep displacement X and omega
    deltaX = (((wheel_dia)*lspeed + (wheel_dia)*rspeed)/2)*(timestep/1000)
    deltaW = (((wheel_dia)*rspeed - (wheel_dia)*lspeed)/(wheel_dist))*(timestep/1000)
    
    #cumulative displacement X and Angle alpha
    cumulativeX = cumulativeX + deltaX
    alpha = alpha + deltaW
    
    #world coordinate transformation. 
    xw += np.cos(alpha)*deltaX
    yw += np.sin(alpha)*deltaX
    
    print('X: ', xw, 'Y: ', yw, 'Error: ', np.sqrt(xw**2 + yw**2), 'Angle: ', alpha*(180/math.pi))
    pass

# Enter here exit cleanup code.
