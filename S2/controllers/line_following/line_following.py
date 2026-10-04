# FSM_controller

# import the Robot class
from controller import Robot
import numpy as np

# constants
# some of the following are just for experimentation
TIME_STEP = 32 # time in [ms] of a simulation step
n = 1 # speed reduction factor
REDUCE_THE_MOTOR = 0.1 # coefficient for the speed during turn or low speed
MAX_SPEED = 5/n # maximum speed of the motors in [rad/s]
SPEED = MAX_SPEED # actual max speed of the motors in [rad/s]
ON_LINE_THRESHOLD = 700 # threshold for the line sensors
OFF_LINE_THRESHOLD = 700 # threshold for the line sensors
LOST_LINE_COUNT = int(0) # counter for the lost line events 
FINISH_LINE_THRESHOLD = 50 # threshold for the finish line events
WHEEL_RADIUS = 0.0201 # radius of the wheels in [m]
WHEEL_BASE = 0.052 # base of the wheels in [m]
ODOMETER = 0 # odometer in [m]
HDG = 0 # heading in [rad] referenced to the initial HDG
xw = 0 # x position in [m]
yw = 0 # y position in [m]
alpha = np.pi/2 # heading in [rad] referenced to the initial alpha world frame
RED_CIRCLE = "🔴"#red circle emoji
GREEN_CIRCLE = "💚"#green circle emoji


# list of the ground sensors devices
gs_sensor_list=['gs0', 'gs1', 'gs2']

# list of the led devices
led_list=['led7', 'led0', 'led1','led8','led9']

# create the Robot instance.
robot = Robot()

# list of the position sensors devices
gs=[] 

# list of the led devices
led=[]

# enable the position sensors devices
for i in range(len(gs_sensor_list)):
    gs.append(robot.getDevice(gs_sensor_list[i]))
    gs[i].enable(TIME_STEP)

# set the led devices, 0,1,2 are the left, center, right sensors and 3,4 are the lost line sensors
for i in range(len(led_list)):
    led.append(robot.getDevice(led_list[i]))

# get the motors devices
leftMotor = robot.getDevice('left wheel motor') # get the left motor device
rightMotor = robot.getDevice('right wheel motor') # get the right motor device
leftMotor.setPosition(float('inf')) # set the left motor to infinite position
rightMotor.setPosition(float('inf')) # set the right motor to infinite position
leftMotor.setVelocity(0.0) # set the left motor to 0 velocity
rightMotor.setVelocity(0.0) # set the right motor to 0 velocity

#FSM States: the STATES for this task and their initial state
CURRENT_STATE = 'STATE_FOLLOW_LINE'
#print('follow line')
PREVIOUS_STATE = 'None'    


#FSM ACTIONS: the actions available to execute the task
ACTION_FOLLOW_LINE = True
ACTION_LOST_LINE = False
ACTION_STOP = False

#FSM Events
EVENT_LINE_LEFT = False
EVENT_LINE_CENTER = False
EVENT_LINE_RIGHT = False
EVENT_LOST_LINE = False 


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
   
   #-------------------------#
   # SENSE --> THINK --> ACT #
   #-------------------------#

    PREVIOUS_STATE = CURRENT_STATE # save the previous state
    ACTION_FOLLOW_LINE = False
    ACTION_LOST_LINE = False
    ACTION_STOP = False 

    EVENT_LOST_LINE = False 
    EVENT_LINE_LEFT = False
    EVENT_LINE_CENTER = False
    EVENT_LINE_RIGHT = False        


    #-------#
    # SENSE #
    #-------#

    # read the ground sensors values
    gsValues = []
    for i in range(len(gs_sensor_list)):
        gsValues.append(gs[i].getValue())

    LEFT_SENSOR_ON_THE_LINE = gsValues[0] < ON_LINE_THRESHOLD
    CENTER_SENSOR_ON_THE_LINE = gsValues[1] < ON_LINE_THRESHOLD
    RIGHT_SENSOR_ON_THE_LINE = gsValues[2] < ON_LINE_THRESHOLD
    LEFT_SENSOR_OFF_THE_LINE = gsValues[0] > OFF_LINE_THRESHOLD
    CENTER_SENSOR_OFF_THE_LINE = gsValues[1] > OFF_LINE_THRESHOLD
    RIGHT_SENSOR_OFF_THE_LINE = gsValues[2] > OFF_LINE_THRESHOLD

    # set the led values
    # it turns on the led if the sensor is on the line
    # it turns off the led if the sensor is off the line
    if LEFT_SENSOR_ON_THE_LINE:
        led[0].set(1)
    if CENTER_SENSOR_ON_THE_LINE:
        led[1].set(1)
    if RIGHT_SENSOR_ON_THE_LINE:
        led[2].set(1)
    if LEFT_SENSOR_OFF_THE_LINE:
        led[0].set(0)
    if CENTER_SENSOR_OFF_THE_LINE:
        led[1].set(0)
    if RIGHT_SENSOR_OFF_THE_LINE:
        led[2].set(0)

    emoji_display = []
    for i in range(len(gsValues)):
        sensor_on_the_line = gsValues[i] < ON_LINE_THRESHOLD
        emoji_display.append(f"{RED_CIRCLE}" if not sensor_on_the_line else f"{GREEN_CIRCLE}")
    #print(f"{emoji_display}")
    #print(f"gsValues {gsValues}")

    #TRANSITIONS
    EVENT_LINE_LEFT = (LEFT_SENSOR_ON_THE_LINE or (LEFT_SENSOR_ON_THE_LINE and CENTER_SENSOR_ON_THE_LINE)) and  RIGHT_SENSOR_OFF_THE_LINE # line left
    EVENT_LINE_CENTER = CENTER_SENSOR_ON_THE_LINE and LEFT_SENSOR_OFF_THE_LINE and RIGHT_SENSOR_OFF_THE_LINE # line center
    EVENT_LINE_RIGHT = (RIGHT_SENSOR_ON_THE_LINE or (RIGHT_SENSOR_ON_THE_LINE and CENTER_SENSOR_ON_THE_LINE)) and LEFT_SENSOR_OFF_THE_LINE # line right
    
    EVENT_LOST_LINE = (LEFT_SENSOR_OFF_THE_LINE and CENTER_SENSOR_OFF_THE_LINE and RIGHT_SENSOR_OFF_THE_LINE) or (LEFT_SENSOR_ON_THE_LINE and CENTER_SENSOR_ON_THE_LINE and RIGHT_SENSOR_ON_THE_LINE) # lost line because all sensors are on the line or off the line
    
    EVENT_ALL_ON_THE_LINE = (LEFT_SENSOR_ON_THE_LINE and CENTER_SENSOR_ON_THE_LINE and RIGHT_SENSOR_ON_THE_LINE) # all sensors are on the line
    EVENT_ALL_OFF_THE_LINE = (LEFT_SENSOR_OFF_THE_LINE and CENTER_SENSOR_OFF_THE_LINE and RIGHT_SENSOR_OFF_THE_LINE) # all sensors are off the line
    
    #print the events in different colors on the same line
    #print(f"EVENT_LINE_LEFT {EVENT_LINE_LEFT}", f"EVENT_LINE_CENTER {EVENT_LINE_CENTER}", f"EVENT_LINE_RIGHT {EVENT_LINE_RIGHT}", f"EVENT_LOST_LINE {EVENT_LOST_LINE}", f"EVENT_ALL_ON_THE_LINE {EVENT_ALL_ON_THE_LINE}", f"EVENT_ALL_OFF_THE_LINE {EVENT_ALL_OFF_THE_LINE}")
    #-------#
    # THINK #
    #-------#

    
    if EVENT_LOST_LINE: # transition from follow line to lost line
        LOST_LINE_COUNT += 1 
        CURRENT_STATE = 'STATE_LOST_LINE' # change the state to LOST_LINE (TRANSITION)
        
        #if PREVIOUS_STATE != CURRENT_STATE:
            #print('Lost Line 🔴')
        
        if LOST_LINE_COUNT < FINISH_LINE_THRESHOLD: # if the lost line count is less than the finish line threshold
            #print('LOST_LINE_COUNT 🔴', LOST_LINE_COUNT,'speed', SPEED) # print the lost line count and the speed
            ACTION_LOST_LINE = True # set the action to lost line
        
        if LOST_LINE_COUNT == FINISH_LINE_THRESHOLD: # transition from lost line to finished if the lost line count is equal to the finish line threshold
            CURRENT_STATE = 'STATE_FINISHED' # change the state to FINISHED (TRANSITION)
            #print('Finished 🔴🔴🔴🔴🔴')   
    
    elif EVENT_LINE_LEFT or EVENT_LINE_CENTER or EVENT_LINE_RIGHT: # transition from lost line to follow line if the robot is on the line
        CURRENT_STATE = 'STATE_FOLLOW_LINE' # change the state to FOLLOW_LINE (TRANSITION)
        LOST_LINE_COUNT = 0 # reset the lost line count
        #if PREVIOUS_STATE != CURRENT_STATE: # if the previous state is different from the current state
            #print('Following Line 💚', 'previous state = ', PREVIOUS_STATE, 'current state = ', CURRENT_STATE) # print the current state

    if CURRENT_STATE == 'STATE_FINISHED': # if the current state is FINISHED
        ACTION_STOP = True # set the action to stop
    elif CURRENT_STATE == 'STATE_LOST_LINE': # if the current state is LOST_LINE
        ACTION_LOST_LINE = True # set the action to lost line
        led[3].set(1)
        led[4].set(1)
    elif CURRENT_STATE == 'STATE_FOLLOW_LINE': # if the current state is FOLLOW_LINE
        ACTION_FOLLOW_LINE = True # set the action to follow line
        SPEED = MAX_SPEED # set the speed to maximum speed
        led[3].set(0)
        led[4].set(0)

    #print('CURRENT_STATE', CURRENT_STATE, '-- ACTION_FOLLOW_LINE', ACTION_FOLLOW_LINE, '-- ACTION_LOST_LINE', ACTION_LOST_LINE, '-- ACTION_STOP', ACTION_STOP)
    #-----#
    # ACT #
    #-----#

    if ACTION_STOP: # if the action is STOP 
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)   

    elif ACTION_LOST_LINE: # if the action is LOST_LINE
        if EVENT_ALL_ON_THE_LINE: # if the event is ALL_ON_THE_LINE
                                # it reduces the speed of the motors to better
                                # sense when the line is lost
            leftMotor.setVelocity(REDUCE_THE_MOTOR*SPEED)
            rightMotor.setVelocity(REDUCE_THE_MOTOR*SPEED)
            dX = (REDUCE_THE_MOTOR*SPEED)*TIME_STEP/1000*WHEEL_RADIUS
            dOmegaZ = 0
        elif EVENT_ALL_OFF_THE_LINE: # if the event is ALL_OFF_THE_LINE
                                    # it turns right to search the line
            leftMotor.setVelocity(SPEED)
            rightMotor.setVelocity(REDUCE_THE_MOTOR*SPEED)
            dX = (SPEED*TIME_STEP/1000*WHEEL_RADIUS+(REDUCE_THE_MOTOR*SPEED)*TIME_STEP/1000*WHEEL_RADIUS)/2
            dOmegaZ = (((REDUCE_THE_MOTOR*SPEED-SPEED))*WHEEL_RADIUS)/WHEEL_BASE*TIME_STEP/1000

    elif ACTION_FOLLOW_LINE: # if the action is FOLLOW_LINE

        if EVENT_LINE_LEFT: # if the event is LEFT_LINE
            leftMotor.setVelocity(REDUCE_THE_MOTOR*SPEED)
            rightMotor.setVelocity(SPEED)
            dX = ((REDUCE_THE_MOTOR*SPEED)*TIME_STEP/1000*WHEEL_RADIUS+(SPEED)*TIME_STEP/1000*WHEEL_RADIUS)/2
            dOmegaZ = (((SPEED-REDUCE_THE_MOTOR*SPEED))*WHEEL_RADIUS)/WHEEL_BASE*TIME_STEP/1000
        
        elif EVENT_LINE_RIGHT: # if the event is RIGHT_LINE
            leftMotor.setVelocity(SPEED)
            rightMotor.setVelocity(REDUCE_THE_MOTOR*SPEED)
            dX = ((SPEED)*TIME_STEP/1000*WHEEL_RADIUS+(REDUCE_THE_MOTOR*SPEED)*TIME_STEP/1000*WHEEL_RADIUS)/2
            dOmegaZ = (((REDUCE_THE_MOTOR*SPEED-SPEED))*WHEEL_RADIUS)/WHEEL_BASE*TIME_STEP/1000
        
        elif EVENT_LINE_CENTER: # if the event is CENTER_LINE
            leftMotor.setVelocity(SPEED)
            rightMotor.setVelocity(SPEED)
            dX = (SPEED)*TIME_STEP/1000*WHEEL_RADIUS
            dOmegaZ = 0

    
        # odometer formulas:
        # dX_straight = (SPEED)*TIME_STEP/1000*WHEEL_RADIUS
        # dX_turns = ((SPEEDleft)*TIME_STEP/1000*WHEEL_RADIUS + (SPEEDright)*TIME_STEP/1000*WHEEL_RADIUS)/2
        # dOmegaZ = ((SPEED)*TIME_STEP/1000*WHEEL_RADIUS-(0.25*SPEED)*TIME_STEP/1000*WHEEL_RADIUS)/WHEEL_BASE

    if ACTION_STOP == False: # if the action is not STOP
        ODOMETER += dX # update the odometer
        HDG += dOmegaZ # update the heading in radians
        alpha += dOmegaZ # update the world heading in radians
        HDGdeg = HDG * 180 / np.pi # convert the heading in radians to degrees
        xw = xw + np.cos(alpha)*dX # update the x position
        yw = yw + np.sin(alpha)*dX # update the y position
        #print('gs0 = ', gsValues[0] , 'gs1 = ', gsValues[1], 'gs2 = ', gsValues[2]) # print the sensor values
        #print('current state = ', CURRENT_STATE, '-- EVENT_LOST_LINE = ', EVENT_LOST_LINE, 'EVENT_LINE_LEFT = ', EVENT_LINE_LEFT, 'EVENT_LINE_CENTER = ', EVENT_LINE_CENTER, 'EVENT_LINE_RIGHT = ', EVENT_LINE_RIGHT) # print the current state
        print('ODOMETER = ', ODOMETER, 'alpha = ', alpha,'xw = ', xw, 'yw = ', yw) # print the odometer
    
    else:
        TOTAL_ERROR = np.sqrt(xw**2+yw**2) # calculate the total error
        print('TOTAL_ERROR = ', TOTAL_ERROR) # print the total error
        print('Simulation finished') # print the simulation finished message
        break # stop the simulation

    #xw=xw+np.cos(alpha)*dX
    #yw=yw+np.sin(alpha)*dX
    #alpha=alpha+dOmegaZ
    #print(np.sqrt(xw**2+yw**2))
    #print(alpha)