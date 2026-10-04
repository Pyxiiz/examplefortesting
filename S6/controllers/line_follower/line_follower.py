"""line_following controller."""

#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import matplotlib.pyplot as plt

# create the Robot instance.
robot = Robot()

#consts
RAD_TO_DEG=180/np.pi #DEG_TO_RAD=1/RAD_TO_DEG


##########################
##########################
### SET RUN values     ###
##########################
vel_init=3 # cm/s
vel_turn=vel_init*.25 # cm/s
n=1  # print state report every n seconds
time_MAX=92 #sec time to kill loop and plot
pose=[0,0.028,np.pi/2] # [m,m,rad ]init position
###########################
###########################
###########################





# inits
timestep = int(robot.getBasicTimeStep())
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
# plotting inits
time_history = []
pos_history = []
heading_history = []
# loop inits
start_flag=0
stop_flag=0
count=0
state_vec=['START']



#######################
###    FUNCTIONS    ####
##########################

def drive_forward(vel=5):
    leftMotor.setVelocity(vel)
    rightMotor.setVelocity(vel)
    return


def turn(dir='left',vel=1):
    turn_mult=.1
    if dir=='left':
        leftMotor. setVelocity(float(-vel* turn_mult))
        rightMotor.setVelocity(float( vel))
    if dir=='right':
        leftMotor. setVelocity(float( vel))
        rightMotor.setVelocity(float(-vel* turn_mult))    
    return


def find_line():

    # inits
    g = []
    g_line=[] #line 0 or 1
    tol=[600,700] #[max white, min dark]
    tol_straight=[350,500]
    
    # set sensor vals to true/false
    for gsensor in gs:
        g.append(gsensor.getValue())
    g_raw=g
    if len(g_raw) != 3: print ('ERROR find_line()- length g_raw')
    for i,val in enumerate(g_raw):
        if val<=max(tol): #dark, low, 1
            g_line.append(1)
        elif val>max(tol): #light, hi, 0
            g_line.append(0)
        else:
            print ('ERROR find_line() - val vs tol: val= '+str(val))
    # set to 'go straight' , reset for t/f values above
    if g_raw[0] > tol_straight[1] and g_raw[1] < tol_straight[0] and g_raw[2] > tol_straight[1]:
        g_line=[0,1,0] 
    if len(g_line) != 3: print ('ERROR find_line()- length g_line')
    return g_line
        
   
       
def get_dist_and_rot(vel_wheels,timestep,pose=[1000,1000,1000]):
    # get distance and angle values
    
    r_wheel=0.0205 #TODO get from device
    d_axis=0.052 #TODO get from device 
    RAD_TO_DEG=180/np.pi #DEG_TO_RAD=1/RAD_TO_DEG
    t_sec=timestep/1000 #ms to sec
    
    #calc dist and ang
    dist_trav=1/2*r_wheel*t_sec*(vel_wheels[1]+vel_wheels[0])
    ang_trav= r_wheel*t_sec*(vel_wheels[1]-vel_wheels[0]) / d_axis    
    ang_trav=ang_trav
    
    #get global position
    pose[2] += ang_trav
    pose[0] += dist_trav * np.cos(pose[2])
    pose[1] += dist_trav * np.sin(pose[2])
    # Euclidean error from origin (0,0)
    Pos_global = np.sqrt(pose[0]**2 + pose[1]**2)
    
    return dist_trav,ang_trav, pose, Pos_global
    
    
    
def plot_results(time_hist, pos_hist, heading_hist):
    #plotting #not part of assignment
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    ax1.plot(pos_hist[0], pos_hist[1], color='b', label='Global Position (cm)')
    ax1.set_ylabel('Position (cm)')
    #ax1.set_xlim(0, 300)
    #ax1.set_ylim(0, 300)
    ax1.grid(True)
    ax1.legend()

    ax2.plot(time_hist, heading_hist, color='r', label='Heading (deg)')
    ax2.set_xlabel('Time (sec)')
    ax2.set_ylabel('Heading (deg)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()       
        
        


gs = []
for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)




# Main loop:
while robot.step(timestep) != -1:
    #set time and pose
    time_sim=count*timestep/1000 #sec
    cur_line=find_line()
    vel_wheels=[leftMotor.getVelocity(),rightMotor.getVelocity()]
    dist_rot_pos=get_dist_and_rot(vel_wheels,timestep,pose)
    pose= dist_rot_pos[2]
    pos_global=dist_rot_pos[3]
    #for plotting
    time_history.append(time_sim)
    pos_history.append([pose[0]* 100,pose[1]* 100,pos_global * 100])
    heading_history.append(pose[2] * RAD_TO_DEG - 90)
    
    # choose state
    if stop_flag: state='STOP'
    elif cur_line==[0,1,0]: state='ONLINE'
    elif cur_line[0]: state='TURN_LEFT'
    elif cur_line[2]: state='TURN_RIGHT'
    elif cur_line==[0,0,0]: state='LOST_OFFLINE'
    elif cur_line==[1,1,1]: state='LOST_ONLINE'
    if  all(val=='LOST_ONLINE' for val in state_vec[-10:-1]): state='ON_START'

    #capture state after chosen
    last_turn = next((s for s in reversed(state_vec) if "TURN" in s), None)
    state_vec.append(state)
    
    
    #assign action from state
    if state=='ONLINE':
        drive_forward(vel=vel_init)
    if state=='TURN_LEFT':
        turn(dir='left',vel=vel_turn)
    if state=='TURN_RIGHT':
        turn(dir='right',vel=vel_turn)
    if state=='LOST_OFFLINE':state=last_turn
    if state=='ON_START':
        print('found start line')
        if not start_flag: time_past_start=time_sim+2
        start_flag=1
        if time_sim>time_past_start:state='START'
        
        state='STOP'
    if state=='STOP':
        drive_forward(vel=0) 

    
    # eval and outputs
    steps_str=timestep*n
    str_state='sim_state= ' + state
    str_time='simtime= '   + str(round(time_sim,2)) + ' sec'
    str_pos='global pos= ' + str(round(pos_global*100,2)) + ' cm'
    str_head='heading= '   + str(round(pose[2]*RAD_TO_DEG-90,2))+ ' deg'
    if count % steps_str == 0:
        print (str_time + '\t' + str_state)
        print ('  '  + str_pos)
        print ('  ' + str_head) 
        print (' ')
    
    count=count+1
    #plotting
    if time_sim>time_MAX:
        pos_hist_cols=[list(column) for column in zip(*pos_history)]
        #plot_results(time_history, pos_hist_cols, heading_history)
        break
    pass

