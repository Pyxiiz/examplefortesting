from controller import Robot
import numpy as np

# constants
d_t          = 32 / 1000.0            # world tick length = 32ms
R            = 0.0201                 # epuck wheel radius 
d            = (0.026 + 0.005/2) * 2  # epuck wheel base (outer diameter)
WHEEL_SPEED  = 3.14 / 2               # epuck wheel speed (slow down to reduce inertia)
GS_THRESHOLD = 500                    # ground sensor threshold (detect white / black surface)

# init robot and world
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# init motors
motor_left  = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

# init ground sensors
gs_left   = robot.getDevice('gs0')
gs_center = robot.getDevice('gs1')
gs_right  = robot.getDevice('gs2')

gs_left.enable(timestep)
gs_center.enable(timestep)
gs_right.enable(timestep)

# init odometer vars
sum_delta_x = 0.0      # distance traveled, meters (odometer)
sum_delta_omega = 0.0  # rotation accumulated, radians 

# init estimated position vars
xw    = 0       # current pos X in world coords
yw    = 0.028   # current pos Y in world coords
alpha = 1.5708  # current rotation around Z axis in world coords (rad)

# FSM memory flag used to ignore all 3
#  sensors detecting black surface while turning 
is_turning = False  # False if stopped or driving straight

# control loop
while robot.step(timestep) != -1:

    # get readings from ground sensors
    gs_l_val = gs_left.getValue()
    gs_c_val = gs_center.getValue()
    gs_r_val = gs_right.getValue()
    
    # stop if not turning and all sensors 
    #  detect black surface (start line)
    if not is_turning \
      and gs_l_val < GS_THRESHOLD \
      and gs_c_val < GS_THRESHOLD \
      and gs_r_val < GS_THRESHOLD:
        phi_l_dot, phi_r_dot = 0, 0
    # drive straight if center sensor detects black surface, 
    #   while L and R sensors detect white surface
    elif gs_l_val > GS_THRESHOLD \
      and gs_c_val < GS_THRESHOLD \
      and gs_r_val > GS_THRESHOLD:
        is_turning = False
        phi_l_dot, phi_r_dot = WHEEL_SPEED, WHEEL_SPEED
    # turn right if right sensor detects black surface
    elif gs_r_val < GS_THRESHOLD:
        is_turning = True
        phi_l_dot = 0.25 * WHEEL_SPEED
        phi_r_dot = -0.1 * WHEEL_SPEED
    # turn left if left sensor detects black surface
    elif gs_l_val < GS_THRESHOLD:
        is_turning = True
        phi_l_dot = -0.1 * WHEEL_SPEED
        phi_r_dot = 0.25 * WHEEL_SPEED
        
    # log current sensor readings
    print(
        f'sensor readings:\n',
        f'gs_l_val = {gs_l_val}\n',
        f'gs_c_val = {gs_c_val}\n',
        f'gs_r_val = {gs_r_val}\n',
    )
    
    # log wheel speeds set in this tick and is_turning
    print(
        'wheel speeds:\n',
        f'phi_l_dot  = {phi_l_dot}\n',
        f'phi_r_dot  = {phi_r_dot}\n',
        f'is_turning = {is_turning}'
    )
        
    # set wheel speeds
    motor_left.setVelocity(phi_l_dot)
    motor_right.setVelocity(phi_r_dot)
    
    # calculate this tick's odometer change and rotation change
    delta_x     = (R * phi_l_dot + R * phi_r_dot) / 2 * d_t
    delta_omega = (R * phi_r_dot - R * phi_l_dot) / d * d_t 
    
    # update odometer and rotation tracker
    sum_delta_x     += delta_x
    sum_delta_omega += delta_omega
    
    # log odometry vals
    print(
        'odometry:\n',
        f'delta_x         (tick) \t\t= {delta_x}\n',
        f'delta_omega     (tick) \t\t= {delta_omega}\n',
        f'sum_delta_x     (odometer) \t\t= {sum_delta_x}\n',
        f'sum_delta_omega (accumulated rotation) \t= {sum_delta_omega}\n',
    )
    
    # update odometry based position estimate (in world coordinates)
    alpha += delta_omega
    xw    += np.cos(alpha) * delta_x
    yw    += np.sin(alpha) * delta_x

    # calculate estimated distance from 0,0
    est_dst_from_origin = np.sqrt(xw**2 + yw**2)
    
    # log estimated position in world coords
    print(
        'estimated position:\n',
        f'xw \t\t= {xw}\n',
        f'yw \t\t= {yw}\n',
        f'alpha (radians) \t= {alpha}\n'
        f' est dist from 0,0 \t= {est_dst_from_origin}\n===', 
    )
    

    