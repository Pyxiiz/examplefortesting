# NOTE: This is a constructed calibration fixture, not a real student
# submission. It is a copy of S9/controllers/line_following/line_following.py
# with every identifier renamed and nothing else changed - it tests
# rename-robustness (structural token canonicalization). See LABELS.json
# at the repo root for ground truth.

from controller import Robot
import numpy as np

# constants
dt_sec         = 32 / 1000.0            # world tick length = 32ms
WHEEL_R        = 0.0201                 # epuck wheel radius
WHEEL_BASE     = (0.026 + 0.005/2) * 2  # epuck wheel base (outer diameter)
BASE_SPEED     = 3.14 / 2               # epuck wheel speed (slow down to reduce inertia)
BLACK_THRESH   = 500                    # ground sensor threshold (detect white / black surface)

# init robot and world
bot = Robot()
step_ms = int(bot.getBasicTimeStep())

# init motors
m_left  = bot.getDevice('left wheel motor')
m_right = bot.getDevice('right wheel motor')

m_left.setPosition(float('Inf'))
m_right.setPosition(float('Inf'))

# init ground sensors
sensor_l = bot.getDevice('gs0')
sensor_c = bot.getDevice('gs1')
sensor_r = bot.getDevice('gs2')

sensor_l.enable(step_ms)
sensor_c.enable(step_ms)
sensor_r.enable(step_ms)

# init odometer vars
total_dist = 0.0  # distance traveled, meters (odometer)
total_rot  = 0.0  # rotation accumulated, radians

# init estimated position vars
pos_x   = 0       # current pos X in world coords
pos_y   = 0.028   # current pos Y in world coords
heading = 1.5708  # current rotation around Z axis in world coords (rad)

# FSM memory flag used to ignore all 3
#  sensors detecting black surface while turning
turning = False  # False if stopped or driving straight

# control loop
while bot.step(step_ms) != -1:

    # get readings from ground sensors
    sl_val = sensor_l.getValue()
    sc_val = sensor_c.getValue()
    sr_val = sensor_r.getValue()

    # stop if not turning and all sensors
    #  detect black surface (start line)
    if not turning \
      and sl_val < BLACK_THRESH \
      and sc_val < BLACK_THRESH \
      and sr_val < BLACK_THRESH:
        vL, vR = 0, 0
    # drive straight if center sensor detects black surface,
    #   while L and R sensors detect white surface
    elif sl_val > BLACK_THRESH \
      and sc_val < BLACK_THRESH \
      and sr_val > BLACK_THRESH:
        turning = False
        vL, vR = BASE_SPEED, BASE_SPEED
    # turn right if right sensor detects black surface
    elif sr_val < BLACK_THRESH:
        turning = True
        vL = 0.25 * BASE_SPEED
        vR = -0.1 * BASE_SPEED
    # turn left if left sensor detects black surface
    elif sl_val < BLACK_THRESH:
        turning = True
        vL = -0.1 * BASE_SPEED
        vR = 0.25 * BASE_SPEED

    # log current sensor readings
    print(
        f'sensor readings:\n',
        f'sl_val = {sl_val}\n',
        f'sc_val = {sc_val}\n',
        f'sr_val = {sr_val}\n',
    )

    # log wheel speeds set in this tick and turning
    print(
        'wheel speeds:\n',
        f'vL  = {vL}\n',
        f'vR  = {vR}\n',
        f'turning = {turning}'
    )

    # set wheel speeds
    m_left.setVelocity(vL)
    m_right.setVelocity(vR)

    # calculate this tick's odometer change and rotation change
    dx     = (WHEEL_R * vL + WHEEL_R * vR) / 2 * dt_sec
    domega = (WHEEL_R * vR - WHEEL_R * vL) / WHEEL_BASE * dt_sec

    # update odometer and rotation tracker
    total_dist += dx
    total_rot  += domega

    # log odometry vals
    print(
        'odometry:\n',
        f'dx              (tick) \t\t= {dx}\n',
        f'domega          (tick) \t\t= {domega}\n',
        f'total_dist      (odometer) \t\t= {total_dist}\n',
        f'total_rot       (accumulated rotation) \t= {total_rot}\n',
    )

    # update odometry based position estimate (in world coordinates)
    heading += domega
    pos_x   += np.cos(heading) * dx
    pos_y   += np.sin(heading) * dx

    # calculate estimated distance from 0,0
    dist_from_origin = np.sqrt(pos_x**2 + pos_y**2)

    # log estimated position in world coords
    print(
        'estimated position:\n',
        f'pos_x \t\t= {pos_x}\n',
        f'pos_y \t\t= {pos_y}\n',
        f'heading (radians) \t= {heading}\n'
        f' est dist from 0,0 \t= {dist_from_origin}\n===',
    )
