from controller import Robot
import math

# ==============================
# CONSTANTS
# ==============================
WHEEL_RADIUS = 0.0205      # meters
AXLE_LENGTH = 0.052        # meters
LINE_THRESHOLD = 500       # black line threshold

# ==============================
# INITIALIZATION
# ==============================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ==============================
# MOTORS
# ==============================
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ==============================
# GROUND SENSORS
# ==============================
gs = []

for i in range(3):
    sensor = robot.getDevice(f"gs{i}")
    if sensor is None:
        print(f"ERROR: gs{i} not found. Make sure groundSensors TRUE in e-puck.")
        exit()
    sensor.enable(timestep)
    gs.append(sensor)

# ==============================
# INITIAL POSE (match Scene Tree)
# ==============================
xw = 0.0
yw = 0.028
theta = 0.0

print("Odometry + Line Following Started")

# ==============================
# MAIN LOOP
# ==============================
while robot.step(timestep) != -1:

    # ---- READ GROUND SENSORS ----
    gs_values = [sensor.getValue() for sensor in gs]

    # ---- LINE FOLLOWING ----
    left_speed = 3.0
    right_speed = 3.0

    if gs_values[1] < LINE_THRESHOLD:
        # center on line
        left_speed = 3.0
        right_speed = 3.0
    elif gs_values[0] < LINE_THRESHOLD:
        # drifted right -> turn left
        left_speed = 1.0
        right_speed = 3.0
    elif gs_values[2] < LINE_THRESHOLD:
        # drifted left -> turn right
        left_speed = 3.0
        right_speed = 1.0

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # ==============================
    # ODOMETRY CALCULATION
    # ==============================
    v_l = left_speed * WHEEL_RADIUS
    v_r = right_speed * WHEEL_RADIUS

    v = (v_r + v_l) / 2.0
    omega = (v_r - v_l) / AXLE_LENGTH

    dt = timestep / 1000.0

    # Update pose
    xw += v * math.cos(theta) * dt
    yw += v * math.sin(theta) * dt
    theta += omega * dt

    # ==============================
    # ERROR FROM ORIGIN
    # ==============================
    error = math.sqrt(xw**2 + yw**2)

    print(
        "x:", round(xw, 3),
        "y:", round(yw, 3),
        "theta:", round(theta, 3),
        "error:", round(error, 3)
    )
