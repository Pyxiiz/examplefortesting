"""
odometry_controller.py
======================
Assignment 2 -- line following + wheel-encoder odometry for the e-puck.

WHAT THIS CONTROLLER DOES
-------------------------
1.  Follows a closed black line painted on the floor, using the e-puck's three
    downward-facing infra-red ground sensors (gs0 left, gs1 centre, gs2 right).
2.  While it drives, it integrates the two wheel ENCODERS -- not the commanded
    wheel speeds -- into an estimate of the robot's pose in WORLD coordinates:
        xw      position along the world x axis  [m]
        yw      position along the world y axis  [m]
        omegaz  heading, i.e. rotation about the world z axis  [rad]
3.  When one full lap has been driven the robot stops and prints the Euclidean
    distance of its own position estimate from the world origin,
    sqrt(xw**2 + yw**2).  The origin is where the start line crosses the track,
    so that number is the odometry error the assignment is graded on.

Encoders are used rather than the commanded velocities on purpose: the
commanded velocity is what we ASKED the wheel to do, the encoder is what the
wheel actually DID.  They differ every time the motor saturates, accelerates or
slips, and those differences are exactly the errors odometry exists to capture.

COORDINATE SYSTEM
-----------------
Webots R2023b defaults to ENU for the world (+x east, +y north, +z up) and FLU
for robots (+x forward, +y left, +z up).  The e-puck is placed in the world at
(x = 0, y = 0.028) rotated +90 deg about z, so it starts on the line facing
north.  The odometry state is therefore initialised to exactly that pose --
the brief warns specifically about forgetting this, and starting from (0, 0, 0)
would rotate the whole estimated trajectory by 90 deg.
"""

import numpy as np
from controller import Robot

# ===========================================================================
#  ROBOT CONSTANTS -- taken from E-puck.proto (Webots R2023b), not guessed
# ===========================================================================
WHEEL_RADIUS = 0.020        # [m]  `radius 0.02` on both wheel HingeJoints.
                            #      Confirmed by measurement: driving 0.300 m in
                            #      a straight line gave r = 0.020004 m.

AXLE_GEOMETRIC = 0.052      # [m]  the wheel HingeJoints are anchored at
                            #      y = +0.026 and y = -0.026, so the spacing of
                            #      the wheel CENTRE PLANES is 0.052.  This is
                            #      the number you get by reading the proto, and
                            #      it is NOT the right one to use -- see below.

AXLE_LENGTH = 0.0570        # [m]  the effective track width, MEASURED.
# This "L" converts a wheel-speed difference into a yaw rate, so an error in it
# scales every single turn the robot makes.  Using the geometric 0.052 made the
# odometry over-estimate every turn by 0.0570/0.052 = 9.4%; over a lap, which
# contains a full 2*pi of rotation, that is a 34 degree heading error and about
# 0.24 m of position error -- a failing result, and the single largest error
# source in this assignment.
#
# Why the two differ: each wheel's collision shape is a Cylinder of radius 0.02
# and height 0.005 centred at y = +/-0.026, so its outer rim lies at
# y = +/-0.0285.  The physics engine generates the dominant wheel/ground contact
# out at that rim rather than on the centre plane, making the real track width
# 2 * 0.0285 = 0.0570 m.
#
# This was measured rather than assumed, by driving known manoeuvres and
# comparing the encoders against the simulator's true pose
# (L_eff = r * (dphi_r - dphi_l) / true heading change):
#
#     spin in place, both directions   0.05679, 0.05681
#     gentle arc, left and right       0.05688, 0.05699
#     very gentle arc                  0.05707
#
# Consistent to better than 0.5% across manoeuvres, which is what makes it a
# calibration rather than a fudge factor.  Calibrating the effective track width
# against measured motion is standard practice on real differential drives too
# (it is what the UMBmark procedure does).
MAX_SPEED = 6.28            # [rad/s]  `maxVelocity 6.28` in the proto.

# ===========================================================================
#  STARTING POSE -- must match the scene tree in worlds/line_following.wbt
# ===========================================================================
X0 = 0.0                    # [m]     E-puck translation x
Y0 = 0.028                  # [m]     E-puck translation y ("slightly in front
                            #         of the start line")
OMEGAZ0 = np.pi / 2.0       # [rad]   E-puck rotation is `0 0 1 1.5707963`,
                            #         i.e. +90 deg about z: the robot's forward
                            #         axis points along world +y (north).

# ===========================================================================
#  TRACK CONSTANT -- printed by tools/make_track.py when the texture is built
# ===========================================================================
TRACK_LENGTH = 4.484956     # [m]  centreline length of the closed loop:
                            #      2 x 1.00 m + 2 x 0.30 m straights, plus four
                            #      quarter-circles of radius 0.30 m which add up
                            #      to one full circle (2*pi*0.30 = 1.885 m).

# Distance the robot must travel to come to rest ON the start line.  The robot
# starts 0.028 m PAST the origin, so going all the way round the loop and back
# to the origin is one lap minus that 0.028 m.  Stopping here rather than at the
# start pose is what makes the printed sqrt(xw**2 + yw**2) a direct measurement
# of the localisation error: the robot's true position is then (0, 0), so any
# non-zero reading is purely odometry drift.
LAP_DISTANCE = TRACK_LENGTH - 0.028   # 4.456956 m

# ===========================================================================
#  GROUND-SENSOR CALIBRATION
# ===========================================================================
# The e-puck ground sensors are DistanceSensors of type "infra-red": Webots
# scales their reading by the reflectance of whatever the beam lands on, so a
# BLACK line reads LOW and the WHITE floor reads HIGH.
#
# These levels are MEASURED, not assumed.  Spinning the robot in place on this
# track and logging the raw values gave 296-302 over the line and 758-775 over
# the floor.  (300 is also the far end of the sensor's lookupTable in
# E-puckGroundSensors.proto, `[0 1000 0.002, 0.016 300 0.004]`.)
#
# Rather than a single hard threshold, each reading is mapped onto a normalised
# 0.0 (definitely black) .. 1.0 (definitely white) scale with the two levels
# below, set just INSIDE the measured extremes so ordinary readings saturate
# cleanly at 0.0 and 1.0.  A linear ramp, instead of a step, is what turns
# three sensors into a usable PROPORTIONAL error signal: the anti-aliased edge
# of the painted line makes a sensor near the edge return an intermediate
# value, and that intermediate value is the fine steering information.
GS_BLACK = 350.0            # reading at or below this counts as fully black
GS_WHITE = 700.0            # reading at or above this counts as fully white
LINE_LEVEL = 0.5            # normalised value below which a sensor "sees line"
                            # (equivalent to a raw threshold of about 525)

# ===========================================================================
#  CONTROL GAINS
# ===========================================================================
BASE_SPEED = 2.5            # [rad/s] cruise wheel speed = 0.05 m/s.  Well under
                            # the 6.28 limit: leaving headroom means the
                            # steering correction is never clipped, and a slower
                            # robot slips less, which directly buys odometry
                            # accuracy.
KP = 0.6                    # [rad/s per unit error] proportional steering gain.
                            # The sensors are single-ray, so the error signal is
                            # close to bang-bang: it swings to +/-1 within about
                            # a millimetre of drift.  The gain therefore sets
                            # the radius the robot turns at while correcting:
                            #   wheel difference = 2 * KP        = 1.2 rad/s
                            #   yaw rate         = 1.2 * r / L   = 0.46 rad/s
                            #   turn radius      = v / yaw rate  = 0.11 m
                            # That is comfortably tighter than the track's
                            # 0.30 m corners, so the robot always has the
                            # authority to hold a corner, but it is gentle
                            # enough not to zig-zag on the straights.  KP = 2
                            # gives a 0.03 m correction radius, which scrubs the
                            # tyres, and slip is invisible to the encoders and
                            # therefore turns straight into odometry error.
KD = 1.0                    # derivative gain, acting on the per-step change in
                            # error.  Pure P control on a line follower rings:
                            # the sensors are 3 cm ahead of the wheel axle, so
                            # there is lag between a correction and its effect.
                            # A small D term damps that overshoot.
CORNER_SLOWDOWN = 0.45      # fraction of cruise speed given up at full steering
                            # deflection.  Slowing into corners keeps the tyres
                            # inside their friction budget -- wheel slip is
                            # invisible to the encoders and is therefore pure,
                            # unrecoverable odometry error.

# ===========================================================================
#  LAP DETECTION
# ===========================================================================
LAP_ARM_FRACTION = 0.80     # the lap detector only arms after 80 % of a lap, so
                            # that "I am near the origin" at the very start
                            # cannot be mistaken for "I have come all the way
                            # back round".
START_RADIUS = 0.05         # [m] how close the ESTIMATE has to be to the origin
                            # for the run to count as "back at the start line".
                            # NOTE: this is checked and reported, but it is NOT
                            # what stops the robot -- see the comment at the lap
                            # test below.
MAX_SIM_TIME = 400.0        # [s] hard stop, so a run that loses the line still
                            # terminates and still reports an honest number.

PRINT_EVERY = 50            # print telemetry every 50 control steps (0.8 s at a
                            # 16 ms timestep).  Printing every step floods the
                            # console with thousands of unreadable lines.


def main():
    # -----------------------------------------------------------------------
    # Webots boilerplate
    # -----------------------------------------------------------------------
    robot = Robot()                                  # handle on this robot node
    timestep = int(robot.getBasicTimeStep())         # ms; matches WorldInfo
    dt = timestep / 1000.0                           # same thing in seconds

    # --- motors ------------------------------------------------------------
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    # setPosition(inf) takes the motor out of position control and puts it into
    # pure VELOCITY control, which is what setVelocity() then commands.
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)                      # start stationary
    right_motor.setVelocity(0.0)

    # --- wheel encoders ----------------------------------------------------
    # PositionSensors report the accumulated wheel angle in radians.  They must
    # be enabled with a sampling period before they return anything but NaN.
    left_enc = robot.getDevice('left wheel sensor')
    right_enc = robot.getDevice('right wheel sensor')
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    # --- ground sensors ----------------------------------------------------
    # Supplied by the E-puckGroundSensors node in the world's groundSensorsSlot.
    # gs0 = left (y = +0.010), gs1 = centre (y = 0), gs2 = right (y = -0.010),
    # all 3 cm ahead of the wheel axle and 3.3 mm above the floor.  Each is a
    # single-ray infra-red DistanceSensor, so it samples a point on the texture.
    gs = []
    for name in ('gs0', 'gs1', 'gs2'):
        sensor = robot.getDevice(name)
        if sensor is None:
            raise RuntimeError(
                "Ground sensor '%s' not found. The E-puck node in the world "
                "needs 'groundSensorsSlot [ E-puckGroundSensors { } ]'." % name)
        sensor.enable(timestep)
        gs.append(sensor)

    # One warm-up step so every enabled device has produced its first sample.
    # Before this, getValue() returns NaN and the first encoder difference
    # would poison the odometry with a NaN that never washes out.
    robot.step(timestep)

    def read_encoder(sensor):
        """Encoder angle in radians, with NaN mapped to 0.0 for safety."""
        value = sensor.getValue()
        return 0.0 if np.isnan(value) else value

    prev_phi_l = read_encoder(left_enc)               # previous left wheel angle
    prev_phi_r = read_encoder(right_enc)              # previous right wheel angle

    # -----------------------------------------------------------------------
    # Odometry state, initialised from the ACTUAL scene-tree pose.
    # -----------------------------------------------------------------------
    xw = X0
    yw = Y0
    omegaz = OMEGAZ0
    path_length = 0.0        # [m] total distance travelled, used for lap timing

    # Controller state
    prev_err = 0.0           # previous steering error, for the derivative term
    last_err = 0.0           # last error seen while the line was actually
                             # visible, used to decide which way to search if
                             # the line is lost
    step_count = 0
    finished = False
    stop_reason = ""

    print("", flush=True)
    print("e-puck odometry controller started", flush=True)
    print("  timestep      : %d ms" % timestep, flush=True)
    print("  initial pose  : xw = %+.3f m  yw = %+.3f m  omegaz = %+.3f rad"
          % (xw, yw, omegaz), flush=True)
    print("  lap length    : %.3f m" % TRACK_LENGTH, flush=True)
    print("  stopping after: %.3f m (one lap back round to the start line)"
          % LAP_DISTANCE, flush=True)
    print("", flush=True)

    # =======================================================================
    #  MAIN CONTROL LOOP
    # =======================================================================
    # robot.step() advances the simulation by one timestep and returns -1 when
    # Webots wants the controller to quit.
    while robot.step(timestep) != -1:
        step_count += 1
        sim_time = robot.getTime()                    # [s] simulated clock

        # ===================================================================
        #  1. ODOMETRY -- integrate the encoders
        # ===================================================================
        phi_l = read_encoder(left_enc)                # [rad] wheel angle now
        phi_r = read_encoder(right_enc)
        dphi_l = phi_l - prev_phi_l                   # [rad] turned this step
        dphi_r = phi_r - prev_phi_r
        prev_phi_l = phi_l
        prev_phi_r = phi_r

        # Wheel angle -> distance rolled by that wheel along the floor.
        # Rolling without slipping: arc = radius * angle.
        ds_l = WHEEL_RADIUS * dphi_l                  # [m]
        ds_r = WHEEL_RADIUS * dphi_r                  # [m]

        # Differential-drive kinematics.  The centre of the axle advances by the
        # mean of the two wheels; the robot yaws by their difference divided by
        # the axle length (the two wheels trace concentric arcs about the
        # instantaneous centre of rotation, and the difference in arc length
        # over the radius difference L is the swept angle).
        ds = 0.5 * (ds_r + ds_l)                      # [m]   forward increment
        dtheta = (ds_r - ds_l) / AXLE_LENGTH          # [rad] heading increment

        # Project the forward increment into world x and y.
        #
        # WHY  omegaz + dtheta/2  AND NOT JUST  omegaz :
        # Over one timestep the robot does not travel in a straight line at its
        # starting heading -- it sweeps a small arc from omegaz to
        # omegaz + dtheta.  Using the starting heading (the naive first-order
        # Euler form) systematically places every step on the outside of the
        # turn, and because a closed loop is turning almost continuously those
        # one-sided errors ADD UP instead of cancelling: on this 4.5 m lap, with
        # its 2*pi of total rotation, that alone is worth several centimetres.
        # Evaluating the heading at the MIDPOINT of the step is the midpoint
        # rule, second-order accurate, and it is most of what buys the sub-20 cm
        # result.  (The exact closed form uses the chord of the arc, but for
        # dtheta of order 1e-3 rad per step the midpoint form is identical to
        # well within floating point noise.)
        xw += ds * np.cos(omegaz + dtheta / 2.0)      # [m]
        yw += ds * np.sin(omegaz + dtheta / 2.0)      # [m]
        omegaz += dtheta                              # [rad]

        # Wrap the heading back into [-pi, pi].  atan2(sin, cos) is the standard
        # branch-free way to do it: it round-trips the angle through the unit
        # circle, so it cannot drift the way repeated "+= 2pi" tests can, and it
        # keeps the printed heading readable after several laps.
        omegaz = np.arctan2(np.sin(omegaz), np.cos(omegaz))

        # Distance travelled so far.  abs() so that reversing (during a line
        # search, say) still counts as distance covered rather than unwinding
        # the lap counter.
        path_length += abs(ds)

        # ===================================================================
        #  2. LINE FOLLOWING -- proportional-derivative steering
        # ===================================================================
        raw = [s.getValue() for s in gs]              # gs0, gs1, gs2 raw values

        # Normalise to 0.0 = black line, 1.0 = white floor.
        # np.clip keeps the value in range when a reading falls outside the
        # calibrated band (e.g. over the anti-aliased edge of the line).
        n = [float(np.clip((v - GS_BLACK) / (GS_WHITE - GS_BLACK), 0.0, 1.0))
             for v in raw]

        # The line is "visible" if any sensor is darker than the mid level.
        line_visible = min(n) < LINE_LEVEL

        # Steering error = right sensor minus left sensor.
        #   robot drifted RIGHT of the line -> the line lies to its LEFT
        #     -> gs0 (left) goes black (n0 -> 0), gs2 (right) stays white
        #     -> err = n2 - n0 > 0  -> steer LEFT (right wheel faster).
        # The sign therefore works out so that a POSITIVE error means
        # "turn left / counter-clockwise", which is also the positive direction
        # of omegaz.
        err = n[2] - n[0]

        if line_visible:
            # Remember the last meaningful direction the line was seen in.  The
            # 0.05 dead band stops tiny noise from overwriting a genuine
            # "the line went off to the right" memory.
            if abs(err) > 0.05:
                last_err = err
            base = BASE_SPEED * (1.0 - CORNER_SLOWDOWN * min(abs(err), 1.0))
        else:
            # Line lost: all three sensors are on white.  Pivot back towards
            # whichever side the line was last seen on, at reduced speed, until
            # a sensor picks it up again.  np.sign gives full deflection.
            err = float(np.sign(last_err))
            base = BASE_SPEED * 0.4

        # PD steering law.  The derivative acts on the change in error per
        # control step (dividing by dt as well would only rescale KD).
        d_err = err - prev_err
        prev_err = err
        steer = KP * err + KD * d_err                 # [rad/s] wheel differential

        # Differential drive: subtract the steer term from the left wheel and
        # add it to the right to turn left, and clip to the motor's limits.
        v_left = float(np.clip(base - steer, -MAX_SPEED, MAX_SPEED))
        v_right = float(np.clip(base + steer, -MAX_SPEED, MAX_SPEED))

        # ===================================================================
        #  3. LAP DETECTION
        # ===================================================================
        dist_to_start = np.sqrt(xw ** 2 + yw ** 2)    # [m] estimate vs. origin

        # armed: at least 80 % of a lap has been driven, so we are definitely
        #        not still sitting on the start line.
        armed = path_length >= LAP_ARM_FRACTION * TRACK_LENGTH

        # The robot stops when the TRAVELLED PATH LENGTH reaches LAP_DISTANCE,
        # which puts it physically back on the start line at the origin.
        #
        # It deliberately does NOT stop the instant the position ESTIMATE comes
        # within START_RADIUS of the origin.  That would make the reported error
        # self-fulfilling -- it could never print a number larger than the
        # trigger radius, no matter how badly the odometry had drifted.  Path
        # length comes straight from the encoders and is accurate to well under
        # 1 % (it is immune to heading drift, which is where odometry error
        # actually accumulates), so it is a fair, independent proxy for "the
        # robot is physically back on the start line".  The START_RADIUS check
        # is still evaluated and reported below, as the pass/fail statement the
        # brief asks for.
        if armed and path_length >= LAP_DISTANCE:
            finished = True
            stop_reason = "returned to the start line"
        elif sim_time > MAX_SIM_TIME:
            finished = True
            stop_reason = ("timed out after %.0f s without completing a lap"
                           % MAX_SIM_TIME)

        # ===================================================================
        #  4. DRIVE (or stop)
        # ===================================================================
        if finished:
            v_left = 0.0
            v_right = 0.0
        left_motor.setVelocity(v_left)                # command the wheels
        right_motor.setVelocity(v_right)

        # ===================================================================
        #  5. TELEMETRY
        # ===================================================================
        if step_count % PRINT_EVERY == 0:
            # xw/yw/omegaz are the estimate; |p| is its distance from the
            # origin; s is the distance travelled; gs are the raw ground-sensor
            # readings (handy for re-checking the GS_BLACK/GS_WHITE levels).
            print("t=%6.2fs  xw=%+7.4f  yw=%+7.4f  omegaz=%+6.3f  "
                  "|p|=%5.3f  s=%5.3f  gs=[%4.0f %4.0f %4.0f]  err=%+5.2f"
                  % (sim_time, xw, yw, omegaz, dist_to_start, path_length,
                     raw[0], raw[1], raw[2], err), flush=True)

        if finished:
            break

    # =======================================================================
    #  FINAL REPORT
    # =======================================================================
    # Belt and braces: make sure the wheels really are stopped.
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    error = np.sqrt(xw ** 2 + yw ** 2)

    print("", flush=True)
    print("=" * 68, flush=True)
    print("  RUN FINISHED -- %s" % stop_reason, flush=True)
    print("-" * 68, flush=True)
    print("  final pose estimate : xw = %+.4f m   yw = %+.4f m" % (xw, yw),
          flush=True)
    print("                        omegaz = %+.4f rad (%+.1f deg)"
          % (omegaz, np.degrees(omegaz)), flush=True)
    print("  path length driven  : %.4f m  (start line is %.4f m away"
          " round the loop)" % (path_length, LAP_DISTANCE), flush=True)
    print("  back at start line  : %s  (estimate within %.0f cm of origin)"
          % ("YES" if error < START_RADIUS else "NO", START_RADIUS * 100.0),
          flush=True)
    print("-" * 68, flush=True)
    print("  FINAL ODOMETRY ERROR: %.3f m" % error, flush=True)
    print("     (the robot is physically stopped on the start line at the", flush=True)
    print("      origin, so this is the localisation error itself: a perfect", flush=True)
    print("      odometer would print 0.000 m)", flush=True)
    print("=" * 68, flush=True)

    # Idle so the robot stays put and the console output remains on screen.
    while robot.step(timestep) != -1:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)


if __name__ == "__main__":
    main()
