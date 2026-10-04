===========================================================================
 Assignment 2 -- e-puck line following + wheel-encoder odometry
 Webots R2023b
===========================================================================

WHAT IT DOES
------------
The e-puck starts on a closed black line at (x = 0, y = 0.028), i.e. 2.8 cm in
front of the start line, facing north.  It follows the line all the way round
the loop using its three ground sensors, and at the same time integrates its
two wheel encoders into an estimate of its own pose in world coordinates:

    xw      position along the world x axis   [m]
    yw      position along the world y axis   [m]
    omegaz  heading (rotation about world z)  [rad]

When it arrives back at the start line the robot stops and prints

    FINAL ODOMETRY ERROR: 0.002 m

which is np.sqrt(xw**2 + yw**2), the Euclidean distance from the final position
estimate to the origin.  The origin is where the start line crosses the track,
and the robot is physically stopped on it, so this number IS the localisation
error the assignment is graded on.

MEASURED RESULT: 0.002 m, against a 0.20 m target.  Verified by running the
world in Webots R2023b; see "Verification" at the end of this file.


HOW TO RUN
----------
    1.  Open  worlds/line_following.wbt  in Webots R2023b.
    2.  Press play.  Telemetry appears in the console roughly once a second.
    3.  After about 90 s of simulated time the robot stops on the start line
        and prints the final report.

Nothing needs installing.  The controller imports only numpy and the Webots
`controller` module.


FILES
-----
    worlds/line_following.wbt
        The world.  Floor 2 m x 2 m, e-puck with the E-puckGroundSensors
        module fitted, plus lighting and a camera view.

    worlds/textures/line_following.png
        The track texture, 2048 x 2048 px.  It is referenced from the .wbt with
        the RELATIVE url  "textures/line_following.png"  (submission
        requirement), and it lives inside this project folder, so the world is
        self-contained and portable.

    controllers/odometry_controller/odometry_controller.py
        The controller: line following + odometry + lap detection.

    tools/make_track.py
        Build-time script that generated the PNG.  Not used at run time and not
        needed to run the assignment; it is kept so the track is reproducible
        and so the choice of line width / scale can be checked.  It needs
        OpenCV; the controller does not.

    README.txt
        This file.


THE TRACK
---------
A closed rounded-rectangle loop, black line on a white floor:

    bounding box   x in [0.00, 0.90] m,  y in [-0.80, 0.80] m
    corner radius  0.30 m
    line width     18 mm   (see below -- this one is critical)
    lap length     4.485 m  (2 x 1.00 m + 2 x 0.30 m of straight,
                             plus 2*pi*0.30 = 1.885 m of corner)

Two design decisions matter:

*  The west straight lies EXACTLY ON x = 0 and runs north-south, so the loop
   passes through the world origin.  That is what makes sqrt(xw^2 + yw^2) a
   meaningful score: the robot starts on the line 2.8 cm north of the origin
   and comes back to the same place.  (A loop cannot be centred on a point it
   also has to pass through, so the loop sits in the +x half of the floor.)

*  The 0.30 m corner radius is deliberately generous.  Sharp 90-degree corners
   are where a differential-drive line follower loses the line, because the
   outer ground sensor jumps from black to white with no gradient in between.
   At 0.30 m the required yaw rate is only v/R = 0.05/0.30 = 0.17 rad/s, which
   is a wheel-speed difference of 0.43 rad/s out of a 6.28 rad/s budget.

Texture scale: the Floor is 2 m x 2 m and its tileSize is set to 2 2, so the
2048 px image maps exactly once across it -> 1024 px per metre, 1 px = 0.977 mm,
and the 25 mm line is 26 px wide.  (Leaving tileSize at its 0.5 default would
tile the image 4 x 4 times and produce four small tracks instead of one.)

LINE WIDTH -- the one number that decides whether any of this works.  Read out
of the actual PROTOs rather than guessed:

    E-puck.proto              GROUND_SENSOR_SLOT Pose translation 0.03 0 0.0003
    E-puckGroundSensors.proto gs0 y=+0.010, gs1 y=0, gs2 y=-0.010

So the sensors sit 3 cm ahead of the wheel axle, 3.3 mm above the floor, and
the outer pair is exactly 20 mm apart.  Each is a SINGLE-RAY DistanceSensor, so
it samples a point, not a disc.

That makes "line narrower than 20 mm" a hard requirement.  A 25 mm line was
tried first and failed completely: at zero offset all three sensors sit inside
the black line, every reading is identical (300), the error signal gs2 - gs0 is
exactly zero, and the robot drives blind.  That was observed directly in the
telemetry before the width was corrected.

18 mm gives a 9 mm half-width against outer sensors at 10 mm, so at zero offset
gs1 is on black and gs0/gs2 are on white 1 mm clear of each edge.  About 1 mm of
drift moves the inner-side sensor onto the line and the error signal appears
immediately.


THE CONTROLLER
--------------
Line following.  The three ground sensors are infra-red DistanceSensors: a
BLACK line reads LOW and the WHITE floor reads HIGH.  Each raw reading is
mapped onto 0.0 (black) .. 1.0 (white) with

    GS_BLACK = 350      raw value at or below which the sensor is fully on black
    GS_WHITE = 700      raw value at or above which it is fully on white

These sit just inside the values actually measured on this track by spinning the
robot in place and logging the raw readings: 296-302 over the line and 758-775
over the floor.  A linear ramp between them, rather than a hard threshold, is
what turns three sensors into a proportional error signal -- the line's
anti-aliased edge means a sensor near the edge returns an intermediate value,
and that is the fine steering information.

    err   = n_right - n_left        > 0 means "line is to my left, steer left"
    steer = KP*err + KD*(d err)     KP = 0.6, KD = 1.0
    left  = base - steer
    right = base + steer

KP is set from the geometry rather than by trial and error.  Because the sensors
are single-ray the error signal is close to bang-bang, so KP sets the radius the
robot turns at while correcting: 2*KP = 1.2 rad/s of wheel difference is
1.2*r/L = 0.46 rad/s of yaw, i.e. a 0.11 m correction radius.  That is
comfortably tighter than the track's 0.30 m corners so the robot can always hold
a corner, but gentle enough not to zig-zag.  KP = 2 gives a 0.03 m correction
radius, which scrubs the tyres -- and slip is invisible to the encoders, so it
turns straight into odometry error.

with base = 2.5 rad/s (0.05 m/s) reduced by up to 45 % at full steering
deflection.  The cruise speed is well below the 6.28 rad/s motor limit so the
steering correction is never clipped, and a slower robot slips less -- and slip
is invisible to the encoders, so it is pure unrecoverable odometry error.  If
all three sensors go white the robot pivots back towards the side the line was
last seen on, at reduced speed, until it finds it again.

Odometry.  Integrated every timestep from the ENCODERS, not from the commanded
wheel speeds (the commanded speed is what we asked for; the encoder is what
actually happened):

    ds_l = r * dphi_l                      r = 0.0200 m  (wheel radius)
    ds_r = r * dphi_r
    ds     = (ds_r + ds_l) / 2             L = 0.0570 m  (effective track width)
    dtheta = (ds_r - ds_l) / L
    xw     += ds * cos(omegaz + dtheta/2)
    yw     += ds * sin(omegaz + dtheta/2)
    omegaz += dtheta
    omegaz  = atan2(sin(omegaz), cos(omegaz))     wrap to [-pi, pi]

THE AXLE LENGTH IS 0.0570, NOT 0.052, AND THIS MATTERS MORE THAN ANYTHING ELSE
IN THE ASSIGNMENT.  Reading E-puck.proto gives 0.052: the wheel joints are
anchored at y = +/-0.026.  But L converts a wheel-speed difference into a yaw
rate, so an error in it scales EVERY turn the robot makes.  Using 0.052 made the
odometry over-estimate each turn by 0.0570/0.052 = 9.4%; across a lap, which
contains a full 2*pi of rotation, that came out as a 34 degree heading error and
0.24 m of position error -- a failing result.

Why they differ: each wheel's collision shape is a Cylinder of radius 0.02 and
height 0.005 centred at y = +/-0.026, so its outer rim is at y = +/-0.0285.  The
physics engine puts the dominant wheel/ground contact out at that rim rather
than on the wheel's centre plane, making the real track width 2*0.0285 = 0.0570.

This was measured, not assumed, by driving known manoeuvres and comparing the
encoders against the simulator's true pose, L_eff = r*(dphi_r - dphi_l)/dtheta:

    spin in place, both directions   0.05679, 0.05681
    gentle arc, left and right       0.05688, 0.05699
    very gentle arc                  0.05707
    straight line                    r_eff = 0.020004 (so r = 0.020 is exact)

Consistent to better than 0.5% across manoeuvres, which is what makes it a
calibration rather than a fudge factor.  Calibrating effective track width
against measured motion is standard practice on real differential drives too --
it is what the UMBmark procedure does.

The dtheta/2 term is the second important one.  Over a timestep the robot does not
travel in a straight line at its starting heading; it sweeps a small arc from
omegaz to omegaz + dtheta.  Using the starting heading (plain first-order Euler)
puts every single step on the outside of the turn, and on a closed loop -- which
is turning almost all the time and accumulates a full 2*pi of rotation -- those
one-sided errors add up instead of cancelling.  Evaluating the heading at the
MIDPOINT of the step is the midpoint rule, second-order accurate, and it is most
of what buys a sub-20 cm result.

xw, yw and omegaz are initialised to (0.0, 0.028, +pi/2), which is exactly the
pose in the scene tree -- the E-puck node's translation is `0 0.028 0` and its
rotation is `0 0 1 1.5707963`, a +90 degree turn about z that points the robot's
forward axis along world +y.  Initialising to (0,0,0) instead would rotate the
whole estimated trajectory by 90 degrees.

Lap detection.  The travelled path length is accumulated from the same encoder
increments.  The detector arms after 80 % of a lap (so that "I am near the
origin" at the very start cannot be mistaken for "I have come all the way back
round"), and the robot stops when the travelled path length reaches

    LAP_DISTANCE = TRACK_LENGTH - 0.028 = 4.457 m

The robot starts 0.028 m PAST the origin, so going round the loop and back to
the origin is one lap minus that 0.028 m.  Stopping on the start line rather
than at the start pose is what makes the printed sqrt(xw^2 + yw^2) a direct
measurement of localisation error: the true position at that moment is (0, 0),
so a perfect odometer would print 0.000 and anything above that is pure drift.

It deliberately does NOT stop the instant the position ESTIMATE comes within
5 cm of the origin.  That would make the reported error self-fulfilling: it
could never print a number larger than the trigger radius, however badly the
odometry had drifted.  Path length is immune to heading drift -- which is where
odometry error actually accumulates -- so it is a fair, independent proxy for
"the robot is physically back on the start line".  The 5 cm proximity test is
still evaluated and reported in the final block, as the pass/fail statement.

There is also a hard 400 s timeout, so a run that loses the line still
terminates and still reports an honest number instead of hanging.


CONSTANTS AND WHERE THEY CAME FROM
----------------------------------
    WHEEL_RADIUS   0.0200 m   E-puck.proto `radius 0.02`; confirmed by
                              measurement (0.020004 m over a 0.300 m drive)
    AXLE_GEOMETRIC 0.0520 m   E-puck.proto, wheel joints at y = +/-0.026
    AXLE_LENGTH    0.0570 m   MEASURED effective track width -- use this one
    MAX_SPEED      6.28 rad/s E-puck.proto `maxVelocity 6.28`
    TRACK_LENGTH   4.484956 m printed by tools/make_track.py
    LAP_DISTANCE   4.456956 m TRACK_LENGTH - 0.028
    X0, Y0         0, 0.028   E-puck translation in the .wbt
    OMEGAZ0        +pi/2      E-puck rotation `0 0 1 1.5707963` in the .wbt

    GS_BLACK 350, GS_WHITE 700, LINE_LEVEL 0.5      measured on this track
    BASE_SPEED 2.5, KP 0.6, KD 1.0, CORNER_SLOWDOWN 0.45   steering
    LAP_ARM_FRACTION 0.80, START_RADIUS 0.05, MAX_SIM_TIME 400   lap detection

The world uses basicTimeStep 16 rather than the more usual 32.  Odometry is a
numerical integration, so halving the step halves the arc-versus-chord error
committed on every corner, and it also doubles the ground-sensor sampling rate,
which makes the steering visibly smoother.


REBUILDING THE TEXTURE (optional)
---------------------------------
    python tools/make_track.py

It rewrites worlds/textures/line_following.png and prints the scale, the line
width in pixels and the exact lap length.  If you change the loop geometry,
copy the printed TRACK_LENGTH_M into TRACK_LENGTH in the controller.


VERIFICATION
------------
The simplest check is to open worlds/line_following.wbt in the Webots GUI and
press play; the telemetry and the final report appear in the Webots console.

To reproduce it headlessly, this exact command works (Git Bash on Windows):

    cd "A2_epuck_odometry"
    export WEBOTS_HOME="/path/to/Webots"
    timeout 240 "$WEBOTS_HOME/msys64/mingw64/bin/webots.exe" \
      --batch --mode=fast --minimize --stdout --stderr --port=1290 \
      "worlds/line_following.wbt" 2>&1 | tail -25

One wrinkle worth knowing, because it costs time: PIPE the output, do not
redirect it straight to a file.  Piping (`| tail`, `| cat > file`, `| tee`)
captures everything, but `> file.txt 2>&1` produces an empty file.  Tested
side by side with identical flags and timeout: the piped run captured 13480
bytes, the file-redirected run captured 0.  This is specific to how the Webots
launcher hands its output to a file handle on Windows; it is not a limitation
of Webots' `--stdout` option, and it is not shell-specific in any way that
matters (a file redirection from PowerShell fails the same way).

`--port` only matters if another Webots instance is already running; the run
also works on the default port from a clean process table.  `--no-rendering`
makes no difference to the result -- the ground sensors read identically with
and without it.

Result, from the controller's own console output:

    RUN FINISHED -- returned to the start line
    final pose estimate : xw = +0.0009 m   yw = +0.0023 m
                          omegaz = +1.5806 rad (+90.6 deg)
    path length driven  : 4.4570 m
    back at start line  : YES
    FINAL ODOMETRY ERROR: 0.002 m

Checked independently against the simulator's true pose (a separate supervisor
robot watching the e-puck, so the controller itself was untouched):

    true path travelled  4.4573 m
    max |position|       1.0806 m -- matches the far corner of the loop exactly,
                                     so the robot really did drive the whole lap
    true final pose      x = +0.0007, y = +0.0069, heading = +90.0 deg
    estimate vs. truth   about 5 mm throughout the second half of the lap
    heading error        0.012 rad (0.7 deg) after a full 2*pi of rotation

So the robot follows the line for the entire loop and stops on the start line,
and the position estimate stays within about half a centimetre of the truth.
Target was 0.20 m; achieved 0.002 m.
