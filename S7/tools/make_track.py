"""
make_track.py -- generator for  worlds/textures/line_following.png
=================================================================

This script is *build-time only*.  The artefact that ships with the assignment
is the PNG it writes; the script is kept in the project so that the track is
reproducible and so that a reviewer can see exactly how the geometry and the
line width were chosen (nothing here is guessed).

Run it from anywhere:

    python tools/make_track.py

Requires numpy + OpenCV (both used only here, NEVER by the robot controller --
the controller depends on numpy and the Webots `controller` module alone).


THE TRACK
---------
A closed rounded-rectangle loop, black line on white background:

                 y = +0.80  ,-----------------.
                            |                 |
                            |                 |
    start line -> y =  0.00 |                 |      (robot starts at
                            |                 |       x=0, y=+0.028,
                            |                 |       heading NORTH, +y)
                 y = -0.80  `-----------------'
                          x = 0.0           x = 0.9

*  The **west straight sits exactly on x = 0 and runs north-south**, so the
   loop passes through the world origin (0, 0).  That matters because the
   assignment starts the robot at (0, 0.028) -- 2.8 cm "slightly in front of
   the start line" -- and scores the run with the Euclidean distance of the
   final pose estimate from (0, 0).  Because the line at the origin runs along
   the y axis, the robot at (0, 0.028) is sitting *on* the line, not beside it.

*  Corner radius is 0.30 m.  A differential-drive line follower loses the line
   at sharp corners because the outer ground sensor jumps straight from black
   to white with no gradient in between.  A 0.30 m radius means the e-puck only
   needs omega = v / R = 0.05 / 0.30 = 0.17 rad/s of yaw rate to stay on the
   arc, i.e. a wheel-speed difference of just omega * L / r = 0.43 rad/s out of
   a 6.28 rad/s budget.  There is no corner anywhere on this track that the
   robot has to fight.

*  The loop is deliberately NOT centred on the floor.  It cannot be: the origin
   has to lie *on* the line (see above), and no convex loop centred on a point
   can pass through that point.  So the loop is pushed to the +x half of the
   floor and the origin sits on its western edge.


LINE WIDTH -- the number that decides whether line following works at all
-------------------------------------------------------------------------
This is set by the sensor geometry, which was read out of the actual PROTOs
rather than guessed:

    E-puck.proto            DEF GROUND_SENSOR_SLOT Pose { translation 0.03 0 0.0003 }
    E-puckGroundSensors.proto   gs0 at y = +0.010, gs1 at y = 0, gs2 at y = -0.010

So the three sensors sit 3 cm ahead of the wheel axle and the outer pair is
exactly **20 mm apart** (+/-10 mm from the centreline), 3.3 mm above the floor.
Each is a single-ray DistanceSensor, so it samples essentially a POINT on the
texture, not a disc.

That makes the line width a hard constraint, not a preference:

    line wider than 20 mm -> at zero offset ALL THREE sensors are inside the
                             black line, every reading is identical, the error
                             signal (gs2 - gs0) is exactly zero, and the robot
                             drives blind until it falls off the line entirely.

(That failure was observed directly: with a 25 mm line every sensor sat at 300
-- the "black" reading -- for the whole run and the robot never steered.)

The line therefore has to be NARROWER than the outer sensor pair, so that at
zero offset the outer sensors straddle the line and sit on white while gs1
stays on black.  18 mm gives:

    half width          9 mm  (vs outer sensors at 10 mm)
    at zero offset      gs1 black, gs0/gs2 white 1 mm clear of each edge
    drift of ~1 mm      the outer sensor on the inside of the drift crosses
                        onto black -> error signal appears immediately
    drift of ~9 mm      gs1 leaves the line -> "line lost" recovery arms

Measured raw readings on this floor: ~300 over the line, ~765 over the floor.


PIXELS PER METRE
----------------
The Floor node in the world is 2.0 m x 2.0 m and its `tileSize` is set to
2.0 x 2.0 so the texture is mapped **exactly once** across the whole floor
(the Floor default tileSize is 0.5, which would tile the image 4x4 times and
produce four little tracks instead of one).  Therefore:

    PX_PER_M = IMG_PX / FLOOR_SIZE_M = 2048 / 2.0 = 1024 px per metre
             -> 1 pixel = 0.977 mm on the floor
    LINE_WIDTH_PX = 0.018 m * 1024 px/m = 18.4 -> 18 px (= 17.6 mm on the floor)

2048 is a power of two, which is what the renderer wants for a texture.
"""

import os
import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Floor / image scaling
# ---------------------------------------------------------------------------
FLOOR_SIZE_M = 2.0                       # must equal the Floor node's `size`
IMG_PX = 2048                            # texture resolution (power of two)
PX_PER_M = IMG_PX / FLOOR_SIZE_M         # 1024 px per metre

LINE_WIDTH_M = 0.018                     # 18 mm -- see the discussion above.
                                         # MUST stay below the 20 mm spacing of
                                         # the outer ground sensors.
LINE_WIDTH_PX = int(round(LINE_WIDTH_M * PX_PER_M))   # 18 px

# ---------------------------------------------------------------------------
# Loop geometry, in WORLD metres (ENU: +x east, +y north)
# ---------------------------------------------------------------------------
X_W = 0.00        # west straight -- lies on x = 0 so the loop hits the origin
X_E = 0.90        # east straight
Y_S = -0.80       # south straight
Y_N = 0.80        # north straight
R = 0.30          # corner radius (generous on purpose)

# The straight portions that survive after the corners are rounded off.
STRAIGHT_Y = (Y_N - R) - (Y_S + R)       # 1.00 m, on each of x = X_W and X_E
STRAIGHT_X = (X_E - R) - (X_W + R)       # 0.30 m, on each of y = Y_S and Y_N

# Total centreline length of the closed loop.  The four quarter-circles add up
# to exactly one full circle of radius R, hence the 2*pi*R term.
TRACK_LENGTH_M = 2.0 * STRAIGHT_Y + 2.0 * STRAIGHT_X + 2.0 * np.pi * R


# ---------------------------------------------------------------------------
# World -> pixel mapping
# ---------------------------------------------------------------------------
def world_to_px(x, y):
    """Convert world metres to (column, row) in the texture image.

    Webots maps texture coordinate u along +x and v along +y, and an image's
    first row is the top of the picture (v = 1).  So the image reads like a map
    seen from above with north (+y) at the top and east (+x) to the right:

        col = (x + FLOOR/2) * PX_PER_M      x = -1.0 -> col 0,  x = +1.0 -> 2048
        row = (FLOOR/2 - y) * PX_PER_M      y = +1.0 -> row 0,  y = -1.0 -> 2048

    (The track happens to be symmetric about y = 0, and mirroring it in x would
    merely reverse the direction of travel, so even if a future Webots release
    flipped u or v the track would still be a valid closed loop through the
    origin.  That was a deliberate design choice.)
    """
    col = (np.asarray(x) + FLOOR_SIZE_M / 2.0) * PX_PER_M
    row = (FLOOR_SIZE_M / 2.0 - np.asarray(y)) * PX_PER_M
    return col, row


def arc(cx, cy, a_start, a_end, n=240):
    """Sample a circular arc of radius R centred on (cx, cy).

    Angles are standard maths angles (0 = +x, pi/2 = +y).  Going from a_start
    DOWN to a_end traces the arc clockwise, which is the direction this track
    is driven.
    """
    a = np.linspace(a_start, a_end, n)
    return cx + R * np.cos(a), cy + R * np.sin(a)


def line(x0, y0, x1, y1, n=200):
    """Sample a straight segment from (x0, y0) to (x1, y1)."""
    return np.linspace(x0, x1, n), np.linspace(y0, y1, n)


def build_centreline():
    """Return the closed centreline of the loop as world-coordinate arrays.

    Traced CLOCKWISE starting at the origin, which is the direction the robot
    drives: north up the west straight, east across the top, south down the
    east straight, west across the bottom, back to the origin.
    """
    segs = []
    # West straight, heading north, from the SW corner tangent up to the NW one.
    segs.append(line(X_W, Y_S + R, X_W, Y_N - R))
    # NW corner: 180 deg -> 90 deg, i.e. from "pointing north" to "pointing east".
    segs.append(arc(X_W + R, Y_N - R, np.pi, np.pi / 2))
    # North straight, heading east.
    segs.append(line(X_W + R, Y_N, X_E - R, Y_N))
    # NE corner: 90 deg -> 0 deg  (east -> south).
    segs.append(arc(X_E - R, Y_N - R, np.pi / 2, 0.0))
    # East straight, heading south.
    segs.append(line(X_E, Y_N - R, X_E, Y_S + R))
    # SE corner: 0 deg -> -90 deg  (south -> west).
    segs.append(arc(X_E - R, Y_S + R, 0.0, -np.pi / 2))
    # South straight, heading west.
    segs.append(line(X_E - R, Y_S, X_W + R, Y_S))
    # SW corner: -90 deg -> -180 deg  (west -> north), closing the loop.
    segs.append(arc(X_W + R, Y_S + R, -np.pi / 2, -np.pi))

    xs = np.concatenate([s[0] for s in segs])
    ys = np.concatenate([s[1] for s in segs])
    return xs, ys


def main():
    # Blank white floor.  3-channel so the PNG is a plain RGB image, which every
    # Webots texture loader is happy with.
    img = np.full((IMG_PX, IMG_PX, 3), 255, dtype=np.uint8)

    # ---- the racing line ---------------------------------------------------
    xs, ys = build_centreline()
    col, row = world_to_px(xs, ys)
    pts = np.stack([col, row], axis=1).round().astype(np.int32)

    # One closed anti-aliased polyline.  The points are ~1 mm apart, so the
    # polygonal approximation of the arcs is far finer than the texture's own
    # 0.977 mm pixel pitch -- the result is visually a perfect smooth curve.
    # Anti-aliasing is not cosmetic here: it gives the infra-red ground sensors
    # a *gradual* black-to-white edge instead of a hard step, which is what lets
    # the proportional controller produce a smooth steering signal.
    cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 0),
                  thickness=LINE_WIDTH_PX, lineType=cv2.LINE_AA)

    # ---- start-line markers ------------------------------------------------
    # Two short bars flanking the track at y = 0 so the start line is visible.
    # They stop 45 mm short of the line centre on both sides.  The outer ground
    # sensors only reach ~11 mm off centre (plus a few mm of IR spot), so the
    # robot can never see these marks -- they are decoration, not track.
    for x0, x1 in ((0.045, 0.100), (-0.100, -0.045)):
        c0, r0 = world_to_px(x0, +LINE_WIDTH_M / 2.0)
        c1, r1 = world_to_px(x1, -LINE_WIDTH_M / 2.0)
        cv2.rectangle(img,
                      (int(round(min(c0, c1))), int(round(min(r0, r1)))),
                      (int(round(max(c0, c1))), int(round(max(r0, r1)))),
                      color=(0, 0, 0), thickness=-1)

    # ---- decoration inside the loop ---------------------------------------
    # Light grey, and 0.2 m or more from any part of the track, so it can never
    # be mistaken for the line by a ground sensor.
    grey = (170, 170, 170)
    c, r = world_to_px(0.30, -0.03)
    cv2.putText(img, "START", (int(c), int(r)), cv2.FONT_HERSHEY_SIMPLEX,
                2.6, grey, 6, cv2.LINE_AA)
    # Direction-of-travel arrow: the robot leaves the start line heading north.
    c0, r0 = world_to_px(0.20, -0.10)
    c1, r1 = world_to_px(0.20, 0.10)
    cv2.arrowedLine(img, (int(c0), int(r0)), (int(c1), int(r1)),
                    grey, 8, cv2.LINE_AA, tipLength=0.3)

    # ---- write it out ------------------------------------------------------
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "worlds", "textures", "line_following.png")
    out = os.path.normpath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, img)

    print("wrote %s" % out)
    print("  image            : %d x %d px" % (IMG_PX, IMG_PX))
    print("  floor            : %.2f x %.2f m" % (FLOOR_SIZE_M, FLOOR_SIZE_M))
    print("  scale            : %.1f px/m  (1 px = %.3f mm)"
          % (PX_PER_M, 1000.0 / PX_PER_M))
    print("  line width       : %d px = %.1f mm"
          % (LINE_WIDTH_PX, 1000.0 * LINE_WIDTH_PX / PX_PER_M))
    print("  loop bbox        : x [%.2f, %.2f]  y [%.2f, %.2f] m"
          % (X_W, X_E, Y_S, Y_N))
    print("  corner radius    : %.2f m" % R)
    print("  TRACK_LENGTH_M   : %.6f m   <-- copy into the controller"
          % TRACK_LENGTH_M)


if __name__ == "__main__":
    main()
