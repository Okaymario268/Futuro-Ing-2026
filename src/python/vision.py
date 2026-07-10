"""
vision.py - OpenCV vision pipeline for the FuturoIng WRO robot (Arduino UNO Q).

Runs on the Linux side. Reads the Logitech USB camera and produces driving hints
for the gyro-steered car. It NEVER steers directly - it feeds the gyro loop on
the MCU through a callback (main.py wires it to the bridge):

    speed_pct  : 0..100               -> Bridge set_speed   (proximity throttle)
    steer_bias : deg, +right / -left  -> Bridge nudge        (lane centering / pass)
    turn       : 0 / +90 / -90        -> Bridge turn         (corner trigger, latched)

TWO independent, toggleable behaviors:
  * WALL / CORNER  (wall_enabled)  - detects the black walls: centers in the lane,
        slows/halts at a wall ahead, and TRIGGERS a corner turn toward the opening.
        This is the "corner" assist. Wall-detection approach is adapted (not copied)
        from github.com/Okaymario268/Futuro-Ing-2026 - RETUNE the thresholds.
  * PILLAR  (pillar_enabled)  - red->pass RIGHT, green->pass LEFT (WRO sign rule).

Disabling pillars keeps the corner/wall assist, and vice-versa.

Board-agnostic (no bridge import). Run standalone to calibrate without the board:
    python vision.py                  # prints the command stream from camera 0
    python vision.py --camera 1
    python vision.py --save out.jpg   # also writes an annotated frame ~1x/second
"""
from dataclasses import dataclass
import argparse
import glob
import time
import threading

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Camera / geometry calibration
# ---------------------------------------------------------------------------
# 320x240: quarter the pixels of 640x480 -> ~2x the frame rate / half the latency
# on the UNO Q's A53 cores, and detection at WRO distances doesn't need more.
# RESOLUTION-DEPENDENT CONSTANTS (scale together if you change FRAME_W/H):
#     FOCAL_PX, MIN_BLOB_AREA, LINE_MIN_PIX, K_PILLAR
FRAME_W, FRAME_H = 320, 240
CAMERA_INDEX = 0

# Pinhole focal length in px AT THIS RESOLUTION. CALIBRATE ONCE: stand a pillar
# (real height 100 mm) at a known distance D mm, read its pixel height h in the
# 320x240 frame, then FOCAL_PX = h * D / 100.
#     distance_mm = real_height_mm * FOCAL_PX / pixel_height
FOCAL_PX     = 350.0      # (~700 at 640x480 -> ~350 at 320x240)
PILLAR_H_MM  = 100.0      # WRO traffic-sign pillar height (50x50x100 mm)
MARKER_H_MM  = 100.0      # magenta parking marker height (200x20x100 mm)

# ---------------------------------------------------------------------------
# HSV colour thresholds (OpenCV hue 0-179).  CALIBRATE ON THE REAL MAT.
# Rulebook RGB: red(238,39,55) green(68,214,44) magenta(255,0,255).
# Stored as np arrays once (cv2.inRange takes them directly; no per-frame allocs).
# ---------------------------------------------------------------------------
def _hsv(t):
    return np.array(t, dtype=np.uint8)

RED1_LO,    RED1_HI    = _hsv((0, 120, 70)),   _hsv((10, 255, 255))
RED2_LO,    RED2_HI    = _hsv((170, 120, 70)), _hsv((179, 255, 255))
GREEN_LO,   GREEN_HI   = _hsv((40, 70, 50)),   _hsv((85, 255, 255))
MAGENTA_LO, MAGENTA_HI = _hsv((140, 80, 80)),  _hsv((165, 255, 255))
MIN_BLOB_AREA = 75        # px^2 - ignore specks (300 at 640x480 -> 75 at 320x240)

# ---------------------------------------------------------------------------
# Black-wall detection (idea from Okaymario268/Futuro-Ing-2026 - RETUNE).
# Walls are black (low S/V). We measure black coverage (0..1) in three ROIs:
# a centre FRONT band (corner/wall ahead?) and LEFT/RIGHT strips (which side is
# open -> the side to turn into).
# ---------------------------------------------------------------------------
BLACK_LO = _hsv((0, 0, 0))
BLACK_HI = _hsv((180, 120, 95))
FRONT_T, FRONT_B = int(0.35 * FRAME_H), int(0.65 * FRAME_H)   # MID band: catch a wall
FRONT_L, FRONT_R = int(0.25 * FRAME_W), int(0.75 * FRAME_W)   # AHEAD, not only when it's
                                                             # already filling the bottom
                                                             # (old 0.55-0.85 read ~0 with a
                                                             # wall dead ahead -> drove into it)
SIDE_T,  SIDE_B  = int(0.55 * FRAME_H), int(0.95 * FRAME_H)
LEFT_L,  LEFT_R  = 0,                    int(0.18 * FRAME_W)
RIGHT_L, RIGHT_R = int(0.82 * FRAME_W),  FRAME_W

# White floor (track surface). Corner-direction FALLBACK used when the side wall
# strips are ambiguous: the track continues toward the side showing more floor.
WHITE_LO = _hsv((0, 0, 140))
WHITE_HI = _hsv((180, 70, 255))
FLOOR_T, FLOOR_B = int(0.45 * FRAME_H), int(0.78 * FRAME_H)   # left/right halves split at centre

# ---------------------------------------------------------------------------
# Orange / blue CORNER LINES (WRO FE). The 20 mm floor lines are the PRIMARY
# corner trigger for both stages; the FIRST colour seen latches the run's turn
# direction. (The black-wall heuristic stays only as a centering / front-halt aid.)
#   The orange->right / blue->left mapping is a TEAM CONVENTION, not a rule -
#   confirm it on YOUR mat for a known CW/CCW layout and flip if turns go wrong.
# HSV starting ranges from F2er-WRO 2025 (open_challenge_color.py). TUNE ON MAT.
# ---------------------------------------------------------------------------
# WIDE band: SEE the line early (latch direction, show it in the UI)...
LINE_T, LINE_B = int(0.55 * FRAME_H), int(0.98 * FRAME_H)
LINE_L, LINE_R = int(0.10 * FRAME_W), int(0.90 * FRAME_W)
# ...but only FIRE the turn once the line sits LOW in the frame (= near the nose,
# whatever the camera's height/tilt): its pixel-centroid row must pass this line.
# The 90-deg arc should START at the line, not 0.4 m before it (sim-verified).
LINE_FIRE_ROW = int(0.80 * FRAME_H)
ORANGE_LINE_LO, ORANGE_LINE_HI = _hsv((5, 80, 80)),   _hsv((25, 255, 255))
BLUE_LINE_LO,   BLUE_LINE_HI   = _hsv((100, 100, 40)), _hsv((130, 255, 255))
LINE_MIN_PIX = 200        # px in the ROI to call a line "present"  (TUNE ON MAT via the feed)
ORANGE_TURN, BLUE_TURN = +90, -90    # latched turn sign per first-line colour (verify!)

# ---------------------------------------------------------------------------
# Driving policy
# ---------------------------------------------------------------------------
CRUISE_PCT     = 90       # speed when the path is clear (raised 70 -> 90 with the
                          # MCU's MOTOR_SPEED now at full 255 duty; camera-on no
                          # longer strangles the motor)
SLOW_DIST_MM   = 500      # start slowing for a pillar closer than this
HALT_DIST_MM   = 150      # full stop for a pillar closer than this
MAX_BIAS_DEG   = 18       # strongest single pillar-pass bias
MAX_TOTAL_BIAS = 28       # clamp on (wall-centering + pillar) bias, deg

FRONT_CORNER  = 0.45      # front-black above this -> a wall/corner is ahead -> steer around
FRONT_HALT    = 0.88      # front-black above this -> wall right in front -> last-resort halt
AVOID_BIAS    = 24        # deg: hard steer toward the OPEN side when a wall looms ahead
SIDE_DIFF     = 0.18      # L/R wall coverage gap needed to choose a turn direction
FLOOR_DIFF    = 0.12      # L/R floor gap to pick a turn when walls are ambiguous
WALL_GAIN     = 26.0      # deg of centering bias per unit (left-right) gap
CORNER_SPEED  = 60        # % drive speed while committing to a corner (raised 35 -> 60:
                          # 35% was inside the L298N stall region -- the "camera on =
                          # no motor power" bug; matches OPEN_CORNER_SPEED/TURN_DRIVE_PCT)
CORNER_STREAK = 3         # consecutive frames before firing a corner turn()
CLEAR_STREAK  = 4         # consecutive clear frames before re-arming the trigger

_KERNEL = np.ones((5, 5), np.uint8)


# ===========================================================================
# Data types
# ===========================================================================
@dataclass
class Blob:
    color: str
    cx: int
    cy: int
    w: int
    h: int
    area: int
    distance_mm: float


@dataclass
class WallInfo:
    front: float            # 0..1 black coverage of the centre FRONT band
    left: float             # 0..1 black coverage of the LEFT strip
    right: float            # 0..1 black coverage of the RIGHT strip
    floor_left: float = 0.0   # 0..1 white-floor coverage, LEFT half (fallback)
    floor_right: float = 0.0  # 0..1 white-floor coverage, RIGHT half (fallback)


@dataclass
class VisionCommand:
    speed_pct: int    # 0..100 -> Bridge set_speed; None = leave speed UNCHANGED
    steer_bias: int   # deg, +right/-left -> Bridge nudge
    note: str         # human-readable reason (logged)
    turn: int = 0     # 0 / +90 / -90 -> Bridge turn() at a corner (latched)


# ===========================================================================
# Detection
# ===========================================================================
def _mask(hsv, lo, hi):
    m = cv2.inRange(hsv, lo, hi)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, _KERNEL)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, _KERNEL)
    return m


def _largest_blob(mask, color, real_h_mm=PILLAR_H_MM, min_area=MIN_BLOB_AREA):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area or (best is not None and area <= best.area):
            continue
        x, y, w, h = cv2.boundingRect(c)
        dist = (real_h_mm * FOCAL_PX / h) if h > 0 else 1e9
        best = Blob(color, x + w // 2, y + h // 2, w, h, int(area), dist)
    return best


def detect_pillars(hsv):
    """(red_blob, green_blob) - either may be None."""
    red = _mask(hsv, RED1_LO, RED1_HI) | _mask(hsv, RED2_LO, RED2_HI)
    green = _mask(hsv, GREEN_LO, GREEN_HI)
    return _largest_blob(red, "red"), _largest_blob(green, "green")


def detect_marker(hsv):
    """Magenta parking marker blob, or None."""
    return _largest_blob(_mask(hsv, MAGENTA_LO, MAGENTA_HI), "magenta", MARKER_H_MM)


def detect_walls(hsv):
    """Black-wall coverage in the FRONT/side ROIs, plus white-floor coverage in
    the left/right halves (the corner-direction fallback). All values 0..1."""
    black = cv2.inRange(hsv, BLACK_LO, BLACK_HI)
    white = cv2.inRange(hsv, WHITE_LO, WHITE_HI)

    def frac(mask, y0, y1, x0, x1):
        roi = mask[y0:y1, x0:x1]
        return float(roi.mean() / 255.0) if roi.size else 0.0

    midx = FRAME_W // 2
    return WallInfo(
        front=frac(black, FRONT_T, FRONT_B, FRONT_L, FRONT_R),
        left=frac(black, SIDE_T, SIDE_B, LEFT_L, LEFT_R),
        right=frac(black, SIDE_T, SIDE_B, RIGHT_L, RIGHT_R),
        floor_left=frac(white, FLOOR_T, FLOOR_B, 0, midx),
        floor_right=frac(white, FLOOR_T, FLOOR_B, midx, FRAME_W),
    )


def detect_lines(hsv):
    """(orange_px, blue_px, orange_row, blue_row) inside the floor-line ROI — the
    corner trigger + direction cue for both stages. Counts compare to LINE_MIN_PIX
    for 'present'; rows are each colour's pixel-centroid row in FULL-FRAME coords
    (-1 = none) and gate the FIRE: low row = line near the nose.

    Red-PILLAR pixels are subtracted from the orange count: pillar red (H~0-10,
    high S) overlaps the orange band's low end, so without this a close red pillar
    inside the ROI can false-fire a corner — or worse, mis-latch turn_direction on
    the FIRST event and turn every corner of the run the wrong way."""
    roi = hsv[LINE_T:LINE_B, LINE_L:LINE_R]
    o = cv2.inRange(roi, ORANGE_LINE_LO, ORANGE_LINE_HI)
    red = cv2.inRange(roi, RED1_LO, RED1_HI) | cv2.inRange(roi, RED2_LO, RED2_HI)
    o = cv2.bitwise_and(o, cv2.bitwise_not(red))
    b = cv2.inRange(roi, BLUE_LINE_LO, BLUE_LINE_HI)

    def _centroid_row(mask):
        ys = np.nonzero(mask)[0]
        return int(ys.mean()) + LINE_T if ys.size else -1

    return (int(cv2.countNonZero(o)), int(cv2.countNonZero(b)),
            _centroid_row(o), _centroid_row(b))


# ===========================================================================
# Policy: detections -> a driving command
# ===========================================================================
def _throttle(dist_mm):
    if dist_mm <= HALT_DIST_MM:
        return 0
    if dist_mm >= SLOW_DIST_MM:
        return CRUISE_PCT
    frac = (dist_mm - HALT_DIST_MM) / (SLOW_DIST_MM - HALT_DIST_MM)
    return int(CRUISE_PCT * frac)


def decide(walls, red, green, pillar_enabled=True, wall_enabled=True):
    """Fuse wall + pillar detections into a driving command. `turn` here is the
    INSTANTANEOUS corner suggestion; VisionPipeline latches it to fire once."""
    bias, speed, turn = 0, CRUISE_PCT, 0
    notes = []

    # --- WALL / CORNER behavior (toggleable: this is the "corner" assist) --
    if wall_enabled:
        # Centre in the lane: steer away from the closer (more-covered) wall.
        bias += int(WALL_GAIN * (walls.left - walls.right))     # + = steer right
        if walls.front > FRONT_HALT:
            speed = 0
            notes.append(f"front wall {walls.front:.2f} -> HALT")
        elif walls.front > FRONT_CORNER:
            speed = min(speed, CORNER_SPEED)
            bias += _avoid_bias(walls)          # steer around the wall while deciding the corner
            if walls.left - walls.right > SIDE_DIFF:
                turn = +90
                notes.append("corner: wall LEFT, open right -> turn right")
            elif walls.right - walls.left > SIDE_DIFF:
                turn = -90
                notes.append("corner: wall RIGHT, open left -> turn left")
            # Fallback: side strips ambiguous -> turn toward the side with more
            # white floor (the track continues that way). Robust at odd corners.
            elif walls.floor_right - walls.floor_left > FLOOR_DIFF:
                turn = +90
                notes.append("corner: more floor RIGHT -> turn right")
            elif walls.floor_left - walls.floor_right > FLOOR_DIFF:
                turn = -90
                notes.append("corner: more floor LEFT -> turn left")
            else:
                notes.append(f"corner ahead {walls.front:.2f} (ambiguous) -> slow")

    # --- PILLAR behavior (toggleable) -------------------------------------
    if pillar_enabled:
        target = None
        if red and green:
            target = red if red.distance_mm < green.distance_mm else green
        else:
            target = red or green
        if target is not None:
            closeness = max(0.0, min(1.0, (SLOW_DIST_MM - target.distance_mm) / SLOW_DIST_MM))
            b = int(MAX_BIAS_DEG * max(0.4, closeness))
            if target.color == "red":
                bias += +b
                notes.append(f"red @ {target.distance_mm:.0f}mm -> keep RIGHT")
            else:
                bias += -b
                notes.append(f"green @ {target.distance_mm:.0f}mm -> keep LEFT")
            speed = min(speed, _throttle(target.distance_mm))
    elif red or green:
        notes.append("pillar seen - behavior OFF")

    bias = max(-MAX_TOTAL_BIAS, min(MAX_TOTAL_BIAS, bias))
    note = "; ".join(notes) if notes else "clear"
    return VisionCommand(int(speed), int(bias), note, int(turn))


# ===========================================================================
# Stage-specific policies (corner turn is added by the pipeline from the lines)
# ===========================================================================
# Pillar passing target-x (Stage 2): steer each colour's blob toward a target column.
RED_TARGET_X   = int(0.17 * FRAME_W)   # red  -> keep blob LEFT  -> car swings RIGHT (red-on-right)
GREEN_TARGET_X = int(0.83 * FRAME_W)   # green -> keep blob RIGHT -> car swings LEFT  (green-on-left)
K_PILLAR = 0.12                        # deg of bias per px of x-error at 320x240
                                       # (0.06 at 640x480 — px errors halve)  TUNE ON MAT


# --- STAGE 1 (open) overrides — tuned on the real mat, July 2026 --------------
# The shared defaults made the Stage-1 bot react to the black wall way too early:
# it slowed/halted mid-straight and crawled into corners at 35%, which stalls the
# L298N drivetrain and looks like a dead stop. React at ~HALF the distance (the
# front band's black coverage grows as the wall nears -> a higher threshold =
# a closer wall) and keep real motor power through the corner. The HC-SR04 on
# the MCU still hard-blocks forward under 150 mm, so reacting later is safe.
OPEN_FRONT_CORNER = 0.70   # was FRONT_CORNER 0.45 -> steer-around kicks in ~50% closer
OPEN_FRONT_HALT   = 0.96   # was FRONT_HALT 0.88 -> halt only when the band is truly filled
OPEN_CORNER_SPEED = 60     # % drive while steering around / sweeping a corner (was 35)


def _centering_bias(walls):
    return int(WALL_GAIN * (walls.left - walls.right))   # steer away from the closer wall


def _avoid_bias(walls):
    """When a wall looms straight AHEAD, steer toward the OPEN side (the side with more
    floor / less wall) so the car noses AROUND the corner instead of driving into it."""
    open_score = (walls.floor_right - walls.floor_left) + (walls.left - walls.right)
    return AVOID_BIAS if open_score >= 0 else -AVOID_BIAS


def decide_open(walls):
    """STAGE 1 steering: gyro drives straight; BLACK-WALL detection is back ON
    (user request) but reacts LATE -- at ~half the distance of the shared
    defaults (the OPEN_* constants above) -- so the bot no longer slows/halts
    mid-straight or crawls into corners. Corners come from the LINES
    (VisionPipeline); the front ultrasonic on the MCU stays the hard wall
    guard. NO pillars/magenta."""
    bias = _centering_bias(walls)
    speed, note = CRUISE_PCT, "open"
    if walls.front > OPEN_FRONT_CORNER:             # wall/corner CLOSE ahead
        speed = min(CRUISE_PCT, OPEN_CORNER_SPEED)  # keep power, just ease off...
        bias += _avoid_bias(walls)                  # ...and STEER toward the open side
        note = f"wall {walls.front:.2f} -> steer around"
        if walls.front > OPEN_FRONT_HALT:           # band truly filled: last-resort stop
            speed, note = 0, f"wall {walls.front:.2f} HALT"
    bias = max(-MAX_TOTAL_BIAS, min(MAX_TOTAL_BIAS, bias))
    return VisionCommand(int(speed), int(bias), note, 0)


def decide_obstacle(walls, red, green):
    """STAGE 2 steering: wall-centering + red/green pillar passing (target-x PD). Corners
    still come from the lines (pipeline); magenta/parking handled by the parking stage."""
    bias = _centering_bias(walls)
    speed, notes = CRUISE_PCT, []
    target = red if (red and green and red.distance_mm < green.distance_mm) else (red or green)
    if target is not None:
        tx = RED_TARGET_X if target.color == "red" else GREEN_TARGET_X
        pbias = int(-K_PILLAR * (tx - target.cx))     # red-on-right / green-on-left, self-correcting
        pbias = max(-MAX_BIAS_DEG, min(MAX_BIAS_DEG, pbias))
        bias += pbias
        speed = min(speed, _throttle(target.distance_mm))
        notes.append(f"{target.color}@{target.distance_mm:.0f}mm x{target.cx} bias{pbias:+d}")
    if walls.front > FRONT_CORNER:
        speed = min(speed, CORNER_SPEED)
        bias += _avoid_bias(walls)                  # steer around a wall ahead
        notes.append(f"wall {walls.front:.2f} -> steer around")
        if walls.front > FRONT_HALT:
            speed = 0
            notes.append("HALT")
    bias = max(-MAX_TOTAL_BIAS, min(MAX_TOTAL_BIAS, bias))
    return VisionCommand(int(speed), int(bias), "; ".join(notes) or "obstacle: clear", 0)


def annotate(frame, red, green, marker, walls, cmd):
    """Draw ROIs, detections and the command for the standalone/debug view."""
    # LINE ROI (yellow box) + highlight detected line pixels so threshold tuning
    # is visual: orange/blue tape must light up inside this box, counts >= LINE_MIN_PIX.
    roi_bgr = frame[LINE_T:LINE_B, LINE_L:LINE_R]
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    om = cv2.inRange(roi_hsv, ORANGE_LINE_LO, ORANGE_LINE_HI)
    bm = cv2.inRange(roi_hsv, BLUE_LINE_LO, BLUE_LINE_HI)
    roi_bgr[om > 0] = (0, 140, 255)
    roi_bgr[bm > 0] = (255, 80, 0)
    cv2.rectangle(frame, (LINE_L, LINE_T), (LINE_R, LINE_B), (0, 255, 255), 1)
    cv2.line(frame, (LINE_L, LINE_FIRE_ROW), (LINE_R, LINE_FIRE_ROW), (0, 255, 255), 1)
    cv2.putText(frame, f"line o={int(cv2.countNonZero(om))} b={int(cv2.countNonZero(bm))}",
                (LINE_L + 4, LINE_T + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    cv2.rectangle(frame, (FRONT_L, FRONT_T), (FRONT_R, FRONT_B), (200, 200, 0), 1)
    cv2.rectangle(frame, (LEFT_L, SIDE_T), (LEFT_R, SIDE_B), (200, 120, 0), 1)
    cv2.rectangle(frame, (RIGHT_L, SIDE_T), (RIGHT_R, SIDE_B), (200, 120, 0), 1)
    # floor fallback band + centre divider
    cv2.rectangle(frame, (0, FLOOR_T), (FRAME_W, FLOOR_B), (0, 160, 160), 1)
    cv2.line(frame, (FRAME_W // 2, FLOOR_T), (FRAME_W // 2, FLOOR_B), (0, 160, 160), 1)
    for b, col in ((red, (0, 0, 255)), (green, (0, 255, 0)), (marker, (255, 0, 255))):
        if b:
            x, y = b.cx - b.w // 2, b.cy - b.h // 2
            cv2.rectangle(frame, (x, y), (x + b.w, y + b.h), col, 2)
            cv2.putText(frame, f"{b.color} {b.distance_mm:.0f}mm", (x, max(0, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    cv2.putText(frame, f"wall F{walls.front:.2f} L{walls.left:.2f} R{walls.right:.2f}  "
                       f"floor L{walls.floor_left:.2f} R{walls.floor_right:.2f}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, f"speed={cmd.speed_pct}% bias={cmd.steer_bias:+d} turn={cmd.turn:+d}  {cmd.note}",
                (8, FRAME_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return frame


# ===========================================================================
# Camera setup (shared by the pipeline thread and the standalone mode)
# ===========================================================================
def _camera_candidates(preferred):
    """Places to look for a REAL camera, best first.

    UNO Q gotcha (found on hardware): /dev/video0 and /dev/video1 are the
    Qualcomm Venus video CODEC (decoder/encoder), NOT cameras — opening index 0
    always fails. A USB webcam enumerates at /dev/video2+ and, more reliably,
    under /dev/v4l/by-id/usb-* which can only match USB devices. So: by-id
    paths first, then the preferred index, then a 0..9 index sweep."""
    cands = sorted(glob.glob("/dev/v4l/by-id/usb-*video-index0"))
    if isinstance(preferred, int):
        cands.append(preferred)
        cands.extend(i for i in range(10) if i != preferred)
    else:
        cands.append(preferred)
        cands.extend(range(10))
    return cands


def _configure(cap):
    """LOW LATENCY: MJPG (less USB bandwidth than raw YUYV, so the requested fps
    is actually delivered) and a 1-frame buffer — otherwise V4L2 queues stale
    frames and the car acts on where a wall/pillar WAS several frames ago."""
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def _open_camera(index):
    """Open the first candidate that opens AND delivers a frame (the Venus codec
    nodes fail one of the two). Returns a closed VideoCapture when none works."""
    for cand in _camera_candidates(index):
        try:
            cap = cv2.VideoCapture(cand, cv2.CAP_V4L2) if isinstance(cand, str) \
                  else cv2.VideoCapture(cand)
        except Exception:  # noqa: BLE001 - backend quirks on odd nodes
            continue
        if not cap.isOpened():
            cap.release()
            continue
        _configure(cap)
        ok, _ = cap.read()
        if ok:
            print(f"[vision] camera found at {cand!r}", flush=True)
            return cap
        cap.release()
    print("[vision] NO usable camera. Note: on the UNO Q /dev/video0-1 are the "
          "Qualcomm video CODEC, not cameras — a USB webcam appears as "
          "/dev/video2+ (or /dev/v4l/by-id/usb-*). If none is listed, the "
          "webcam is not enumerated: check the powered USB hub / cable.",
          flush=True)
    return cv2.VideoCapture()   # closed sentinel; callers check isOpened()


# ===========================================================================
# Pipeline (background thread; start/stop = the camera toggle)
# ===========================================================================
class VisionPipeline:
    def __init__(self, on_command=None, camera_index=CAMERA_INDEX,
                 pillar_enabled=True, wall_enabled=True, stage=None):
        self.on_command = on_command
        self.camera_index = camera_index
        self.pillar_enabled = pillar_enabled   # red/green pillar passing (legacy path)
        self.wall_enabled = wall_enabled       # wall centering + corner trigger (legacy path)
        # stage = "open" | "obstacle" -> autonomous race path (line-triggered corners).
        # stage = None -> legacy combined behaviour used by the manual Web UI (main.py).
        self.stage = stage
        self.turn_direction = 0                # latched +90/-90 from the FIRST corner line
        self.last = None
        self.last_lines = (0, 0)               # latest (orange_px, blue_px) for the UI
        self.last_frame = None                 # most recent frame (for the Web UI preview)
        self._overlay = None                   # (red, green, walls, cmd) for the annotated feed
        self._thread = None
        self._stop = threading.Event()
        # corner-trigger latch state (fire a turn once per corner)
        self._corner_streak = 0
        self._clear_streak = 0
        self._last_dir = 0
        self._armed = True

    def is_running(self):
        return bool(self._thread and self._thread.is_alive())

    def set_pillar_enabled(self, enabled):
        """Toggle ONLY the red/green pillar behavior. Returns the new state."""
        self.pillar_enabled = bool(enabled)
        return self.pillar_enabled

    def set_wall_enabled(self, enabled):
        """Toggle ONLY the wall / corner assist. Returns the new state."""
        self.wall_enabled = bool(enabled)
        if not self.wall_enabled:      # reset the latch so it can't fire stale
            self._corner_streak = self._clear_streak = 0
            self._armed = True
        return self.wall_enabled

    def _latch_corner(self, suggestion):
        """Debounce + one-shot: only fire a turn after CORNER_STREAK consistent
        frames, then disarm until the front is clear again for CLEAR_STREAK."""
        fire = 0
        if suggestion != 0:
            self._clear_streak = 0
            if suggestion == self._last_dir:
                self._corner_streak += 1
            else:
                self._last_dir = suggestion
                self._corner_streak = 1
            if self._armed and self._corner_streak >= CORNER_STREAK:
                fire = suggestion
                self._armed = False
                self._corner_streak = 0
        else:
            self._corner_streak = 0
            self._last_dir = 0
            self._clear_streak += 1
            if self._clear_streak >= CLEAR_STREAK:
                self._armed = True
        return fire

    def start(self):
        if self.is_running():
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        self._thread = None
        # Clear the steering bias but leave the SPEED untouched (speed_pct=None):
        # if the pipeline had throttled down because a wall was close, jumping to
        # 100% on camera-off would floor the car straight at that wall.
        if self.on_command:
            try:
                self.on_command(VisionCommand(None, 0, "camera off", 0))
            except Exception:
                pass

    def process(self, frame):
        if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
            frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        self.last_frame = frame                # for the Web UI live preview
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        walls = detect_walls(hsv)

        # --- legacy / Web-UI path (no stage): keep the old combined behaviour ---
        if self.stage not in ("open", "obstacle"):
            red, green = detect_pillars(hsv)
            cmd = decide(walls, red, green, self.pillar_enabled, self.wall_enabled)
            cmd.turn = self._latch_corner(cmd.turn)   # debounce + one-shot
            self.last = cmd
            self._overlay = (red, green, walls, cmd)
            return cmd, red, green, walls

        # --- stage path: the orange/blue line is the PRIMARY corner trigger ---
        if self.stage == "obstacle":
            red, green = detect_pillars(hsv)
            cmd = decide_obstacle(walls, red, green)
        else:                                    # "open"
            red = green = None
            cmd = decide_open(walls)

        o, b, o_row, b_row = detect_lines(hsv)
        self.last_lines = (o, b)                          # live counts for the UI/tuning
        line_present = max(o, b) >= LINE_MIN_PIX
        if line_present and self.turn_direction == 0:     # latch direction on the FIRST line
            self.turn_direction = ORANGE_TURN if o >= b else BLUE_TURN
        # ENTRY lines only: every corner zone has BOTH colours (entry + exit). Counting
        # any line double-counts corners -- the exit line is crossed right as the sweep
        # ends, past cooldown, at_target true again. The mat's semantics: the colour that
        # latched the direction is the ENTRY colour for the whole run; only it may fire.
        dominant = ORANGE_TURN if o >= b else BLUE_TURN
        row = o_row if dominant == ORANGE_TURN else b_row
        near = row >= LINE_FIRE_ROW                       # low in frame = under the nose
        suggestion = self.turn_direction if (line_present and near
                                             and dominant == self.turn_direction) else 0
        if max(o, b) > LINE_MIN_PIX // 2:                 # surface it while tuning
            cmd.note = f"{cmd.note} | line o={o} b={b} row={row}"
        cmd.turn = self._latch_corner(suggestion)         # debounce -> one fire per physical line
        self.last = cmd
        self._overlay = (red, green, walls, cmd)
        return cmd, red, green, walls

    def snapshot_jpeg(self, quality=70, overlay=True):
        """Encode the latest frame to JPEG bytes for the Web UI live view. Returns
        None if no frame yet. With overlay=True, draws the detection boxes/ROIs so
        the feed doubles as a vision-calibration view."""
        f = self.last_frame
        if f is None:
            return None
        if overlay and self._overlay is not None:
            f = f.copy()                        # don't scribble on the shared frame
            red, green, walls, cmd = self._overlay
            annotate(f, red, green, None, walls, cmd)
        ok, buf = cv2.imencode(".jpg", f, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return buf.tobytes() if ok else None

    def _run(self):
        cap = _open_camera(self.camera_index)
        if not cap.isOpened():
            print(f"[vision] ERROR: cannot open camera {self.camera_index}", flush=True)
            return
        print(f"[vision] camera {self.camera_index} open; running", flush=True)
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                cmd, _, _, _ = self.process(frame)
                if self.on_command:
                    try:
                        self.on_command(cmd)
                    except Exception as e:  # noqa: BLE001
                        print(f"[vision] callback error: {e!r}", flush=True)
        finally:
            cap.release()
            print("[vision] camera released", flush=True)


# ===========================================================================
# Standalone entry point (no board, no bridge - for calibration / debugging)
# ===========================================================================
def _main():
    ap = argparse.ArgumentParser(description="FuturoIng WRO vision pipeline (standalone)")
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX, help="camera index")
    ap.add_argument("--save", metavar="PATH", default=None,
                    help="write an annotated frame to PATH ~1x/second")
    args = ap.parse_args()

    cap = _open_camera(args.camera)
    if not cap.isOpened():
        print(f"[vision] cannot open camera {args.camera}", flush=True)
        return

    print("[vision] standalone mode (instantaneous, no latch). Ctrl-C to stop.", flush=True)
    last_save = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            red, green = detect_pillars(hsv)
            marker = detect_marker(hsv)
            walls = detect_walls(hsv)
            cmd = decide(walls, red, green)
            print(f"[vision] wall F{walls.front:.2f} L{walls.left:.2f} R{walls.right:.2f}  "
                  f"floor L{walls.floor_left:.2f} R{walls.floor_right:.2f} | "
                  f"speed={cmd.speed_pct:3d}% bias={cmd.steer_bias:+3d} turn={cmd.turn:+d}  {cmd.note}",
                  flush=True)
            if args.save and (time.time() - last_save) > 1.0:
                cv2.imwrite(args.save, annotate(frame, red, green, marker, walls, cmd))
                last_save = time.time()
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()


if __name__ == "__main__":
    _main()
