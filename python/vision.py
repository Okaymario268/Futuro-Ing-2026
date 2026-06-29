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
import time
import threading

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Camera / geometry calibration
# ---------------------------------------------------------------------------
FRAME_W, FRAME_H = 640, 480
CAMERA_INDEX = 0

# Pinhole focal length in px. CALIBRATE ONCE: stand a pillar (real height 100 mm)
# at a known distance D mm, read its pixel height h, then FOCAL_PX = h * D / 100.
#     distance_mm = real_height_mm * FOCAL_PX / pixel_height
FOCAL_PX     = 700.0
PILLAR_H_MM  = 100.0      # WRO traffic-sign pillar height (50x50x100 mm)
MARKER_H_MM  = 100.0      # magenta parking marker height (200x20x100 mm)

# ---------------------------------------------------------------------------
# HSV colour thresholds (OpenCV hue 0-179).  CALIBRATE ON THE REAL MAT.
# Rulebook RGB: red(238,39,55) green(68,214,44) magenta(255,0,255).
# ---------------------------------------------------------------------------
RED1_LO,    RED1_HI    = (0, 120, 70),   (10, 255, 255)
RED2_LO,    RED2_HI    = (170, 120, 70), (179, 255, 255)
GREEN_LO,   GREEN_HI   = (40, 70, 50),   (85, 255, 255)
MAGENTA_LO, MAGENTA_HI = (140, 80, 80),  (165, 255, 255)
MIN_BLOB_AREA = 300       # px^2 - ignore specks

# ---------------------------------------------------------------------------
# Black-wall detection (idea from Okaymario268/Futuro-Ing-2026 - RETUNE).
# Walls are black (low S/V). We measure black coverage (0..1) in three ROIs:
# a centre FRONT band (corner/wall ahead?) and LEFT/RIGHT strips (which side is
# open -> the side to turn into).
# ---------------------------------------------------------------------------
BLACK_LO = (0, 0, 0)
BLACK_HI = (180, 120, 95)
FRONT_T, FRONT_B = int(0.55 * FRAME_H), int(0.85 * FRAME_H)
FRONT_L, FRONT_R = int(0.25 * FRAME_W), int(0.75 * FRAME_W)
SIDE_T,  SIDE_B  = int(0.55 * FRAME_H), int(0.95 * FRAME_H)
LEFT_L,  LEFT_R  = 0,                    int(0.18 * FRAME_W)
RIGHT_L, RIGHT_R = int(0.82 * FRAME_W),  FRAME_W

# White floor (track surface). Corner-direction FALLBACK used when the side wall
# strips are ambiguous: the track continues toward the side showing more floor.
WHITE_LO = (0, 0, 140)
WHITE_HI = (180, 70, 255)
FLOOR_T, FLOOR_B = int(0.45 * FRAME_H), int(0.78 * FRAME_H)   # left/right halves split at centre

# ---------------------------------------------------------------------------
# Driving policy
# ---------------------------------------------------------------------------
CRUISE_PCT     = 70       # speed when the path is clear
SLOW_DIST_MM   = 500      # start slowing for a pillar closer than this
HALT_DIST_MM   = 150      # full stop for a pillar closer than this
MAX_BIAS_DEG   = 18       # strongest single pillar-pass bias
MAX_TOTAL_BIAS = 28       # clamp on (wall-centering + pillar) bias, deg

FRONT_CORNER  = 0.45      # front-black above this -> a wall/corner is ahead
FRONT_HALT    = 0.85      # front-black above this -> wall too close -> halt
SIDE_DIFF     = 0.18      # L/R wall coverage gap needed to choose a turn direction
FLOOR_DIFF    = 0.12      # L/R floor gap to pick a turn when walls are ambiguous
WALL_GAIN     = 26.0      # deg of centering bias per unit (left-right) gap
CORNER_SPEED  = 35        # % drive speed while committing to a corner
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
    speed_pct: int    # 0..100  -> Bridge set_speed
    steer_bias: int   # deg, +right/-left -> Bridge nudge
    note: str         # human-readable reason (logged)
    turn: int = 0     # 0 / +90 / -90 -> Bridge turn() at a corner (latched)


# ===========================================================================
# Detection
# ===========================================================================
def _mask(hsv, lo, hi):
    m = cv2.inRange(hsv, np.array(lo), np.array(hi))
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
    black = cv2.inRange(hsv, np.array(BLACK_LO), np.array(BLACK_HI))
    white = cv2.inRange(hsv, np.array(WHITE_LO), np.array(WHITE_HI))

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


def annotate(frame, red, green, marker, walls, cmd):
    """Draw ROIs, detections and the command for the standalone/debug view."""
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
# Pipeline (background thread; start/stop = the camera toggle)
# ===========================================================================
class VisionPipeline:
    def __init__(self, on_command=None, camera_index=CAMERA_INDEX,
                 pillar_enabled=True, wall_enabled=True):
        self.on_command = on_command
        self.camera_index = camera_index
        self.pillar_enabled = pillar_enabled   # red/green pillar passing
        self.wall_enabled = wall_enabled       # wall centering + corner trigger
        self.last = None
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
        if self.on_command:                # leave the car un-biased / free to cruise
            try:
                self.on_command(VisionCommand(100, 0, "camera off", 0))
            except Exception:
                pass

    def process(self, frame):
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red, green = detect_pillars(hsv)
        walls = detect_walls(hsv)
        cmd = decide(walls, red, green, self.pillar_enabled, self.wall_enabled)
        cmd.turn = self._latch_corner(cmd.turn)   # debounce + one-shot
        self.last = cmd
        return cmd, red, green, walls

    def _run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
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

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
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
