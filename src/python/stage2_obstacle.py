"""stage2_obstacle.py — WRO Future Engineers · STAGE 2 · OBSTACLE CHALLENGE entry point.

Everything Stage 1 does (3 laps, orange/blue corners, direction latch, 12-corner count)
PLUS the two Stage-2-only behaviours:
  * red/green pillar passing  — produced in vision.py `decide_obstacle` (red on the
    RIGHT, green on the LEFT) and applied through the same `nudge` bias channel.
  * parallel parking finish   — the reverse-arc maneuver below, fired after corner #12.

Select this file as the App Lab program for an Obstacle-Challenge round.

    Camera (vision, stage="obstacle") --> RaceController (fsm) --> bridge_io --> MCU

>>> IMPORTANT (from the verified brief) <<<
This robot has only a camera + gyro — no side distance sensor. Dead-reckoned parking
is the documented failure mode (drift can clip a magenta marker -> ZERO parking points).
The sequence below is a tunable STARTING FRAMEWORK; strongly consider adding one
side-facing ToF/ultrasonic sensor to localise the slot opening and the rear wall.
All angles/speeds/durations are starting points to tune on the real mat.
"""
import threading
import time

from arduino.app_utils import App

from vision import VisionPipeline
from fsm import RaceController
import bridge_io as io

# ---- Parallel-park (reverse-arc) tuning — Stage 2 only --------------------------------
PARK_SPEED   = 45    # % drive speed during the maneuver (slow + controlled)
ENTRY_ANGLE  = 75    # deg: swing the tail toward the curb/slot
STRAIGHTEN   = 70    # deg: counter-steer back toward the lane while still reversing
TUCK         = 5     # deg: small forward nudge to kill residual angle (<=2 cm parallel tol)
SEG_TIMEOUT  = 4.0   # s: per-segment safety timeout waiting for the heading target


def _wait_target(timeout=SEG_TIMEOUT):
    """Block until the MCU reports the heading is within tolerance of its target."""
    t0 = time.time()
    while not io.at_target() and (time.time() - t0) < timeout:
        time.sleep(0.05)


def parallel_park(direction):
    """Reverse-arc parallel park. `direction` is the latched lap turn sign (+90 CW / -90 CCW);
    `s` mirrors the maneuver for the slot side. VERIFY the side on your mat and flip if needed."""
    s = 1 if direction >= 0 else -1
    print(f"[park] start (direction={direction}, s={s})", flush=True)

    io.set_speed(PARK_SPEED)
    io.set_drive_dir(-1)                 # reverse
    io.turn(-ENTRY_ANGLE * s)            # 1) swing tail into the slot
    _wait_target()
    io.turn(+STRAIGHTEN * s)             # 2) counter-steer straight, still reversing
    _wait_target()
    io.set_drive_dir(+1)                 # 3) small forward tuck to remove residual angle
    io.turn(+TUCK * s)
    _wait_target()
    io.set_speed(0)
    io.set_drive_dir(+1)
    print("[park] done", flush=True)


controller = RaceController(stage="obstacle", on_park=parallel_park)
camera = VisionPipeline(stage="obstacle", on_command=controller.on_vision)

START_DELAY_S = 2.0


def _begin():
    time.sleep(START_DELAY_S)
    controller.start()


print("[obstacle] Stage 2 — Obstacle Challenge: starting camera + run", flush=True)
camera.start()
threading.Thread(target=_begin, daemon=True).start()

App.run()
