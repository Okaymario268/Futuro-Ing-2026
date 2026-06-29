# =============================================================================
# Servo + camera perception test (WRO Future Engineers 2026)
# Run with:  python3 test_servo.py
#
# The camera looks at the floor and decides what the steering servo should do:
#   - Sees an ORANGE line on the track  -> steer LEFT_ANGLE   (CCW corner cue)
#   - Sees a BLUE line on the track     -> steer RIGHT_ANGLE  (CW corner cue)
#   - Sees a BLACK wall close in front  -> reverse + look for a new approach
#   - Otherwise                         -> hold centre
#
# Line colours come from the WRO 2026 rules section 13.9
#   orange line:  CMYK (0, 60, 100, 0)
#   blue line:    CMYK (100, 80, 0, 0)
# Press 'q' in the preview window to quit.
# =============================================================================

import time
import cv2
import numpy as np
from gpiozero import AngularServo, Device
from gpiozero.pins.lgpio import LGPIOFactory

# Pi 5 needs the lgpio backend (RPi.GPIO does not work on the RP1 chip).
Device.pin_factory = LGPIOFactory()

# -----------------------------------------------------------------------------
# Servo config - keep these in sync with Main.py
# -----------------------------------------------------------------------------
SERVO_PIN     = 23     # BCM GPIO pin wired to the servo signal line
# Mechanical reality of this car: 0 deg = wheels pointing fully right,
# -30 deg = wheels pointing fully left. "Straight" is offset because of the
# linkage geometry, so it is measured by hand rather than derived.
LEFT_LIMIT    = -30    # Servo angle that fully steers the wheels left
RIGHT_LIMIT   =   0    # Servo angle that fully steers the wheels right
SERVO_CENTER  = -15   # Empirical "wheels straight" angle for this car
                       # (just right of the geometric midpoint at -15 deg)
LEFT_ANGLE    = LEFT_LIMIT     # Command for "turn left"
RIGHT_ANGLE   = RIGHT_LIMIT    # Command for "turn right"
SERVO_MAX     = max(abs(LEFT_LIMIT), abs(RIGHT_LIMIT))

# Pulse-width range. 0.5-2.5 ms is the servo's full ~180 deg sweep, which
# made our 60 deg logical range hyper-sensitive (1 logical deg = ~3 physical
# deg). Tightening to 1.0-2.0 ms maps the same 60 deg over half the physical
# travel, so small commanded changes (like -9.0 vs -9.1) produce small wheel
# movements. Narrow further if it is still too jumpy.
MIN_PULSE_MS  = 1.0
MAX_PULSE_MS  = 2.0

# -----------------------------------------------------------------------------
# Camera config
# -----------------------------------------------------------------------------
CAMERA_INDEX  = 0
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480
SHOW_PREVIEW  = True   # set False for headless runs (no GUI)

# Only look at pixels that correspond to floor within ~10 cm of the car.
# With a typical Pi-cam mounted ~10 cm above the floor and tilted slightly
# down, the 10 cm slice ends up being roughly the bottom 17% of the frame.
# Tune by enabling SHOW_PREVIEW and watching the green debug rectangle.
FLOOR_ROI_TOP    = int(FRAME_HEIGHT * 0.83)   # top edge of the close-range band
FLOOR_ROI_BOTTOM = FRAME_HEIGHT               # bottom of the frame
# The wall ROI sits just above the floor band - same horizontal centre slice.
WALL_ROI_TOP     = int(FRAME_HEIGHT * 0.60)
WALL_ROI_BOTTOM  = int(FRAME_HEIGHT * 0.83)
WALL_ROI_LEFT    = FRAME_WIDTH  // 4
WALL_ROI_RIGHT   = 3 * FRAME_WIDTH // 4

# Side wall ROIs - thin vertical strips on the far left and far right of the
# frame. Black pixels here mean "wall on that side of the car", which lets us
# override the line cue (e.g. orange line says LEFT, but left side is blocked
# by a black wall, so we have to go RIGHT instead).
SIDE_ROI_TOP     = int(FRAME_HEIGHT * 0.40)
SIDE_ROI_BOTTOM  = int(FRAME_HEIGHT * 0.90)
LEFT_SIDE_LEFT   = 0
LEFT_SIDE_RIGHT  = int(FRAME_WIDTH * 0.18)
RIGHT_SIDE_LEFT  = int(FRAME_WIDTH * 0.82)
RIGHT_SIDE_RIGHT = FRAME_WIDTH

# HSV colour thresholds. OpenCV stores H in 0-179, S and V in 0-255.
# Orange line ~ RGB(255,102,0) -> H~12 in OpenCV scale.
ORANGE_LOW    = np.array([  5, 130, 130])
ORANGE_HIGH   = np.array([ 22, 255, 255])
# Blue line ~ RGB(0,51,255)    -> H~115 in OpenCV scale.
BLUE_LOW      = np.array([100, 130,  70])
BLUE_HIGH     = np.array([130, 255, 255])
# Black wall - very low value, any hue, low-to-moderate saturation.
# Looser bounds here so dark navy, dark grey and dimly-lit black all count.
BLACK_LOW     = np.array([  0,   0,   0])
BLACK_HIGH    = np.array([180, 120,  95])

# Decision thresholds (fraction of the relevant ROI that must match).
LINE_THRESHOLD = 0.45   # floor ROI orange/blue fraction -> line is visible
WALL_THRESHOLD = 0.90   # front ROI black fraction -> wall is close in front
                         # (live-updatable: press 'b' with a black object in
                         #  the wall ROI to recalibrate)
SIDE_THRESHOLD = 0.35   # side ROI black fraction -> wall is close on that side

# -----------------------------------------------------------------------------
# Hardware setup
# -----------------------------------------------------------------------------
servo = AngularServo(
    SERVO_PIN,
    min_angle=-SERVO_MAX,
    max_angle=SERVO_MAX,
    min_pulse_width=MIN_PULSE_MS / 1000,
    max_pulse_width=MAX_PULSE_MS / 1000,
)

camera = cv2.VideoCapture(CAMERA_INDEX)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


# -----------------------------------------------------------------------------
# Perception
# -----------------------------------------------------------------------------
def analyse(frame):
    """Return a dict of detection ratios for the current frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Floor ROI = narrow band near the bottom, ~10 cm in front of the car.
    floor = hsv[FLOOR_ROI_TOP:FLOOR_ROI_BOTTOM, :]
    # Front wall ROI = horizontal band just above the floor band, centred.
    front = hsv[WALL_ROI_TOP:WALL_ROI_BOTTOM, WALL_ROI_LEFT:WALL_ROI_RIGHT]
    # Side wall ROIs = thin vertical strips on the far left and right.
    left_side  = hsv[SIDE_ROI_TOP:SIDE_ROI_BOTTOM,
                     LEFT_SIDE_LEFT:LEFT_SIDE_RIGHT]
    right_side = hsv[SIDE_ROI_TOP:SIDE_ROI_BOTTOM,
                     RIGHT_SIDE_LEFT:RIGHT_SIDE_RIGHT]

    orange_mask     = cv2.inRange(floor,      ORANGE_LOW, ORANGE_HIGH)
    blue_mask       = cv2.inRange(floor,      BLUE_LOW,   BLUE_HIGH)
    black_mask      = cv2.inRange(front,      BLACK_LOW,  BLACK_HIGH)
    left_black_mask  = cv2.inRange(left_side,  BLACK_LOW,  BLACK_HIGH)
    right_black_mask = cv2.inRange(right_side, BLACK_LOW,  BLACK_HIGH)

    return {
        "orange":      orange_mask.mean()     / 255.0,
        "blue":        blue_mask.mean()       / 255.0,
        "black":       black_mask.mean()      / 255.0,
        "left_black":  left_black_mask.mean()  / 255.0,
        "right_black": right_black_mask.mean() / 255.0,
    }


def draw_rois(frame):
    """Overlay the floor, front wall and side wall ROIs for tuning."""
    # Floor band (green).
    cv2.rectangle(frame, (0, FLOOR_ROI_TOP),
                  (FRAME_WIDTH - 1, FLOOR_ROI_BOTTOM - 1), (0, 255, 0), 2)
    # Front wall (red).
    cv2.rectangle(frame, (WALL_ROI_LEFT, WALL_ROI_TOP),
                  (WALL_ROI_RIGHT - 1, WALL_ROI_BOTTOM - 1), (0, 0, 255), 2)
    # Left side wall (yellow).
    cv2.rectangle(frame, (LEFT_SIDE_LEFT, SIDE_ROI_TOP),
                  (LEFT_SIDE_RIGHT - 1, SIDE_ROI_BOTTOM - 1), (0, 255, 255), 2)
    # Right side wall (yellow).
    cv2.rectangle(frame, (RIGHT_SIDE_LEFT, SIDE_ROI_TOP),
                  (RIGHT_SIDE_RIGHT - 1, SIDE_ROI_BOTTOM - 1), (0, 255, 255), 2)


def decide(ratios):
    """Translate detection ratios into a (label, angle) command.

    Priority:
      1. Black wall straight ahead -> reverse.
      2. Black wall on one side    -> turn AWAY from it (overrides line cue).
      3. Floor line colour         -> orange = left, blue = right.
      4. Nothing detected          -> hold centre.
    """
    front_wall = ratios["black"]       > WALL_THRESHOLD
    left_wall  = ratios["left_black"]  > SIDE_THRESHOLD
    right_wall = ratios["right_black"] > SIDE_THRESHOLD

    if front_wall:
        return "REVERSE", SERVO_CENTER

    # Side walls override the line cue: if the left side is blocked we must
    # turn right even if the front camera sees an orange "go left" line.
    if left_wall and right_wall:
        return "REVERSE", SERVO_CENTER
    if left_wall:
        return "RIGHT", RIGHT_ANGLE
    if right_wall:
        return "LEFT",  LEFT_ANGLE

    if ratios["blue"] > LINE_THRESHOLD and ratios["blue"] > ratios["orange"]:
        return "RIGHT", RIGHT_ANGLE
    if ratios["orange"] > LINE_THRESHOLD:
        return "LEFT",  LEFT_ANGLE
    return "CENTER", SERVO_CENTER


# -----------------------------------------------------------------------------
# Interactive servo calibration
# -----------------------------------------------------------------------------
# Mode toggles.
# CALIBRATE_ON_START  : run the interactive servo wizard at start-up.
# RUN_DETECTION       : after (or instead of) calibration, run the camera-based
#                       black/orange/blue perception loop. Disable to use this
#                       script purely as a servo calibration tool.
CALIBRATE_ON_START = False
RUN_DETECTION      = True

def calibrate_servo():
    """Walk the user through capturing CENTER, RIGHT and LEFT servo angles.

    Controls (focus the 'servo calibration' window):
      a / d        : nudge servo by -/+ STEP_SMALL deg
      A / D (caps) : nudge servo by -/+ STEP_BIG   deg
      s            : auto-sweep on/off (slow back-and-forth)
      SPACE        : lock in the current angle for the current phase
      q            : abort and return whatever has been captured so far
    """
    STEP_SMALL = 0.01
    
    STEP_BIG   = 5.0
    SWEEP_RATE = 8.0          # deg/sec when auto-sweep is on
    phases = ["CENTER", "RIGHT", "LEFT"]
    captured = {}
    phase_idx = 0
    angle     = 0.0
    sweeping  = False
    direction = +1
    last_tick = time.monotonic()
    win = "servo calibration"

    cv2.namedWindow(win)
    servo.angle = angle

    print("\n=== Servo calibration ===")
    print(f"Order: {' -> '.join(phases)}")
    print("Keys: a/d nudge  A/D big-nudge  s sweep  SPACE capture  q abort\n")

    while phase_idx < len(phases):
        # Auto-sweep ticks the angle on a timer.
        now = time.monotonic()
        if sweeping:
            dt = now - last_tick
            angle += direction * SWEEP_RATE * dt
            if angle >= SERVO_MAX:
                angle, direction = SERVO_MAX, -1
            elif angle <= -SERVO_MAX:
                angle, direction = -SERVO_MAX, +1
            servo.angle = angle
        last_tick = now

        # HUD frame.
        hud = np.zeros((220, 640, 3), dtype=np.uint8)
        cv2.putText(hud, f"Phase: {phases[phase_idx]} ({phase_idx + 1}/3)",
                    (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(hud, f"Angle: {angle:+6.2f} deg   sweep={'ON' if sweeping else 'off'}",
                    (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(hud, "a/d nudge   A/D big   s sweep   SPACE capture   q quit",
                    (15, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.putText(hud, "captured: " + ", ".join(
                        f"{k}={v:+.1f}" for k, v in captured.items()),
                    (15, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 255), 1)
        cv2.imshow(win, hud)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('a'):
            angle = max(-SERVO_MAX, angle - STEP_SMALL); servo.angle = angle
        elif key == ord('d'):
            angle = min( SERVO_MAX, angle + STEP_SMALL); servo.angle = angle
        elif key == ord('A'):
            angle = max(-SERVO_MAX, angle - STEP_BIG);   servo.angle = angle
        elif key == ord('D'):
            angle = min( SERVO_MAX, angle + STEP_BIG);   servo.angle = angle
        elif key == ord('s'):
            sweeping = not sweeping
        elif key == ord(' '):
            captured[phases[phase_idx]] = round(angle, 2)
            print(f"[CAPTURED] {phases[phase_idx]} = {angle:+.2f} deg")
            phase_idx += 1
            sweeping = False     # pause the sweep between captures

    cv2.destroyWindow(win)
    print("\n--- Calibration result ---")
    for name in phases:
        if name in captured:
            print(f"  {name:<6} = {captured[name]:+.2f} deg")
        else:
            print(f"  {name:<6} = (not captured)")
    print("Paste these into the constants at the top of the file to make"
          " them permanent.\n")
    return captured


# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def main():
    global WALL_THRESHOLD, SERVO_CENTER, LEFT_ANGLE, RIGHT_ANGLE

    # Step 1: optionally run the interactive calibration wizard and overwrite
    # the angle constants with whatever the user captures.
    if CALIBRATE_ON_START:
        captured = calibrate_servo()
        if "CENTER" in captured: SERVO_CENTER = captured["CENTER"]
        if "RIGHT"  in captured: RIGHT_ANGLE  = captured["RIGHT"]
        if "LEFT"   in captured: LEFT_ANGLE   = captured["LEFT"]

    # Calibration-only mode: park the wheels at the captured centre and exit.
    if not RUN_DETECTION:
        servo.angle = SERVO_CENTER
        time.sleep(0.3)
        servo.detach()
        cv2.destroyAllWindows()
        print("Detection disabled (RUN_DETECTION = False). Exiting.")
        return

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera at index {CAMERA_INDEX}")

    print(f"Servo on GPIO {SERVO_PIN}, angles L/R = {LEFT_ANGLE}/{RIGHT_ANGLE}, "
          f"center = {SERVO_CENTER}")
    print("Keys (preview window): 'q' quit  |  'b' calibrate black wall  |"
          "  'r' re-arm one-shot colours\n")

    last_print = 0.0
    PRINT_INTERVAL = 0.2   # seconds between debug lines (~5 Hz)
    last_angle = None      # cache so we only repulse the servo on real changes
    streak_label = "CENTER"
    streak_count = 0
    REQUIRED_STREAK = 3    # frames of agreement before committing a turn
    centred_at = time.monotonic()
    DETACH_AFTER = 0.4     # seconds to wait in CENTER before releasing the servo
    detached = False
    # One-shot latch: each non-CENTER label fires the servo exactly once
    # per session. Subsequent detections of the same colour are ignored
    # (servo holds its last commanded angle). Press 'r' to clear and re-arm.
    fired_labels = set()
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                continue

            ratios = analyse(frame)
            raw_label, raw_angle = decide(ratios)

            # Debounce: a non-CENTER decision must persist for several frames
            # before we commit. Anything else snaps the wheels back to centre,
            # so brief mis-detections never flick the servo around.
            if raw_label == streak_label:
                streak_count += 1
            else:
                streak_label = raw_label
                streak_count = 1

            if raw_label == "CENTER" or streak_count < REQUIRED_STREAK:
                label, angle = "CENTER", SERVO_CENTER
                locked = False
            elif raw_label in fired_labels:
                # Same colour was already fired earlier in this session.
                # Skip the servo write so the wheels stay where they are.
                label  = raw_label + "*"     # '*' = locked-out in the log
                angle  = last_angle if last_angle is not None else SERVO_CENTER
                locked = True
            else:
                # First time we are committing to this colour - fire and save.
                label, angle = raw_label, raw_angle
                fired_labels.add(raw_label)
                locked = False
                print(f"[FIRE] {raw_label} -> servo {angle:+.1f} deg "
                      f"(fired so far: {sorted(fired_labels)})")

            # Commit to the servo only on real changes, so we don't keep
            # repulsing the same target (which is what makes hobby servos hum).
            if not locked and angle != last_angle:
                servo.angle = angle
                last_angle = angle
                detached = False
                centred_at = time.monotonic() if label == "CENTER" else 0

            # If we have been holding CENTER long enough, fully release the
            # servo so it stops driving the output and goes quiet.
            if (label == "CENTER" and not detached
                    and centred_at and time.monotonic() - centred_at > DETACH_AFTER):
                servo.detach()
                detached = True

            # Throttled live debug feed - one line every PRINT_INTERVAL seconds.
            now = time.monotonic()
            if now - last_print >= PRINT_INTERVAL:
                print(f"{label:8s}  "
                      f"O={ratios['orange']:.2f} "
                      f"B={ratios['blue']:.2f} "
                      f"front={ratios['black']:.2f} "
                      f"L={ratios['left_black']:.2f} "
                      f"R={ratios['right_black']:.2f}  "
                      f"-> servo {angle:+.1f} deg  "
                      f"fired={sorted(fired_labels) or '-'}")
                last_print = now

            if SHOW_PREVIEW:
                draw_rois(frame)
                cv2.imshow("test_servo", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("b"):
                    sampled = ratios["black"]
                    WALL_THRESHOLD = max(0.02, sampled * 0.8)
                    print(f"[CALIBRATE] sampled black={sampled:.3f}  "
                          f"-> WALL_THRESHOLD={WALL_THRESHOLD:.3f}")
                if key == ord("r"):
                    fired_labels.clear()
                    print("[RE-ARM] one-shot colours cleared")
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        servo.angle = SERVO_CENTER
        time.sleep(0.3)
        servo.detach()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
