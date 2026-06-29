# =============================================================================
# Autonomous Car - Central Control Script (WRO Future Engineers 2026)
# Board: Raspberry Pi 5
# Vision: OpenCV (cv2) - orange/blue floor lines + black wall detection
# Drive train: 1 DC motor driven through an L298N H-bridge (one channel)
# Steering: 1 servo motor on a hardware PWM-capable pin
# GPIO library: gpiozero on the lgpio backend (required on Pi 5)
# =============================================================================

import time
import cv2
import numpy as np
from gpiozero import Motor, PWMOutputDevice, AngularServo, Device
from gpiozero.pins.lgpio import LGPIOFactory

# RP1 on the Pi 5 needs lgpio; RPi.GPIO does not work.
Device.pin_factory = LGPIOFactory()

# -----------------------------------------------------------------------------
# Pin configuration - every component pin lives here, BCM numbering.
# -----------------------------------------------------------------------------
PINS = {
    "L298N_IN1":   19,
    "L298N_IN2":   18,
    "L298N_ENA":   12,
    "SERVO":       23,
    "CAMERA_INDEX": 0,
}

# -----------------------------------------------------------------------------
# Feature toggles
# -----------------------------------------------------------------------------
MOTOR_ENABLED  = True
SERVO_ENABLED  = True
CAMERA_ENABLED = True

# -----------------------------------------------------------------------------
# Drive / steering parameters
# -----------------------------------------------------------------------------
DRIVE_SPEED   = 0.6     # PWM duty cycle when moving forward (0.0 - 1.0)
REVERSE_SPEED = 0.5
LEFT_LIMIT    = -30     # Servo angle for full-left
RIGHT_LIMIT   =   0     # Servo angle for full-right
SERVO_CENTER  = -12     # Empirical wheels-straight angle for this car
LEFT_ANGLE    = LEFT_LIMIT
RIGHT_ANGLE   = RIGHT_LIMIT
SERVO_MAX     = max(abs(LEFT_LIMIT), abs(RIGHT_LIMIT))
MIN_PULSE_MS  = 1.0     # Narrow pulse range -> finer steering resolution
MAX_PULSE_MS  = 2.0

# Auto-init: park servo at centre, hold for SERVO_CENTER_HOLD, then go.
SERVO_CENTER_HOLD = 2   # seconds the wheels stay straight before motor engages

# -----------------------------------------------------------------------------
# Camera + ROIs
# -----------------------------------------------------------------------------
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480

# Floor band ~10 cm in front of the car (orange/blue line cues).
FLOOR_ROI_TOP    = int(FRAME_HEIGHT * 0.83)
FLOOR_ROI_BOTTOM = FRAME_HEIGHT
# Front wall band just above the floor (black wall straight ahead).
WALL_ROI_TOP     = int(FRAME_HEIGHT * 0.60)
WALL_ROI_BOTTOM  = int(FRAME_HEIGHT * 0.83)
WALL_ROI_LEFT    = FRAME_WIDTH  // 4
WALL_ROI_RIGHT   = 3 * FRAME_WIDTH // 4
# Side wall strips ~6 cm to the left/right of the car. Tighter than before:
# narrower horizontal slice and only the bottom of the frame, so only walls
# very close to the side of the car register.
SIDE_ROI_TOP     = int(FRAME_HEIGHT * 0.65)
SIDE_ROI_BOTTOM  = int(FRAME_HEIGHT * 0.95)
LEFT_SIDE_LEFT   = 0
LEFT_SIDE_RIGHT  = int(FRAME_WIDTH * 0.12)
RIGHT_SIDE_LEFT  = int(FRAME_WIDTH * 0.88)
RIGHT_SIDE_RIGHT = FRAME_WIDTH

# -----------------------------------------------------------------------------
# HSV colour thresholds (OpenCV: H 0-179, S 0-255, V 0-255)
# -----------------------------------------------------------------------------
ORANGE_LOW  = np.array([  5, 130, 130])
ORANGE_HIGH = np.array([ 22, 255, 255])
BLUE_LOW    = np.array([100, 130,  70])
BLUE_HIGH   = np.array([130, 255, 255])
BLACK_LOW   = np.array([  0,   0,   0])
BLACK_HIGH  = np.array([180, 120,  95])

LINE_THRESHOLD = 0.45   # fraction of floor ROI that must be the line colour
WALL_THRESHOLD = 0.90   # fraction of front ROI that must be black
SIDE_THRESHOLD = 0.35   # fraction of side ROI that must be black

# -----------------------------------------------------------------------------
# Hardware setup
# -----------------------------------------------------------------------------
if MOTOR_ENABLED:
    drive_motor = Motor(forward=PINS["L298N_IN1"], backward=PINS["L298N_IN2"])
    drive_pwm   = PWMOutputDevice(PINS["L298N_ENA"])
else:
    drive_motor = None
    drive_pwm   = None

if SERVO_ENABLED:
    steering = AngularServo(
        PINS["SERVO"],
        min_angle=-SERVO_MAX,
        max_angle=SERVO_MAX,
        min_pulse_width=MIN_PULSE_MS / 1000,
        max_pulse_width=MAX_PULSE_MS / 1000,
    )
else:
    steering = None

if CAMERA_ENABLED:
    camera = cv2.VideoCapture(PINS["CAMERA_INDEX"])
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
else:
    camera = None


# -----------------------------------------------------------------------------
# Drive primitives
# -----------------------------------------------------------------------------
def set_speed(value):
    if not MOTOR_ENABLED:
        return
    drive_pwm.value = max(0.0, min(1.0, value))


def forward(speed=DRIVE_SPEED):
    if not MOTOR_ENABLED:
        return
    drive_motor.forward()
    set_speed(speed)


def backward(speed=REVERSE_SPEED):
    if not MOTOR_ENABLED:
        return
    drive_motor.backward()
    set_speed(speed)


def stop():
    if MOTOR_ENABLED:
        drive_motor.stop()
        set_speed(0)


def steer(angle):
    if not SERVO_ENABLED:
        return
    steering.angle = max(-SERVO_MAX, min(SERVO_MAX, angle))


def countdown(seconds, label):
    """Print a 1 Hz countdown to the terminal (overwrites the same line)."""
    remaining = int(seconds)
    while remaining > 0:
        print(f"{label}: {remaining:2d}s ", end="\r", flush=True)
        time.sleep(1)
        remaining -= 1
    leftover = seconds - int(seconds)
    if leftover > 0:
        time.sleep(leftover)
    print(f"{label}: GO!     ")


# -----------------------------------------------------------------------------
# Perception
# -----------------------------------------------------------------------------
def analyse(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    floor      = hsv[FLOOR_ROI_TOP:FLOOR_ROI_BOTTOM, :]
    front      = hsv[WALL_ROI_TOP:WALL_ROI_BOTTOM,
                     WALL_ROI_LEFT:WALL_ROI_RIGHT]
    left_side  = hsv[SIDE_ROI_TOP:SIDE_ROI_BOTTOM,
                     LEFT_SIDE_LEFT:LEFT_SIDE_RIGHT]
    right_side = hsv[SIDE_ROI_TOP:SIDE_ROI_BOTTOM,
                     RIGHT_SIDE_LEFT:RIGHT_SIDE_RIGHT]

    return {
        "orange":      cv2.inRange(floor,      ORANGE_LOW, ORANGE_HIGH).mean() / 255.0,
        "blue":        cv2.inRange(floor,      BLUE_LOW,   BLUE_HIGH  ).mean() / 255.0,
        "black":       cv2.inRange(front,      BLACK_LOW,  BLACK_HIGH ).mean() / 255.0,
        "left_black":  cv2.inRange(left_side,  BLACK_LOW,  BLACK_HIGH ).mean() / 255.0,
        "right_black": cv2.inRange(right_side, BLACK_LOW,  BLACK_HIGH ).mean() / 255.0,
    }


def draw_rois(frame):
    cv2.rectangle(frame, (0, FLOOR_ROI_TOP),
                  (FRAME_WIDTH - 1, FLOOR_ROI_BOTTOM - 1), (0, 255, 0), 2)
    cv2.rectangle(frame, (WALL_ROI_LEFT, WALL_ROI_TOP),
                  (WALL_ROI_RIGHT - 1, WALL_ROI_BOTTOM - 1), (0, 0, 255), 2)
    cv2.rectangle(frame, (LEFT_SIDE_LEFT, SIDE_ROI_TOP),
                  (LEFT_SIDE_RIGHT - 1, SIDE_ROI_BOTTOM - 1), (0, 255, 255), 2)
    cv2.rectangle(frame, (RIGHT_SIDE_LEFT, SIDE_ROI_TOP),
                  (RIGHT_SIDE_RIGHT - 1, SIDE_ROI_BOTTOM - 1), (0, 255, 255), 2)


def decide(ratios):
    """Translate detection ratios into a (label, steering angle) command.

    Priority:
      1. Black wall straight ahead          -> REVERSE
      2. Both sides blocked                 -> REVERSE
      3. One side blocked                   -> turn AWAY (overrides line cue)
      4. Floor line                         -> orange = LEFT, blue = RIGHT
      5. Nothing                            -> CENTER
    """
    if ratios["black"] > WALL_THRESHOLD:
        return "REVERSE", SERVO_CENTER

    left_wall  = ratios["left_black"]  > SIDE_THRESHOLD
    right_wall = ratios["right_black"] > SIDE_THRESHOLD
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
# Start-up sequence
# -----------------------------------------------------------------------------
def auto_init():
    """Park the wheels at centre, hold SERVO_CENTER_HOLD seconds, then
    engage forward drive. The perception loop takes over after this."""
    stop()
    steer(SERVO_CENTER)
    countdown(SERVO_CENTER_HOLD, label="Centering wheels")
    forward()


# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def main():
    # Camera-less bench test: just centre the servo, hold, and exit.
    if not CAMERA_ENABLED:
        try:
            auto_init()
            time.sleep(2)
        finally:
            stop()
            steer(SERVO_CENTER)
        return

    if not camera.isOpened():
        raise RuntimeError(f"Could not open camera at index {PINS['CAMERA_INDEX']}")

    print(f"Servo center={SERVO_CENTER}, L={LEFT_ANGLE}, R={RIGHT_ANGLE}")
    print("Keys (preview window): 'q' quit | 'r' re-arm one-shot colours\n")

    last_angle = None
    streak_label = "CENTER"
    streak_count = 0
    REQUIRED_STREAK = 3
    fired_labels = set()    # one-shot latch: each non-CENTER fires once
    last_print = 0.0
    PRINT_INTERVAL = 0.2

    try:
        auto_init()    # centre wheels, wait 2 s, then start the motor

        while True:
            ok, frame = camera.read()
            if not ok:
                continue

            ratios = analyse(frame)
            raw_label, raw_angle = decide(ratios)

            # Debounce: a non-CENTER decision needs N consecutive frames.
            if raw_label == streak_label:
                streak_count += 1
            else:
                streak_label = raw_label
                streak_count = 1

            if raw_label == "CENTER" or streak_count < REQUIRED_STREAK:
                label, angle = "CENTER", SERVO_CENTER
                locked = False
                # Track the first stable CENTER commit so it appears in the
                # fired set alongside the colour labels. CENTER is exempt
                # from the lockout below - we want the wheels to always be
                # allowed to return to centre.
                if (raw_label == "CENTER"
                        and streak_count >= REQUIRED_STREAK
                        and "CENTER" not in fired_labels):
                    fired_labels.add("CENTER")
                    print(f"[FIRE] CENTER -> servo {angle:+.1f} deg "
                          f"(fired so far: {sorted(fired_labels)})")
            elif raw_label in fired_labels:
                # Same colour already fired - hold last commanded angle.
                label  = raw_label + "*"
                angle  = last_angle if last_angle is not None else SERVO_CENTER
                locked = True
            else:
                label, angle = raw_label, raw_angle
                fired_labels.add(raw_label)
                locked = False
                print(f"[FIRE] {raw_label} -> servo {angle:+.1f} deg "
                      f"(fired so far: {sorted(fired_labels)})")

            # Couple the drive direction to the decision: REVERSE backs up,
            # everything else drives forward at DRIVE_SPEED.
            if not locked:
                if raw_label == "REVERSE":
                    backward()
                else:
                    forward()

            # Commit the steering angle only on real changes.
            if not locked and angle != last_angle:
                steer(angle)
                last_angle = angle

            now = time.monotonic()
            if now - last_print >= PRINT_INTERVAL:
                print(f"{label:8s} "
                      f"O={ratios['orange']:.2f} "
                      f"B={ratios['blue']:.2f} "
                      f"front={ratios['black']:.2f} "
                      f"L={ratios['left_black']:.2f} "
                      f"R={ratios['right_black']:.2f}  "
                      f"-> servo {angle:+.1f} fired={sorted(fired_labels) or '-'}")
                last_print = now

            draw_rois(frame)
            cv2.imshow("car-view", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("r"):
                fired_labels.clear()
                print("[RE-ARM] one-shot colours cleared")
    finally:
        stop()
        steer(SERVO_CENTER)
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
