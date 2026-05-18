# =============================================================================
# Autonomous Car - Central Control Script
# Board: Raspberry Pi 5
# Vision: OpenCV (cv2) via the Pi Camera / USB camera
# Drive train: 1 DC motor driven through an L298N H-bridge (one channel)
# Steering: 1 servo motor (e.g. SG90 / MG996R) on a hardware PWM-capable pin
# GPIO library: gpiozero (works natively on the Pi 5's RP1 chip)
# =============================================================================

import time
import cv2
from gpiozero import Motor, PWMOutputDevice, AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device

# Force gpiozero to use the lgpio backend. On the Raspberry Pi 5 the legacy
# RPi.GPIO library no longer works because the GPIO is controlled by the
# RP1 southbridge - lgpio is the supported replacement.
Device.pin_factory = LGPIOFactory()

# -----------------------------------------------------------------------------
# Pin configuration
# -----------------------------------------------------------------------------
# Every component pin is centralised in this dictionary so the wiring can be
# changed in one place. All numbers are BCM GPIO numbers (not physical pins).
PINS = {
    # ---- L298N : single DC drive motor (uses only channel A of the L298N) ----
    "L298N_IN1": 19,   # Direction pin 1 -> drives the motor forward when HIGH
    "L298N_IN2": 18,   # Direction pin 2 -> drives the motor backward when HIGH
    "L298N_ENA": 12,   # PWM speed pin (hardware PWM channel 0)

    # ---- Steering servo ----
    "SERVO": 23,       # PWM signal pin for the steering servo

    # ---- Camera ----
    # The Pi Camera connects through the CSI ribbon (no GPIO), but cv2 still
    # needs the device index. /dev/video0 is index 0 on a fresh Pi OS install.
    "CAMERA_INDEX": 0,
}

# -----------------------------------------------------------------------------
# Tunable parameters
# -----------------------------------------------------------------------------
DRIVE_SPEED   = 1     # Default forward speed (0.0 - 1.0 duty cycle)
SERVO_CENTER  = 0       # Servo angle (deg) that points the wheels straight
SERVO_MAX     = 35      # Maximum steering deflection in either direction
FRAME_WIDTH   = 640
FRAME_HEIGHT  = 480

# Auto-init sequence: idle delay before launch + duration of the initial push.
INIT_DELAY    = 2     # Seconds to wait after power-up before doing anything
INIT_FORWARD  = 9     # Seconds to drive forward at start-up

# Feature toggles - flip these back to True when wiring is complete.
SERVO_ENABLED  = False   # Skip servo init/commands while bench-testing the motor
CAMERA_ENABLED = False   # Skip cv2 capture/preview while bench-testing the motor

# -----------------------------------------------------------------------------
# Hardware objects
# -----------------------------------------------------------------------------
# gpiozero's Motor class wraps the two direction pins of one H-bridge channel.
# The ENA pin is handled separately as a PWMOutputDevice so we can vary the
# duty cycle (= speed) independently of the direction.
drive_motor = Motor(forward=PINS["L298N_IN1"], backward=PINS["L298N_IN2"])
drive_pwm   = PWMOutputDevice(PINS["L298N_ENA"])

# AngularServo gives a clean degrees-based interface instead of raw PWM.
# min_pulse_width / max_pulse_width match a typical hobby servo (SG90).
if SERVO_ENABLED:
    steering = AngularServo(
        PINS["SERVO"],
        min_angle=-SERVO_MAX,
        max_angle=SERVO_MAX,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
    )
else:
    steering = None

# OpenCV capture handle - opened once at boot and reused for every frame.
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
    """Set the PWM duty cycle on the L298N enable pin (0.0 - 1.0)."""
    drive_pwm.value = max(0.0, min(1.0, value))


def forward(speed=DRIVE_SPEED):
    """Drive the motor forward at the given speed."""
    drive_motor.forward()
    set_speed(speed)


def backward(speed=DRIVE_SPEED):
    """Drive the motor in reverse."""
    drive_motor.backward()
    set_speed(speed)


def stop():
    """Cut power to the motor and centre the steering."""
    drive_motor.stop()
    set_speed(0)
    if SERVO_ENABLED:
        steering.angle = SERVO_CENTER


def steer(angle):
    """Aim the front wheels. Negative = left, positive = right."""
    if not SERVO_ENABLED:
        return
    steering.angle = max(-SERVO_MAX, min(SERVO_MAX, angle))


def countdown(seconds, label="Starting in"):
    """Print a 1 Hz countdown to the terminal. Uses '\\r' so the line is
    overwritten in place instead of scrolling."""
    remaining = int(seconds)
    while remaining > 0:
        print(f"{label}: {remaining:2d}s ", end="\r", flush=True)
        time.sleep(1)
        remaining -= 1
    leftover = seconds - int(seconds)   # handle fractional values
    if leftover > 0:
        time.sleep(leftover)
    print(f"{label}: GO!     ")


def auto_init():
    """Start-up routine: idle for INIT_DELAY seconds with the motor off, then
    drive straight forward for INIT_FORWARD seconds. Runs once before the
    main perception loop takes over."""
    stop()
    countdown(INIT_DELAY, label="Init delay")   # arming pause with live timer
    steer(SERVO_CENTER)           # make sure wheels are pointing straight
    forward()
    countdown(INIT_FORWARD, label="Driving")    # initial push with live timer
    stop()

# -----------------------------------------------------------------------------
# Vision pipeline
# -----------------------------------------------------------------------------
def process_frame(frame):
    """Very small placeholder pipeline: detect a dark line on a light floor
    and return the horizontal offset of its centroid from the image centre.
    Replace this with the perception model you actually want to run."""
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)

    moments = cv2.moments(mask)
    if moments["m00"] == 0:
        return None  # Nothing detected this frame.

    cx = int(moments["m10"] / moments["m00"])
    return cx - (FRAME_WIDTH // 2)   # px offset from centre


def decide(offset):
    """Translate the vision offset into a steering angle + drive command."""
    if offset is None:
        stop()
        return
    angle = (offset / (FRAME_WIDTH / 2)) * SERVO_MAX
    steer(angle)
    forward()

# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
def main():
    # Bench-test mode: just run the auto-init drive sequence and exit.
    if not CAMERA_ENABLED:
        try:
            auto_init()
        finally:
            stop()
        return

    if not camera.isOpened():
        raise RuntimeError("Could not open camera at index "
                           f"{PINS['CAMERA_INDEX']}")

    try:
        auto_init()
        while True:
            ok, frame = camera.read()
            if not ok:
                stop()
                continue

            offset = process_frame(frame)
            decide(offset)

            # Press 'q' in the preview window to exit cleanly.
            cv2.imshow("car-view", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.01)
    finally:
        stop()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
