"""
FuturoIng - Arduino UNO Q (App Lab): servo angle control.

Linux/Python side of the app:
  * The Web UI brick serves assets/index.html and a small REST API on port 7000.
  * The browser slider calls  GET /api/angle?angle=N ; we forward N to the MCU
    sketch over the RouterBridge RPC (Bridge.call), which drives the servo on D9.
  * GET /api/state lets the page sync the slider to the servo's current angle.

Note: this brick (FastAPI + fastapi_socketio) does NOT serve a Socket.IO client
library, so we use plain REST + fetch() instead of websockets — the pattern the
working community dashboards use.
"""
from arduino.app_utils import App
from arduino.app_bricks.web_ui import WebUI

import bridge_io as io   # the ONE place the firmware RPC contract lives

# The OpenCV vision pipeline is an independent module (python/vision.py). It is
# optional: if OpenCV/numpy aren't installed the rest of the app still runs and
# the camera toggle simply reports "unavailable".
try:
    from vision import VisionPipeline
    VISION_AVAILABLE = True
except Exception as _e:  # noqa: BLE001 - ImportError or missing cv2 at runtime
    VisionPipeline = None
    VISION_AVAILABLE = False
    print(f"[vision] module unavailable: {_e!r}", flush=True)

ui = WebUI()
current_angle = 79   # last angle we believe the servo is at (CENTER)


def clamp(angle):
    return max(0, min(180, int(angle)))


def api_set_angle(angle: int):
    """GET /api/angle?angle=N  -> command the servo, return the applied angle."""
    global current_angle
    angle = clamp(angle)
    current_angle = angle
    # set_angle() on the sketch applies it and returns the value actually written.
    applied = io.set_angle(angle)
    print(f"[servo] set_angle({angle}) -> {applied!r}", flush=True)
    if isinstance(applied, (int, float)):
        current_angle = int(applied)
    return {"angle": current_angle}


def api_state():
    """GET /api/state -> current angle (used by the page to sync on load)."""
    return {"angle": current_angle}


# --- Gyro heading-hold (drive-straight) controls --------------------------
# These forward to the sketch's provide_safe RPCs. The MCU owns the fast PD
# steering loop; here we just start/stop it and read back the live heading.
# In the WRO robot the *camera* code would call these same RPCs (e.g. turn(+90)
# at a detected corner, or a lateral bias for a red/green pillar).

def api_straight():
    """GET /api/straight -> begin a run: zero heading, motor on, hold enabled."""
    io.set_straight()
    print("[drive] set_straight", flush=True)
    return {"straight": True}


def api_turn(deg: int):
    """GET /api/turn?deg=90 -> bump the target heading by deg (corner)."""
    count = io.turn(deg)
    print(f"[drive] turn({deg}) -> count={count!r}", flush=True)
    return {"turns": count}


def api_stop():
    """GET /api/stop -> stop motor, recenter servo, disable hold."""
    global current_angle
    if io.stop() is not None:
        current_angle = 79  # CENTER -- the sketch recentered the servo
    print("[drive] stop", flush=True)
    return {"straight": False}


# ==========================================================================
# DC MOTOR (L298N) - speed control
# --------------------------------------------------------------------------
# The motor is physically driven by the sketch (L298N ENA=pin3, IN1=6, IN2=5);
# the MCU's set_speed(pct) scales its PWM duty. This section is the single place
# the Linux side sets drive speed -- both the web slider and the camera proximity
# logic funnel through set_drive_speed() so there's one source of truth.
# ==========================================================================
current_speed = 100        # last speed % we commanded


def set_drive_speed(pct):
    """Command the L298N drive speed as 0..100 % of MOTOR_SPEED (via the MCU)."""
    global current_speed
    pct = max(0, min(100, int(pct)))
    applied = io.set_speed(pct)
    current_speed = int(applied) if isinstance(applied, (int, float)) else pct
    return current_speed


def api_speed(pct: int):
    """GET /api/speed?pct=N -> set L298N drive speed (0..100%). 0 halts but the
    gyro keeps holding heading. The camera lowers this as obstacles approach."""
    return {"speed": set_drive_speed(pct)}


def api_motor(on: int = 1):
    """GET /api/motor?on=1|0 -> toggle ONLY the L298N drive motor (independent of
    steering)."""
    return {"motor": bool(io.set_motor(int(on)))}


# ==========================================================================
# SERVO / STEERING - gyro heading-hold toggle (independent of the motor)
# ==========================================================================
def api_steer(on: int = 1):
    """GET /api/steer?on=1|0 -> toggle ONLY the gyro heading-hold steering (servo);
    OFF straightens to center, the motor is unaffected."""
    return {"steer": bool(io.set_steer(int(on)))}


def api_nudge(bias: int):
    """GET /api/nudge?bias=N -> camera lateral steer bias in degrees (+right/-left).

    Used to hug a side past a red/green pillar or to veer off an approaching wall;
    bias=0 clears it. Returns the applied bias."""
    return {"bias": io.nudge(bias)}


def api_heading():
    """GET /api/heading -> live heading / target / steer for the UI to poll."""
    return {
        "heading": io.get_heading(),
        "target": io.get_target(),
        "steer": io.get_steer(),
    }


# ==========================================================================
# CAMERA / VISION  (toggleable OpenCV pipeline -> python/vision.py)
# --------------------------------------------------------------------------
# When ON, the pipeline reads the Logitech USB cam, detects pillars/walls, and
# calls _on_vision() each frame -> which forwards a proximity speed (to the
# L298N via set_drive_speed) and a steering bias (nudge). It runs in its own
# thread, so toggling it never blocks the Web UI.
# ==========================================================================
_last_sent = {"speed": None, "bias": None}


def _on_vision(cmd):
    """Vision callback: push speed/bias to the MCU (only when they change, so we
    don't spam the bridge), and fire a corner turn when the camera detects one."""
    # Corner trigger - already latched to fire once per corner in the pipeline.
    if getattr(cmd, "turn", 0):
        io.turn(cmd.turn)
        print(f"[vision] corner -> turn({cmd.turn})", flush=True)
    # speed_pct None = "leave speed unchanged" (the camera-off command) -- jumping
    # to a fixed speed on camera-off could floor the car at a close wall.
    if cmd.speed_pct is not None and cmd.speed_pct != _last_sent["speed"]:
        set_drive_speed(cmd.speed_pct)
        _last_sent["speed"] = cmd.speed_pct
    if cmd.steer_bias != _last_sent["bias"]:
        io.nudge(cmd.steer_bias)
        _last_sent["bias"] = cmd.steer_bias


camera = VisionPipeline(on_command=_on_vision) if VISION_AVAILABLE else None


def api_camera(on: int = 1):
    """GET /api/camera?on=1|0 -> start/stop the vision pipeline."""
    if not VISION_AVAILABLE or camera is None:
        return {"camera": False, "available": False}
    if int(on):
        camera.start()
    else:
        camera.stop()
        _last_sent["speed"] = _last_sent["bias"] = None
    return {"camera": camera.is_running(), "available": True}


def api_pillar(on: int = 1):
    """GET /api/pillar?on=1|0 -> enable/disable ONLY the red/green pillar behavior.
    The wall / corner assist keeps running regardless. This is the switch that lets
    you turn pillars off but keep the corner behavior."""
    if not VISION_AVAILABLE or camera is None:
        return {"pillar": False, "available": False}
    state = camera.set_pillar_enabled(bool(int(on)))
    print(f"[vision] pillar behavior -> {'ON' if state else 'OFF'}", flush=True)
    return {"pillar": state, "available": True}


def api_wall(on: int = 1):
    """GET /api/wall?on=1|0 -> enable/disable ONLY the wall / corner assist
    (black-wall centering + corner turn trigger). Pillars unaffected."""
    if not VISION_AVAILABLE or camera is None:
        return {"wall": False, "available": False}
    state = camera.set_wall_enabled(bool(int(on)))
    print(f"[vision] wall/corner assist -> {'ON' if state else 'OFF'}", flush=True)
    return {"wall": state, "available": True}


def api_vision():
    """GET /api/vision -> the latest vision command + toggle states (UI / logging)."""
    if not VISION_AVAILABLE or camera is None:
        return {"available": False, "running": False}
    last = camera.last
    return {
        "available": True,
        "running": camera.is_running(),
        "pillar": camera.pillar_enabled,
        "wall": camera.wall_enabled,
        "speed_pct": getattr(last, "speed_pct", None),
        "steer_bias": getattr(last, "steer_bias", None),
        "turn": getattr(last, "turn", None),
        "note": getattr(last, "note", None),
    }


ui.expose_api("GET", "/api/angle", api_set_angle)   # /api/angle?angle=120
ui.expose_api("GET", "/api/state", api_state)
ui.expose_api("GET", "/api/straight", api_straight)  # start straight run
ui.expose_api("GET", "/api/turn", api_turn)          # /api/turn?deg=90
ui.expose_api("GET", "/api/speed", api_speed)        # /api/speed?pct=60 (L298N speed)
ui.expose_api("GET", "/api/motor", api_motor)        # /api/motor?on=1|0  (drive motor)
ui.expose_api("GET", "/api/steer", api_steer)        # /api/steer?on=1|0  (steering hold)
ui.expose_api("GET", "/api/nudge", api_nudge)        # /api/nudge?bias=8 (camera avoid)
ui.expose_api("GET", "/api/stop", api_stop)          # stop + recenter
ui.expose_api("GET", "/api/heading", api_heading)    # live readout (poll)
ui.expose_api("GET", "/api/camera", api_camera)      # /api/camera?on=1|0 (toggle vision)
ui.expose_api("GET", "/api/pillar", api_pillar)      # /api/pillar?on=1|0 (pillar behavior)
ui.expose_api("GET", "/api/wall", api_wall)          # /api/wall?on=1|0   (corner assist)
ui.expose_api("GET", "/api/vision", api_vision)      # latest vision command

# Watchdog heartbeat: the MCU stops the motor if these pings go silent for ~1.5 s
# (i.e. this program crashed while the car was driving).
io.start_heartbeat()

# Keeps the app (and the Web UI server) alive. Work is request-driven via the
# REST handlers above, so no user_loop is needed.
App.run()
