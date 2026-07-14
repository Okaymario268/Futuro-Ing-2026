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

Every variable and function of the app is documented in src/DOCUMENTATION.txt.
"""
import socket
import threading
import time

from arduino.app_utils import App
from arduino.app_bricks.web_ui import WebUI

try:
    from fastapi.responses import StreamingResponse   # ships with the web_ui brick
except Exception:  # noqa: BLE001
    StreamingResponse = None

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

# Autonomous stage launcher (Stage 1 Open / Stage 2 Obstacle) run straight from the
# Web UI. Needs the camera, so it's gated on VISION_AVAILABLE. parallel_park is imported
# from stage2_obstacle (its run code is under `if __name__ == "__main__"`, so importing
# it has no side effects).
STAGES_AVAILABLE = False
RaceController = None
parallel_park = None
if VISION_AVAILABLE:
    try:
        from fsm import RaceController
        from stage2_obstacle import parallel_park
        STAGES_AVAILABLE = True
    except Exception as _e:  # noqa: BLE001
        print(f"[stage] launcher unavailable: {_e!r}", flush=True)

# ===========================================================================
# BOT CONFIGURATION — every Web-UI-side knob lives here, first lines of code.
# (Race tuning is at the top of fsm.py, camera tuning at the top of vision.py,
#  MCU pins/speeds at the top of sketch/sketch.ino — see src/DOCUMENTATION.txt)
# ===========================================================================
WEB_UI_PORT    = 7000   # App Lab default port — its "Open Web UI" button auto-opens
                        # this one (a custom port won't pop up in App Lab). Over WiFi
                        # also reachable at http://<board-ip>:7000 from any device.
SERVO_CENTER   = 79     # servo angle for straight ahead — MUST match CENTER in sketch.ino
TURN_DRIVE_PCT = 60     # % speed forced while a corner sweep is in progress (kept in
                        # step with fsm.TURN_DRIVE_PCT) -- see the halt override in _on_vision
SWEEP_TIMEOUT_S = 4.0   # failsafe end of the sweep latch (mirrors fsm.SWEEP_TIMEOUT_S):
                        # a 90-deg sweep takes ~1-2 s; covers at_target RPC hiccups

# --- Live state (updated at runtime — do NOT edit these to tune) ------------
current_angle     = SERVO_CENTER  # last angle we believe the servo is at
current_speed     = 100           # last drive speed % we commanded (L298N)
start_button_on   = False         # web toggle: gate a run behind the physical button + countdown
_start_fired      = False         # guard so the countdown fires set_straight only once per arm
active_controller = None          # RaceController while a Stage 1/2 run is active, else None (manual)
_last_sent = {"speed": None, "bias": None}   # last speed/bias pushed to the MCU (anti-spam)
_sweep = {"on": False, "t": 0.0}  # sweep latch for the MANUAL camera path (fsm has its own):
                                  # bias must stay 0 mid-sweep -- under the MCU's vision-
                                  # authority steering a standing bias would drag the target
                                  # along the arc and erase the 90-deg bump

ui = WebUI(port=WEB_UI_PORT)      # the Web UI server (serves assets/ + the REST API)


def clamp(angle):
    """Clamp any input to the servo's legal 0..180 deg command range (as int)."""
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
    """GET /api/stop -> stop motor, recenter servo, disable hold; end any stage run."""
    global current_angle, _start_fired
    if active_controller is not None:   # end an autonomous stage -> back to manual
        start_manual()
    if io.stop() is not None:
        current_angle = SERVO_CENTER  # the sketch recentered the servo
    if start_button_on:      # re-arm the start gate for the next press
        _start_fired = False
        io.arm_start()
    print("[drive] stop", flush=True)
    return {"straight": False}


# ==========================================================================
# DC MOTOR (L298N) - speed control
# --------------------------------------------------------------------------
# The motor is physically driven by the sketch (L298N ENB=pin4, IN4=3, IN3=2);
# the MCU's set_speed(pct) scales its PWM duty. This section is the single place
# the Linux side sets drive speed -- both the web slider and the camera proximity
# logic funnel through set_drive_speed() so there's one source of truth.
# (current_speed lives in the state block at the top of the file.)
# ==========================================================================
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


def api_jog(dir: str = "stop"):
    """GET /api/jog?dir=fwd|rev|stop -> manual motor jog (bench). Drives at the
    current speed-slider setting; 'stop' switches the motor off."""
    d = str(dir).lower()
    if d in ("fwd", "forward", "up"):
        io.set_drive_dir(1)
        io.set_motor(1)
        d = "fwd"
    elif d in ("rev", "reverse", "back", "down"):
        io.set_drive_dir(-1)
        io.set_motor(1)
        d = "rev"
    else:
        io.set_motor(0)
        d = "stop"
    print(f"[jog] motor {d}", flush=True)
    return {"jog": d}


# ==========================================================================
# SERVO / STEERING - gyro heading-hold toggle (independent of the motor)
# ==========================================================================
def api_steer(on: int = 1):
    """GET /api/steer?on=1|0 -> toggle ONLY the gyro heading-hold steering (servo);
    OFF straightens to center, the motor is unaffected."""
    return {"steer": bool(io.set_steer(int(on)))}


def api_steerpos(dir: str = "center"):
    """GET /api/steerpos?dir=left|center|right -> manual servo jog. Leaves the motor
    running (unlike the manual slider), so you can drive and steer at once."""
    d = str(dir).lower()
    code = -1 if d == "left" else 1 if d == "right" else 0
    applied = io.steer_dir(code)
    print(f"[jog] steer {d} -> {applied!r}", flush=True)
    return {"steer": d, "angle": applied}


def api_nudge(bias: int):
    """GET /api/nudge?bias=N -> camera lateral steer bias in degrees (+right/-left).

    Used to hug a side past a red/green pillar or to veer off an approaching wall;
    bias=0 clears it. Returns the applied bias."""
    return {"bias": io.nudge(bias)}


def api_heading():
    """GET /api/heading -> live heading / target / steer / ranger distances for the UI."""
    return {
        "heading": io.get_heading(),
        "target": io.get_target(),
        "steer": io.get_steer(),
        "front": io.get_front_mm(),   # HC-SR04 front distance, mm (-1 = no echo/sensor)
        "left": io.get_left_mm(),     # side rangers (corner assist), mm (-1 = open/none)
        "right": io.get_right_mm(),
        "gyro_ok": io.get_gyro_ok(),  # 0 = gyro off the I2C bus (is something on A4/A5?)
    }


# ==========================================================================
# CAMERA / VISION  (toggleable OpenCV pipeline -> python/vision.py)
# --------------------------------------------------------------------------
# When ON, the pipeline reads the Logitech USB cam, detects pillars/walls, and
# calls _on_vision() each frame -> which forwards a proximity speed (to the
# L298N via set_drive_speed) and a steering bias (nudge). It runs in its own
# thread, so toggling it never blocks the Web UI.
# (_last_sent and TURN_DRIVE_PCT live in the config/state block at the top.)
# ==========================================================================
def _on_vision(cmd):
    """Vision callback: push speed/bias to the MCU (only when they change, so we
    don't spam the bridge), and fire a corner turn when the camera detects one."""
    # Corner trigger - already latched to fire once per corner in the pipeline.
    # The sweep must start AND run bias-free (see _sweep in the config block).
    if getattr(cmd, "turn", 0):
        io.nudge(0)                 # bias-free this instant, not one frame later
        _last_sent["bias"] = 0
        io.turn(cmd.turn)           # (turn() on the MCU also zeroes its bias copy)
        _sweep["on"], _sweep["t"] = True, time.time()
        print(f"[vision] corner -> turn({cmd.turn})", flush=True)
    # sweep-latch service: clears the first time the heading arrives (at_target is
    # reliable mid-sweep because the bias is held at 0), or on the timeout failsafe.
    if _sweep["on"] and (io.at_target() is True
                         or time.time() - _sweep["t"] > SWEEP_TIMEOUT_S):
        _sweep["on"] = False
    # speed_pct None = "leave speed unchanged" (the camera-off command) -- jumping
    # to a fixed speed on camera-off could floor the car at a close wall.
    # While a 90-deg sweep is in progress the nose swings PAST the wall, so the
    # camera front band legitimately saturates -> its HALT must NOT pin the car
    # mid-turn: Ackermann steering cannot rotate in place, it has to keep rolling
    # to come around. Floor the speed instead; a real collision stays guarded by
    # the front ultrasonic (the MCU blocks forward under FRONT_HALT_MM).
    speed = cmd.speed_pct
    if speed == 0 and _sweep["on"]:
        speed = TURN_DRIVE_PCT
    if speed is not None and speed != _last_sent["speed"]:
        set_drive_speed(speed)
        _last_sent["speed"] = speed
    if _sweep["on"]:
        pass                        # bias stays 0 for the whole sweep
    elif cmd.steer_bias != _last_sent["bias"]:
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


def api_stream():
    """GET /api/stream -> live MJPEG camera feed (multipart). The Web UI shows it as
    <img src="/api/stream">. Reachable over WiFi like everything else. Needs the
    camera ON (the vision pipeline supplies the frames)."""
    if StreamingResponse is None or not VISION_AVAILABLE or camera is None:
        return {"stream": False, "available": False}

    def gen():
        """Yield JPEG frames as an MJPEG multipart stream (~10 fps); ends itself
        after ~10 s without a frame (camera turned off)."""
        blank = 0
        while True:
            jpg = camera.snapshot_jpeg()
            if jpg is not None:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                blank = 0
            else:
                blank += 1
                if blank > 100:          # ~10 s with no frame (camera off) -> end
                    break
            time.sleep(0.1)              # ~10 fps preview

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


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
        "lines": list(getattr(camera, "last_lines", (0, 0))),  # [orange_px, blue_px]
    }


# ==========================================================================
# START BUTTON gate (physical button + 3-2-1 LED-matrix countdown, on the MCU)
# --------------------------------------------------------------------------
# When enabled, pressing the wired button makes the MCU recalibrate the gyro
# (fresh zero for the run), zero the heading, and play 3-2-1 on the LED matrix.
# THEN this poller fires the run: the ARMED stage if one was selected in the UI
# (see api_stage/arm_stage), else a plain straight run. Toggle it from the web
# UI; it re-arms after each Stop.
# ==========================================================================
def api_startbtn(on: int = 1):
    """GET /api/startbtn?on=1|0 -> enable/disable the button + countdown start gate."""
    global start_button_on, _start_fired
    start_button_on = bool(int(on))
    io.set_start_button(start_button_on)   # toggles + arms the gate on the MCU
    _start_fired = False
    print(f"[start] gate {'ON' if start_button_on else 'OFF'}", flush=True)
    return {"start_button": start_button_on}


def api_startstate():
    """GET /api/startstate -> {enabled, ready, armed} for the UI to poll.
    armed = the stage waiting on the physical button ('open'/'obstacle'), else None."""
    ac = active_controller
    armed = ac.stage if (ac is not None and ac.state == "WAIT") else None
    return {"enabled": start_button_on,
            "ready": io.start_ready() if start_button_on else None,
            "armed": armed}


def _start_gate_poller():
    """When the gate is on, watch for the countdown to finish, then start the run:
    the ARMED stage if one is prepared (FSM in WAIT), else the plain straight run.
    The gyro was already recalibrated + zeroed by the MCU on the press itself."""
    global _start_fired
    time.sleep(2.0)              # let the app/bridge come up before polling
    while True:
        try:
            if start_button_on and not _start_fired and io.start_ready():
                ac = active_controller
                if ac is not None and ac.state == "WAIT":
                    _start_fired = True
                    ac.start()
                    print(f"[start] countdown done -> stage {ac.stage} GO", flush=True)
                elif ac is None:
                    _start_fired = True
                    io.set_straight()
                    print("[start] countdown done -> set_straight", flush=True)
                # ac mid-run (DRIVE/…): nothing to fire — leave the guard alone.
        except Exception as e:  # noqa: BLE001
            print(f"[start] poller error: {e!r}", flush=True)
        time.sleep(0.1)


# ==========================================================================
# AUTONOMOUS STAGE LAUNCHER — start Stage 1 (Open) / Stage 2 (Obstacle) from the UI
# --------------------------------------------------------------------------
# Swaps the manual camera pipeline for a stage pipeline + RaceController (the very
# same objects stage1_open.py / stage2_obstacle.py build when run standalone). The
# web button IS the start trigger here (bench/testing). Stop reverts to manual.
# WRO note: for an OFFICIAL run use the standalone stage program + physical start
# button, with wireless OFF -- this launcher is a dev convenience.
# ==========================================================================
def prepare_stage(stage):
    """Bring a stage up WITHOUT moving the car: swap in the stage vision pipeline
    (camera ON, feeding the FSM) and a fresh RaceController left in WAIT — its
    on_vision ignores frames until start(). The run begins on controller.start()."""
    global camera, active_controller, _last_sent
    if camera is not None:
        camera.stop()
    io.stop()                                   # reset motor/hold before starting
    _last_sent = {"speed": None, "bias": None}
    on_park = parallel_park if stage == "obstacle" else None
    active_controller = RaceController(stage=stage, on_park=on_park)
    camera = VisionPipeline(stage=stage, on_command=active_controller.on_vision)
    active_controller.pipeline = camera   # ranger direction latches sync the line latch
    camera.start()


def start_stage(stage):
    """Switch into an autonomous stage run IMMEDIATELY (web-click / bench path)."""
    prepare_stage(stage)
    active_controller.start()                   # zero heading, motor+hold ON, begin DRIVE
    print(f"[stage] started {stage}", flush=True)


def arm_stage(stage):
    """Prepare a stage and hand the GO to the PHYSICAL button: the camera runs,
    the FSM waits, and when the press finishes (the MCU recalibrates the gyro,
    zeroes the heading, plays 3-2-1) the gate poller fires controller.start()."""
    global _start_fired
    prepare_stage(stage)
    _start_fired = False
    io.arm_start()          # re-arm in case a previous countdown already fired
    print(f"[stage] {stage} ARMED — press the START button to run", flush=True)


def start_manual():
    """Return to manual mode: stop the stage pipeline, restore the manual one."""
    global camera, active_controller, _last_sent
    if camera is not None:
        camera.stop()
    io.stop()
    active_controller = None
    _last_sent = {"speed": None, "bias": None}
    camera = VisionPipeline(on_command=_on_vision)   # fresh manual pipeline (not started)
    print("[stage] back to manual", flush=True)


def api_stage(name: str = "open"):
    """GET /api/stage?name=open|obstacle|manual -> stage run / revert to manual.
    With the physical start gate ON the stage is ARMED instead of started: the
    pipeline + camera come up, and the BUTTON press (gyro recal + 3-2-1) is
    what begins the run. With the gate OFF it starts immediately (bench)."""
    if not STAGES_AVAILABLE:
        return {"stage": None, "available": False}
    n = str(name).lower()
    stage = "open" if n in ("open", "1", "stage1") else \
            "obstacle" if n in ("obstacle", "2", "stage2") else None
    if stage is None:
        start_manual()
        return {"stage": "manual", "available": True}
    if start_button_on:
        arm_stage(stage)
        return {"stage": stage, "armed": True, "available": True}
    start_stage(stage)
    return {"stage": stage, "armed": False, "available": True}


ui.expose_api("GET", "/api/angle", api_set_angle)   # /api/angle?angle=120
ui.expose_api("GET", "/api/state", api_state)
ui.expose_api("GET", "/api/straight", api_straight)  # start straight run
ui.expose_api("GET", "/api/turn", api_turn)          # /api/turn?deg=90
ui.expose_api("GET", "/api/speed", api_speed)        # /api/speed?pct=60 (L298N speed)
ui.expose_api("GET", "/api/motor", api_motor)        # /api/motor?on=1|0  (drive motor)
ui.expose_api("GET", "/api/jog", api_jog)            # /api/jog?dir=fwd|rev|stop (motor jog)
ui.expose_api("GET", "/api/steer", api_steer)        # /api/steer?on=1|0  (steering hold)
ui.expose_api("GET", "/api/steerpos", api_steerpos)  # /api/steerpos?dir=left|center|right
ui.expose_api("GET", "/api/nudge", api_nudge)        # /api/nudge?bias=8 (camera avoid)
ui.expose_api("GET", "/api/stop", api_stop)          # stop + recenter
ui.expose_api("GET", "/api/heading", api_heading)    # live readout (poll)
ui.expose_api("GET", "/api/camera", api_camera)      # /api/camera?on=1|0 (toggle vision)
ui.expose_api("GET", "/api/pillar", api_pillar)      # /api/pillar?on=1|0 (pillar behavior)
ui.expose_api("GET", "/api/wall", api_wall)          # /api/wall?on=1|0   (corner assist)
ui.expose_api("GET", "/api/vision", api_vision)      # latest vision command
ui.expose_api("GET", "/api/stream", api_stream)      # live MJPEG camera feed
ui.expose_api("GET", "/api/stage", api_stage)        # /api/stage?name=open|obstacle|manual
ui.expose_api("GET", "/api/startbtn", api_startbtn)  # /api/startbtn?on=1|0 (start gate)
ui.expose_api("GET", "/api/startstate", api_startstate)  # gate enabled/ready (poll)

# --------------------------------------------------------------------------
# Wireless access: the Web UI brick binds 0.0.0.0, so once the UNO Q is on WiFi
# it's reachable at http://<board-wifi-ip>:7000 from any device on that network.
# We print the board's IP at startup so you don't have to hunt for it.
# NOTE (WRO rule): wireless must be OFF during an official run -- this is a
# DEV / bench monitoring convenience, not for competition.
# --------------------------------------------------------------------------
def _print_access_urls():
    """Print how to reach the Web UI from a phone (once, at startup)."""
    # From INSIDE the App Lab container we can't read the board's real LAN/WiFi IP
    # (socket tricks return the container's 172.x address, which phones can't reach).
    # App Lab itself prints the correct "Network URL: http://<board-ip>:PORT" right
    # after this -- use THAT on your phone (same WiFi). Not localhost, not 172.x.
    print(f"[web] On your phone: open App Lab's 'Network URL' below "
          f"(http://<board-ip>:{WEB_UI_PORT}) -- phone on the SAME WiFi as the board.",
          flush=True)


_print_access_urls()

# Watchdog heartbeat: the MCU stops the motor if these pings go silent for ~1.5 s
# (i.e. this program crashed while the car was driving).
io.start_heartbeat()

# Poll the start-button gate so a physical press can begin a run in the web app.
threading.Thread(target=_start_gate_poller, daemon=True).start()

# Keeps the app (and the Web UI server) alive. Work is request-driven via the
# REST handlers above, so no user_loop is needed.
App.run()
