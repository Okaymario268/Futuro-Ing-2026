"""bridge_io.py — SHARED thin wrappers over the MCU's RouterBridge RPCs.

ALL Python code (stage controllers AND the manual Web UI) talks to the MCU only
through these helpers, so the firmware contract lives in exactly one place. Every
call is best-effort: a failed Bridge.call is logged and swallowed so a single
dropped RPC never crashes a run.

The MCU (src/sketch/sketch.ino) is a stage-agnostic motion server — it owns the fast
gyro PD steering loop. These wrappers are the only thing the Python race logic uses.
"""
import threading
import time

from arduino.app_utils import Bridge


def _call(name, *args):
    try:
        return Bridge.call(name, *args)
    except Exception as e:  # noqa: BLE001 — surface any bridge error, keep the run alive
        print(f"[bridge] {name}{args} FAILED: {e!r}", flush=True)
        return None


# --- run control ---
def set_straight():   return _call("set_straight")     # zero heading, motor + hold ON
def stop():           return _call("stop")             # motor off, recenter, hold OFF

# --- independent subsystem switches ---
def set_motor(on):    return _call("set_motor", 1 if on else 0)  # drive motor alone
def set_steer(on):    return _call("set_steer", 1 if on else 0)  # heading-hold alone

# --- steering / drive (the gyro PD loop lives on the MCU) ---
def set_angle(a):     return _call("set_angle", int(a))     # manual servo (drops auto)
def steer_dir(d):     return _call("steer_dir", int(d))     # jog steer -1/0/+1 (keeps motor)
def turn(deg):        return _call("turn", int(deg))   # bump heading target by +/-deg
def nudge(bias):      return _call("nudge", int(bias)) # camera lateral bias, deg (+right/-left)
def set_speed(pct):   return _call("set_speed", int(pct))   # 0..100 % of MOTOR_SPEED
def set_drive_dir(d): return _call("set_drive_dir", int(d)) # +1 forward, -1 reverse (parking)

# --- readback ---
def at_target():
    """True when heading is within tolerance of target; None if the RPC failed
    (callers treat None as 'unknown', NOT as False -- fail-open, never stalls a run)."""
    r = _call("at_target")
    return None if r is None else bool(r)
def get_heading():    return _call("get_heading")      # integrated yaw, deg
def get_target():     return _call("get_target")       # heading setpoint, deg
def get_steer():      return _call("get_steer")        # last servo angle written
def get_turns():      return _call("get_turns")         # corners executed (MCU turnCount)
def get_front_mm():   return _call("get_front_mm")      # front ultrasonic, mm (-1 none, None = RPC fail)

# --- start-button gate (physical button + LED-matrix 3-2-1 countdown on the MCU) ---
def set_start_button(on): return _call("set_start_button", 1 if on else 0)  # toggle gate
def arm_start():          return _call("arm_start")    # re-arm: wait for the next press
def start_ready():        return bool(_call("start_ready"))  # True once countdown finished


# --- watchdog heartbeat -----------------------------------------------------------
# The MCU auto-stops the motor if pings go silent for WATCHDOG_MS (Python crashed
# mid-run). Call start_heartbeat() once at program start; pinging at 2 Hz against a
# 1.5 s timeout leaves ~2 missed beats of margin. Safe to call more than once.
_heartbeat_started = False


def start_heartbeat(period_s=0.5):
    global _heartbeat_started
    if _heartbeat_started:
        return
    _heartbeat_started = True

    def _beat():
        time.sleep(2.0)          # let App.run() bring the bridge up before pinging
        while True:
            _call("ping")
            time.sleep(period_s)

    threading.Thread(target=_beat, daemon=True, name="mcu-heartbeat").start()
    print("[bridge] watchdog heartbeat started", flush=True)
