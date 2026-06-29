"""bridge_io.py — SHARED thin wrappers over the MCU's RouterBridge RPCs.

Both stage controllers (open / obstacle) talk to the MCU only through these helpers,
so the firmware contract lives in exactly one place. Every call is best-effort: a
failed Bridge.call is logged and swallowed so a single dropped RPC never crashes a run.

The MCU (src/sketch/sketch.ino) is a stage-agnostic motion server — it owns the fast
gyro PD steering loop. These wrappers are the only thing the Python race logic uses.
"""
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

# --- steering / drive (the gyro PD loop lives on the MCU) ---
def turn(deg):        return _call("turn", int(deg))   # bump heading target by +/-deg
def nudge(bias):      return _call("nudge", int(bias)) # camera lateral bias, deg (+right/-left)
def set_speed(pct):   return _call("set_speed", int(pct))   # 0..100 % of MOTOR_SPEED
def set_drive_dir(d): return _call("set_drive_dir", int(d)) # +1 forward, -1 reverse (parking)

# --- readback ---
def at_target():      return bool(_call("at_target"))  # heading within tolerance of target
def get_heading():    return _call("get_heading")      # integrated yaw, deg
def get_turns():      return _call("get_turns")         # corners executed (MCU turnCount)
