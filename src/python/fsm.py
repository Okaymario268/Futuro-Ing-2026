"""fsm.py — SHARED autonomous race controller for both WRO FE stages.

The MCU owns the fast PD steering loop; this is the high-level state machine on the
Linux side. It consumes VisionCommands (one per camera frame, from vision.py) and
drives the car through the bridge_io RPC wrappers.

Shared flow (both stages):
    WAIT  -> START (set_straight)
    DRIVE -> 3 laps; count corners from the orange/blue line events (vision latches one
             fire per physical line; we add a time cooldown as belt-and-suspenders)
    Stage 1 (open):     after corner #12 -> FINISH  (coast OUT of the corner, then stop)
    Stage 2 (obstacle): after corner #12 -> PARK    (run the injected parking routine)

Pillar passing (Stage 2) is produced in vision.py (decide_obstacle) and arrives as
cmd.steer_bias, so this controller stays almost stage-agnostic — the only Stage-2
branch is the PARK hand-off.

NOTE: every threshold here is a STARTING POINT to tune on the real mat (see the brief
in other/fabrication-challenges.md and the README).
"""
import threading
import time

import bridge_io as io

TOTAL_CORNERS = 12        # 3 laps x 4 corners. WRO: a lap counts on EXIT of corner #12.
FINISH_COAST_S = 1.6      # drive OUT of the 12th corner into the finish section, then stop
CORNER_COOLDOWN_S = 1.2   # ignore new corner events this long after a turn (anti double-count)

# --- front-halt deadlock breaker ("un-stick") -------------------------------------
# vision halts the car when a wall fills the front view; but once stopped the view
# never changes, so without recovery the car would stand there for the rest of the
# round. After UNSTICK_AFTER_S halted we back up briefly (the MCU PD loop stays
# stable in reverse — the sketch flips the correction sign with drive dir), then
# resume and let vision re-decide. Same idea as the old Pi Main.py "wall close ->
# reverse + look for a new approach".
UNSTICK_AFTER_S   = 2.0   # s of continuous halt in DRIVE before backing up
UNSTICK_REVERSE_S = 0.7   # s of reverse pulse
UNSTICK_SPEED     = 45    # % drive speed during the pulse


class RaceController:
    """stage = 'open' | 'obstacle'. on_park = callable(direction:int) for Stage 2 only."""

    def __init__(self, stage="open", on_park=None):
        self.stage = stage
        self.on_park = on_park
        self.state = "WAIT"
        self.corner_count = 0
        self.turn_direction = 0          # latched +90 (CW/right) or -90 (CCW/left)
        self._last_corner_t = 0.0
        self._finish_t = None
        self._last_speed = None
        self._last_bias = None
        self._halt_t = None              # when the current continuous halt started
        self._unstick_t = 0.0            # when the reverse pulse started

    def start(self):
        io.set_straight()                # zero heading, motor + hold ON
        self.state = "DRIVE"
        print(f"[fsm] START — stage={self.stage}", flush=True)

    # ---- called by VisionPipeline once per camera frame ----
    def on_vision(self, cmd):
        if self.state in ("FINISH", "PARK", "STOPPED"):
            self._service_terminal()
            return
        if self.state == "UNSTICK":
            self._service_unstick()
            return
        if self.state != "DRIVE":
            return

        now = time.time()

        # forward proximity speed + lateral bias (centering / pillar passing) to the MCU
        # (speed_pct None = "leave speed unchanged", e.g. the camera-off command)
        if cmd.speed_pct is not None and cmd.speed_pct != self._last_speed:
            io.set_speed(cmd.speed_pct)
            self._last_speed = cmd.speed_pct
        if cmd.steer_bias != self._last_bias:
            io.nudge(cmd.steer_bias)
            self._last_bias = cmd.steer_bias

        # front-halt deadlock breaker: halted too long -> brief reverse pulse
        if cmd.speed_pct == 0:
            if self._halt_t is None:
                self._halt_t = now
            elif now - self._halt_t > UNSTICK_AFTER_S:
                print("[fsm] halted too long -> UNSTICK (reverse pulse)", flush=True)
                io.set_drive_dir(-1)
                io.set_speed(UNSTICK_SPEED)
                self._unstick_t = now
                self.state = "UNSTICK"
                return
        else:
            self._halt_t = None

        # corner event — vision already latched it to fire once per physical line
        if cmd.turn != 0 and (now - self._last_corner_t) > CORNER_COOLDOWN_S:
            self._last_corner_t = now
            self.turn_direction = cmd.turn
            io.turn(cmd.turn)
            self.corner_count += 1
            print(f"[fsm] corner {self.corner_count}/{TOTAL_CORNERS} -> turn({cmd.turn})",
                  flush=True)
            if self.corner_count >= TOTAL_CORNERS:
                self._begin_finish()

    def _begin_finish(self):
        # the lap counts on EXIT of corner #12 -> coast out first, THEN stop / park
        if self.stage == "obstacle" and self.on_park:
            self.state = "PARK"
            print("[fsm] 12 corners -> PARK", flush=True)
            threading.Thread(target=self._do_park, daemon=True).start()
        else:
            self.state = "FINISH"
            self._finish_t = time.time()
            print("[fsm] 12 corners -> FINISH (coasting out of the corner)", flush=True)

    def _service_terminal(self):
        if self.state == "FINISH" and self._finish_t is not None:
            if time.time() - self._finish_t >= FINISH_COAST_S:
                io.stop()
                self.state = "STOPPED"
                print("[fsm] STOPPED — 3 laps complete", flush=True)

    def _service_unstick(self):
        """End the reverse pulse: forward again, hand speed control back to vision."""
        if time.time() - self._unstick_t >= UNSTICK_REVERSE_S:
            io.set_drive_dir(+1)
            io.set_speed(0)              # stay stopped for one frame...
            self._last_speed = None      # ...so vision's next command re-sends speed
            self._halt_t = None
            self.state = "DRIVE"
            print("[fsm] unstick done -> DRIVE", flush=True)

    def _do_park(self):
        try:
            self.on_park(self.turn_direction or 90)
        finally:
            io.stop()
            self.state = "STOPPED"
            print("[fsm] STOPPED — parked", flush=True)
