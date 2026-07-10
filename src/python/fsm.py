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
CORNER_COOLDOWN_S = 2.5   # ignore new corner events this long after a turn (anti double-count;
                          # must outlast the corner sweep + exit-line crossing, but stay well
                          # under a straight-section transit so real corners never get eaten)

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

# --- front ultrasonic (HC-SR04, MCU get_front_mm RPC) -----------------------------
# THE fix for the reported "turned at the corner, then never went forward" bug:
# vision read the front as CLEAR while the nose was against the wall, so speed
# stayed at 70% and the old halt-detector (speed==0) never fired. The ranger sees
# the wall regardless of image framing: the MCU blocks forward drive itself, and
# HERE we detect the ranger-standstill and run the same reverse un-stick pulse.
FRONT_STUCK_MM = 110      # ranger closer than this while DRIVing = pressed on a wall
                          # (halved from 220 -- user: react 50% closer to the wall)
FRONT_CLEAR_MM = 150      # reverse pulse may end early once clearance exceeds this
FRONT_POLL_S   = 0.25     # how often to poll the MCU for the front distance

# While a 90-deg sweep is in progress the car has to keep rolling to come around
# (Ackermann steering cannot rotate in place), so this speed is FORCED for the
# whole sweep -- it overrides whatever vision commands, including a halt. A real
# collision is still guarded by the front ultrasonic (the MCU blocks forward
# under FRONT_HALT_MM itself).
TURN_DRIVE_PCT = 60       # % motor power through a corner (user: raised 35 -> 60)


class RaceController:
    """stage = 'open' | 'obstacle'. on_park = callable(direction:int) for Stage 2 only."""

    def __init__(self, stage="open", on_park=None):
        self.stage = stage
        self.on_park = on_park
        self.state = "WAIT"
        self.corner_count = 0
        self.turn_direction = 0          # latched +90 (CW/right) or -90 (CCW/left)
        self._last_corner_t = -1e9       # NOT 0.0: with a clock that starts near 0 (sim,
                                         # monotonic time) 0.0 means "corner at t=0" and the
                                         # cooldown silently eats a corner in the first second
        self._finish_t = None
        self._last_speed = None
        self._last_bias = None
        self._halt_t = None              # when the current continuous halt started
        self._unstick_t = 0.0            # when the reverse pulse started
        self._front_mm = None            # last front-ultrasonic reading (throttled poll)
        self._front_t = 0.0              # when we last polled it
        self._last_tel = 0.0             # last telemetry print

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

        # is a 90-deg sweep still in progress? (None = RPC hiccup -> treat as reached)
        at_ok = io.at_target()
        turning = at_ok is False

        # forward proximity speed + lateral bias (centering / pillar passing) to the MCU
        # (speed_pct None = "leave speed unchanged", e.g. the camera-off command).
        # Mid-turn the sweep speed is FORCED to TURN_DRIVE_PCT (see above): the car
        # drives THROUGH the corner at full turn power instead of freezing or
        # coasting on whatever vision happened to command.
        speed = cmd.speed_pct
        if turning and speed is not None:
            speed = TURN_DRIVE_PCT
        if speed is not None and speed != self._last_speed:
            io.set_speed(speed)
            self._last_speed = speed
        if cmd.steer_bias != self._last_bias:
            io.nudge(cmd.steer_bias)
            self._last_bias = cmd.steer_bias

        # throttled telemetry -> watch the corner behaviour in the App Lab console
        if now - self._last_tel > 0.7:
            self._last_tel = now
            print(f"[tel] {self.state} corner={self.corner_count}/{TOTAL_CORNERS} "
                  f"head={io.get_heading()} tgt={io.get_target()} "
                  f"spd={speed} bias={cmd.steer_bias} turn={cmd.turn} | {cmd.note}",
                  flush=True)

        # front-ultrasonic poll (throttled). Catches the reported bug: vision says
        # CLEAR while the nose is against a wall the camera band missed. The MCU
        # blocks forward on the ranger by itself; here we NOTICE the standstill.
        if now - self._front_t >= FRONT_POLL_S:
            self._front_t = now
            self._front_mm = io.get_front_mm()
        f = self._front_mm
        wall_pressed = isinstance(f, int) and 0 <= f < FRONT_STUCK_MM

        # deadlock breaker: vision-halt OR ranger-wall persisting -> reverse pulse.
        # Uses the POST-override speed: a camera halt that was floored to
        # TURN_DRIVE_PCT mid-turn is not "stuck" (the car is rolling) -- but the
        # ranger pressed-on-wall signal still counts even while turning.
        if speed == 0 or wall_pressed:
            if self._halt_t is None:
                self._halt_t = now
            elif now - self._halt_t > UNSTICK_AFTER_S:
                why = f"ultrasonic {f}mm" if wall_pressed else "vision halt"
                print(f"[fsm] stuck ({why}) -> UNSTICK (reverse pulse)", flush=True)
                io.set_drive_dir(-1)
                io.set_speed(UNSTICK_SPEED)
                self._unstick_t = now
                self.state = "UNSTICK"
                return
        else:
            self._halt_t = None

        # corner event — vision already latched it to fire once per physical line.
        # at_target gate: every WRO corner zone has BOTH an orange and a blue line
        # (entry + exit). Without this gate the EXIT line re-fires turn() while the
        # car is still sweeping the corner -> corners double-count -> "12 corners"
        # after 6 and a premature stop. While the 90-deg sweep is in progress
        # at_target() is False, so exit-line events are swallowed. (None = RPC
        # hiccup -> fail-open so a dropped call can never freeze corner counting.
        # at_ok was polled once above, before the speed override.)
        if (cmd.turn != 0 and (now - self._last_corner_t) > CORNER_COOLDOWN_S
                and at_ok is not False):
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
        """End the reverse pulse: forward again, hand speed control back to vision.
        Ends EARLY once the front ultrasonic reports real clearance (no over-backing)."""
        done = time.time() - self._unstick_t >= UNSTICK_REVERSE_S
        if not done:
            f = io.get_front_mm()
            if isinstance(f, int) and f > FRONT_CLEAR_MM:   # -1 (no sensor) stays timed
                done = True
        if done:
            io.set_drive_dir(+1)
            io.set_speed(0)              # stay stopped for one frame...
            self._last_speed = None      # ...so vision's next command re-sends speed
            self._halt_t = None
            self._front_mm = None        # stale "pressed" reading must not re-trigger
            self.state = "DRIVE"
            print("[fsm] unstick done -> DRIVE", flush=True)

    def _do_park(self):
        try:
            self.on_park(self.turn_direction or 90)
        finally:
            io.stop()
            self.state = "STOPPED"
            print("[fsm] STOPPED — parked", flush=True)
