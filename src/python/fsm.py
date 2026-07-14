"""fsm.py — SHARED autonomous race controller for both WRO FE stages.

The MCU owns the fast PD steering loop; this is the high-level state machine on the
Linux side. It consumes VisionCommands (one per camera frame, from vision.py) and
drives the car through the bridge_io RPC wrappers.

Shared flow (both stages):
    WAIT  -> START (set_straight)
    DRIVE -> 3 laps; count corners from the orange/blue line events (vision latches one
             fire per physical line; we add a time cooldown as belt-and-suspenders).
             STAGE 1 also fires corners from the SIDE ULTRASONICS (front wall close +
             inside wall gone) and centers in the corridor from their distances —
             the run no longer depends on the camera seeing the lines.
    Stage 1 (open):     after corner #12 -> FINISH  (coast OUT of the corner, then stop)
    Stage 2 (obstacle): after corner #12 -> PARK    (run the injected parking routine)

Pillar passing (Stage 2) is produced in vision.py (decide_obstacle) and arrives as
cmd.steer_bias, so this controller stays almost stage-agnostic — the only Stage-2
branch is the PARK hand-off.

NOTE: every threshold here is a STARTING POINT to tune on the real mat (see the brief
in other/fabrication-challenges.md and the README).

Every variable and function of the app is documented in src/DOCUMENTATION.txt.
"""
import threading
import time

import bridge_io as io

# ===========================================================================
# BOT CONFIGURATION — every race-controller knob lives here, first lines of
# code. Camera tuning is at the top of vision.py; MCU pins/speeds at the top
# of sketch/sketch.ino. See src/DOCUMENTATION.txt for the full reference.
# ===========================================================================
TOTAL_CORNERS = 12        # 3 laps x 4 corners = the WRO run (the lap counts on EXIT
                          # of the last corner). It was parked at 16 for a 4-lap
                          # practice session (2026-07-13) — which is exactly the
                          # "car doesn't stop after 3 laps" symptom; keep 12 unless
                          # deliberately testing longer runs.
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

# --- side ultrasonics (2x HC-SR04, MCU get_left_mm / get_right_mm RPCs) -----------
# Added 2026-07-13: the corners ("edges") no longer depend on the camera alone.
# The rangers answer the two questions the camera kept getting wrong on the mat:
#   1. "Am I centered in the corridor?" -> lane-centering bias (stage 1 only).
#      The camera's NOT-floor coverage looks OVER the 100 mm walls, so the side
#      strips mostly measured background, and decide_open pushed a phantom bias
#      every frame — which, under the MCU's vision-authority steering (a standing
#      bias makes the target FOLLOW the heading), re-aimed the hold nonstop and
#      the car wandered from the start line in BOTH directions. Real distances
#      cannot be fooled by what's beyond the wall.
#   2. "Did the corridor open for the corner?" -> ultrasonic corner trigger,
#      direction-agnostic (orange/CW and blue/CCW alike): front wall close + the
#      INSIDE wall gone = the corner is here, turn toward the opening. It shares
#      the camera-line trigger's gate (first source to fire wins), so the lines
#      remain primary when visible and the rangers carry the run when they're not.
SIDE_POLL_S        = 0.10  # poll period for the two side rangers (10 Hz per side)
US_CORNER_FRONT_MM = 620   # corner zone reached: front wall closer than this...
US_CORNER_OPEN_MM  = 1200  # ...AND the inside reading beyond this (or -1 = no echo)
US_CORNER_WALL_MM  = 900   # direction-latching corner only: the OUTSIDE must still
                           # see its wall under this, so one stray far reading can't
                           # latch the run's direction by itself
US_FORCE_FRONT_MM  = 350   # wall in the face -> turn NOW even if the inside ranger
                           # missed the opening (the MCU still hard-blocks forward
                           # under FRONT_HALT_MM, so this can't ram the wall)
OPEN_SLOW_FRONT_MM = 650   # stage 1: ease off when the front wall is nearer than
                           # this (replaces the camera front-band slowdown)
OPEN_SLOW_PCT      = 40    # % drive while closing on a corner wall (duty ~144 on the
                           # MCU's [70..255] mapping — above the L298N stall floor)
CENTER_VALID_MM    = 1000  # center only when BOTH sides see walls closer than this
                           # (an open corner side must never read as "hug me")
CENTER_DEADBAND_MM = 60    # |left-right| under this = centered -> bias EXACTLY 0, so
                           # the gyro heading-hold stays in charge on the straight
                           # (any standing bias re-aims it: vision-authority steering)
CENTER_GAIN        = 0.03  # deg of bias per mm of (right-left) imbalance
CENTER_MAX_BIAS    = 8     # clamp on the centering bias (deg)

# While a 90-deg sweep is in progress the car has to keep rolling to come around
# (Ackermann steering cannot rotate in place), so this speed is FORCED for the
# whole sweep -- it overrides whatever vision commands, including a halt. A real
# collision is still guarded by the front ultrasonic (the MCU blocks forward
# under FRONT_HALT_MM itself).
TURN_DRIVE_PCT = 60       # % motor power through a corner (user: raised 35 -> 60)

# --- sweep latch ------------------------------------------------------------------
# "Sweeping" is tracked with a LATCH owned by this controller (set when WE call
# turn(), cleared the first time at_target() reports the heading arrived), NOT by
# polling at_target() raw — the latch is the single source of "a sweep WE fired
# is running", immune to RPC hiccups (timeout failsafe) and to exit-line refires.
# It also keeps the sweep PURE-PD: lateral bias is forced to 0 for its duration.
# That bias-freeze is LOAD-BEARING under the MCU's vision-authority steering
# (2026-07-13): while a bias stands the MCU's target FOLLOWS the heading, so a
# stale bias mid-sweep wouldn't just bend the endpoint (the old bias/KP
# "kept the angle after the turn" bug) — it would drag the target along the arc
# and erase the 90-deg bump entirely.
SWEEP_TIMEOUT_S = 4.0     # failsafe end of the latch (a 90-deg sweep takes ~1-2 s;
                          # covers at_target RPC hiccups so the latch can't stick)


class RaceController:
    """stage = 'open' | 'obstacle'. on_park = callable(direction:int) for Stage 2 only."""

    def __init__(self, stage="open", on_park=None):
        """Build the controller in WAIT; nothing moves until start() is called.
        All per-run state (counters, timers, latches) is reset here, so a fresh
        RaceController per run = a clean slate."""
        self.stage = stage
        self.on_park = on_park
        self.pipeline = None             # optional VisionPipeline ref (set by the stage
                                         # launcher): keeps the camera's line-direction
                                         # latch in step when the RANGERS latch first
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
        self._sweeping = False           # sweep latch: True from our turn() call until
                                         # at_target() first reports True (or timeout)
        self._sweep_t = 0.0              # when the current sweep started
        self._front_mm = None            # last front-ultrasonic reading (throttled poll)
        self._front_t = 0.0              # when we last polled it
        self._left_mm = None             # last side-ranger readings (throttled poll);
        self._right_mm = None            # -1 = open/no echo, None = RPC failed
        self._side_t = 0.0               # when we last polled the sides
        self._last_tel = 0.0             # last telemetry print

    def start(self):
        """Begin the run: zero the gyro heading on the MCU (current direction
        becomes 'straight'), switch motor + heading-hold ON, enter DRIVE."""
        io.set_straight()                # zero heading, motor + hold ON
        self.state = "DRIVE"
        print(f"[fsm] START — stage={self.stage}", flush=True)

    def on_vision(self, cmd):
        """Main tick — called by VisionPipeline once per camera frame with a
        VisionCommand. Dispatches on self.state: forwards speed/bias to the MCU,
        watches the front ultrasonic for a pressed-on-wall standstill, runs the
        un-stick reverse pulse, counts corner-line events, and hands off to
        FINISH (stage 1) or PARK (stage 2) after corner #12."""
        if self.state in ("FINISH", "PARK", "STOPPED"):
            self._service_terminal()
            return
        if self.state == "UNSTICK":
            self._service_unstick()
            return
        if self.state != "DRIVE":
            return

        now = time.time()

        # sweep-latch service: the latch was set when WE fired turn(); it clears
        # the first time the MCU reports the heading arrived, with a timeout
        # failsafe. (at_target None = RPC hiccup -> leave the latch to the timeout,
        # so a dropped call can neither stick nor cut a sweep short.)
        if self._sweeping:
            at_ok = io.at_target()
            if at_ok is True or (now - self._sweep_t) > SWEEP_TIMEOUT_S:
                self._sweeping = False
        turning = self._sweeping

        # sensor polls (throttled): the front ranger feeds the stuck-watch and the
        # corner trigger; the side rangers feed lane centering and the corner
        # trigger. All best-effort — a failed RPC leaves the previous value.
        if now - self._front_t >= FRONT_POLL_S:
            self._front_t = now
            self._front_mm = io.get_front_mm()
        if now - self._side_t >= SIDE_POLL_S:
            self._side_t = now
            self._left_mm = io.get_left_mm()
            self._right_mm = io.get_right_mm()
        f = self._front_mm
        wall_pressed = isinstance(f, int) and 0 <= f < FRONT_STUCK_MM

        # forward proximity speed + lateral bias (centering / pillar passing) to the MCU
        # (speed_pct None = "leave speed unchanged", e.g. the camera-off command).
        # STAGE 1: the camera contributes no wall authority any more (lines only) —
        # the rangers own the corridor: ease off nearing the corner wall, center in
        # the lane from the real side distances (see the side-ultrasonic block above).
        # Mid-sweep the speed is FORCED to TURN_DRIVE_PCT (see above) and the bias
        # to 0 (pure-PD arc): the car drives THROUGH the corner at full turn power,
        # and a stale lateral bias cannot bend the sweep endpoint off the 90-deg
        # target (that bias-vs-PD equilibrium was the "kept the angle after the
        # turn" bug: heading settles bias/KP away from target while bias stands).
        speed = cmd.speed_pct
        bias = cmd.steer_bias
        if self.stage == "open":
            if speed is not None and isinstance(f, int) and 0 <= f < OPEN_SLOW_FRONT_MM:
                speed = min(speed, OPEN_SLOW_PCT)
            bias += self._us_centering_bias()
        if turning and speed is not None:
            speed = TURN_DRIVE_PCT
        if speed is not None and speed != self._last_speed:
            io.set_speed(speed)
            self._last_speed = speed
        if turning:
            if self._last_bias != 0:
                io.nudge(0)
                self._last_bias = 0
        elif bias != self._last_bias:
            io.nudge(bias)
            self._last_bias = bias

        # throttled telemetry -> watch the corner behaviour in the App Lab console
        if now - self._last_tel > 0.7:
            self._last_tel = now
            print(f"[tel] {self.state} corner={self.corner_count}/{TOTAL_CORNERS} "
                  f"head={io.get_heading()} tgt={io.get_target()} "
                  f"spd={speed} bias={bias} turn={cmd.turn} "
                  f"f={self._front_mm} L={self._left_mm} R={self._right_mm} | {cmd.note}",
                  flush=True)

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

        # corner event — from the camera (orange/blue line, latched one-shot in
        # vision) OR from the rangers (front wall close + inside open, stage 1).
        # First source to fire wins; both pass the same gates below.
        # Sweep gate: every WRO corner zone has BOTH an orange and a blue line
        # (entry + exit), so a re-fire during the 90-deg sweep would double-count
        # corners -> premature stop. The _sweeping LATCH (not raw at_target(), see
        # the config note) swallows mid-sweep events WITHOUT ever starving the
        # count on straights. Every dropped fire is logged so an undercount shows
        # up in the console instead of silently costing a lap.
        turn_req, turn_src = cmd.turn, "line"
        if turn_req == 0 and self.stage == "open":
            turn_req, turn_src = self._us_corner_turn(), "rangers"
        if turn_req != 0 and self.turn_direction and turn_req != self.turn_direction:
            # the run's direction is fixed (WRO randomizes it per run, not per
            # corner): a contradicting fire is sensor noise, never a real corner
            print(f"[fsm] corner fire ({turn_req:+d} {turn_src}) DROPPED: contradicts "
                  f"latched direction {self.turn_direction:+d}", flush=True)
            turn_req = 0
        if turn_req != 0:
            since_last = now - self._last_corner_t
            if self._sweeping or since_last <= CORNER_COOLDOWN_S:
                why = "mid-sweep" if self._sweeping else \
                    f"cooldown ({since_last:.1f}s <= {CORNER_COOLDOWN_S}s)"
                print(f"[fsm] corner fire ({turn_req:+d} {turn_src}) DROPPED: {why}",
                      flush=True)
            else:
                self._last_corner_t = now
                self.turn_direction = turn_req
                if self.pipeline is not None and \
                        getattr(self.pipeline, "turn_direction", 0) == 0:
                    # rangers latched first: hand the camera the same direction so
                    # its future line fires agree instead of fighting the run
                    self.pipeline.turn_direction = turn_req
                io.nudge(0)                  # sweep starts bias-free this instant,
                self._last_bias = 0          # not one camera frame later
                io.turn(turn_req)
                self._sweeping = True
                self._sweep_t = now
                self.corner_count += 1
                print(f"[fsm] corner {self.corner_count}/{TOTAL_CORNERS} -> "
                      f"turn({turn_req}) [{turn_src}]", flush=True)
                if self.corner_count >= TOTAL_CORNERS:
                    self._begin_finish()

    # --- side-ranger helpers (stage 1 corner assist + lane centering) -------------
    @staticmethod
    def _us_open(v):
        """That side reads 'open corridor': a real far distance, or no echo (-1 —
        beyond SIDE_CLEAR_MM on the MCU). None (RPC hiccup) is NOT open: never
        fire a corner on missing data."""
        return isinstance(v, int) and (v < 0 or v >= US_CORNER_OPEN_MM)

    @staticmethod
    def _us_walled(v):
        """That side still sees its wall (a real reading under US_CORNER_WALL_MM)."""
        return isinstance(v, int) and 0 <= v < US_CORNER_WALL_MM

    def _us_centering_bias(self):
        """Lane-centering bias (deg, +right/-left) from the side rangers.

        Acts ONLY when BOTH sides see walls inside CENTER_VALID_MM (an opened
        corner side must not read as 'hug the other wall') and only outside the
        deadband — so on a centered straight the bias is EXACTLY 0 and the gyro
        heading-hold is in charge. Off-center, the standing bias steers the car
        back while the MCU's vision-authority target follows along; re-entering
        the deadband zeroes it and the gyro holds the corrected direction."""
        L, R = self._left_mm, self._right_mm
        if not (isinstance(L, int) and isinstance(R, int)):
            return 0
        if not (0 <= L < CENTER_VALID_MM and 0 <= R < CENTER_VALID_MM):
            return 0
        diff = R - L                    # + = more room on the right = drifted left…
        if abs(diff) <= CENTER_DEADBAND_MM:
            return 0
        bias = int(CENTER_GAIN * diff)  # …so a positive bias steers RIGHT, off the wall
        return max(-CENTER_MAX_BIAS, min(CENTER_MAX_BIAS, bias))

    def _us_corner_turn(self):
        """Ultrasonic corner trigger: front wall close + the corridor open on the
        INSIDE of the run direction -> turn that way (0 / +90 / -90).

        Direction source, in order: the run's own latch (a corner already fired),
        the camera's line latch (a line was SEEN even if it never fired), and —
        for the very first corner with neither — the side signature itself:
        exactly one side open with the other still walled. The caller's sweep
        latch, cooldown and direction guard do the debouncing."""
        if self._sweeping:
            return 0
        f = self._front_mm
        if not (isinstance(f, int) and 0 <= f <= US_CORNER_FRONT_MM):
            return 0
        L, R = self._left_mm, self._right_mm
        latched = self.turn_direction
        if not latched and self.pipeline is not None:
            latched = getattr(self.pipeline, "turn_direction", 0)
        if latched:
            inside = L if latched < 0 else R
            if self._us_open(inside):
                return latched
            if f <= US_FORCE_FRONT_MM:   # wall in the face: the corner is HERE,
                return latched           # opening seen or not (MCU still guards 75 mm)
            return 0
        # first corner, no direction latched anywhere (the camera missed its line):
        # exactly one side open + the other still walled = unambiguous signature
        if self._us_open(R) and self._us_walled(L):
            return +90
        if self._us_open(L) and self._us_walled(R):
            return -90
        if f <= US_FORCE_FRONT_MM and isinstance(L, int) and isinstance(R, int):
            # in the face with no clean signature: turn toward the freer side
            lv = 10 ** 6 if L < 0 else L
            rv = 10 ** 6 if R < 0 else R
            if lv != rv:
                return -90 if lv > rv else +90
        return 0

    def _begin_finish(self):
        """Corner #12 reached: route to the stage ending — PARK (stage 2, runs the
        injected parking routine on its own thread) or FINISH (stage 1, coast out).
        The lap counts on EXIT of corner #12 -> coast out first, THEN stop/park."""
        if self.stage == "obstacle" and self.on_park:
            self.state = "PARK"
            print("[fsm] 12 corners -> PARK", flush=True)
            threading.Thread(target=self._do_park, daemon=True).start()
        else:
            self.state = "FINISH"
            self._finish_t = time.time()
            print("[fsm] 12 corners -> FINISH (coasting out of the corner)", flush=True)

    def _service_terminal(self):
        """Tick the end-of-run states: in FINISH, stop the car once the coast-out
        timer expires (PARK/STOPPED need no servicing — parking runs on its own
        thread and STOPPED is final)."""
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
        """Run the injected Stage-2 parking routine (blocking, so it lives on its
        own thread), then stop the car whatever happens — even if parking raises."""
        try:
            self.on_park(self.turn_direction or 90)
        finally:
            io.stop()
            self.state = "STOPPED"
            print("[fsm] STOPPED — parked", flush=True)
