"""simulator.py — virtual WRO track to test the REAL robot code, no hardware.

What is real vs simulated:
  * REAL, imported unmodified from src/python: vision.py (line/wall detection,
    decide_open), fsm.py (RaceController: corners, laps, un-stick), bridge_io.py.
  * SIMULATED: the board. A fake `arduino.app_utils.Bridge` routes every RPC to
    SimMCU, which mirrors sketch.ino's PD heading-hold (same KP/KD/CENTER/travel,
    driveDir reverse flip, duty floor mapping, laser forward-block hysteresis),
    plus a bicycle-model car and a raycast camera renderer of the WRO mat
    (white floor, black 100 mm walls, orange/blue corner lines).

Run (needs numpy + opencv in the Python you use):
    python tools/simulator.py                    # Stage 1 (open), report + PNG
    python tools/simulator.py --video            # also writes an .mp4 of the run
    python tools/simulator.py --seconds 240

Outputs land in tools/sim_out/: report.txt, trajectory.png, run.mp4 (optional).
"""
import argparse
import math
import os
import sys
import types

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_PY = os.path.normpath(os.path.join(HERE, "..", "src", "python"))
OUT_DIR = os.path.join(HERE, "sim_out")

# ---------------------------------------------------------------------------
# Fake board: arduino.app_utils.Bridge -> SimMCU (must exist BEFORE bridge_io)
# ---------------------------------------------------------------------------
MCU = None  # set in main()


class _Bridge:
    @staticmethod
    def call(name, *args):
        return MCU.rpc(name, *args)


class _App:
    @staticmethod
    def run():
        pass


_fake = types.ModuleType("arduino.app_utils")
_fake.Bridge = _Bridge
_fake.App = _App
_pkg = types.ModuleType("arduino")
_pkg.app_utils = _fake
sys.modules["arduino"] = _pkg
sys.modules["arduino.app_utils"] = _fake
sys.path.insert(0, SRC_PY)

import vision                      # noqa: E402  (the REAL modules under test)
import fsm                         # noqa: E402
import bridge_io                   # noqa: E402  (real wrappers -> fake Bridge)

# fsm paces cooldowns with time.time(); give it the SIM clock instead.
class SimClock:
    t = 0.0

fsm.time = types.SimpleNamespace(time=lambda: SimClock.t, sleep=lambda s: None)

# ---------------------------------------------------------------------------
# SimMCU — mirrors sketch.ino behaviour (constants copied from the sketch)
# ---------------------------------------------------------------------------
CENTER, STEER_TRAVEL = 79, 35
# CORRECTION_SIGN = -1 here: in THIS sim's axes (servo>CENTER = right turn =
# heading increases CW+) the PD needs the flipped sign for negative feedback —
# the exact same "flip if it steers the wrong way" knob as on the real car.
# (First sim run with +1 reproduced the runaway-donut failure. Knob validated.)
KP, KD, CORRECTION_SIGN = 1.2, 0.10, -1
RATE_DEADBAND = 0.5
MOTOR_SPEED, MOTOR_MIN_PWM = 140, 70
TOF_HALT_MM, TOF_RELEASE_MM, TOF_CLEAR_MM = 150, 200, 2000
TARGET_TOL = 5.0

VMAX = 0.45          # m/s at 100% duty (drivetrain guess; tune to your car)
WHEELBASE = 0.15     # m
SERVO_TO_WHEEL = 0.6 # wheel deg per servo deg offset (cap below)
WHEEL_MAX = 21.0     # deg


class SimMCU:
    """The car: sketch logic + physics. heading in deg, CLOCKWISE-positive."""

    def __init__(self, x, y, heading_deg):
        self.x, self.y = x, y                    # mm, world frame
        self.true_heading = heading_deg          # ground truth (CW+)
        self.heading = 0.0                       # integrated (gyro) heading
        self.target = 0.0
        self.yaw_rate = 0.0
        self.servo = CENTER
        self.steer_bias = 0
        self.speed_pct = 100
        self.motor_on = False
        self.hold_on = False
        self.drive_dir = +1
        self.turn_count = 0
        self.tof_blocked = False
        self.front_mm = -1
        self.v = 0.0                             # signed m/s
        self.log_turns = []                      # (t, x, y, deg)

    # ---- RPC surface (matches sketch provide_safe names) ----
    def rpc(self, name, *a):
        fn = getattr(self, "rpc_" + name, None)
        if fn is None:
            return None
        return fn(*a)

    def rpc_set_straight(self):
        self.heading = self.target = 0.0
        self.yaw_rate = 0.0
        self.turn_count = 0
        self.steer_bias = 0
        self.speed_pct = 100
        self.servo = CENTER
        self.hold_on = True
        self.motor_on = True
        return 1

    def rpc_turn(self, deg):
        self.target += deg
        self.turn_count += 1
        self.log_turns.append((SimClock.t, self.x, self.y, deg))
        return self.turn_count

    def rpc_stop(self):
        self.motor_on = self.hold_on = False
        self.steer_bias = 0
        self.speed_pct = 100
        self.servo = CENTER
        return 0

    def rpc_set_motor(self, on):  self.motor_on = bool(on); return int(self.motor_on)
    def rpc_set_steer(self, on):
        self.hold_on = bool(on)
        if not self.hold_on: self.servo = CENTER
        return int(self.hold_on)
    def rpc_set_speed(self, pct): self.speed_pct = max(0, min(100, int(pct))); return self.speed_pct
    def rpc_nudge(self, b):
        self.steer_bias = max(-STEER_TRAVEL, min(STEER_TRAVEL, int(b))); return self.steer_bias
    def rpc_set_drive_dir(self, d): self.drive_dir = -1 if d < 0 else +1; return self.drive_dir
    def rpc_at_target(self):   return 1 if abs(self.target - self.heading) < TARGET_TOL else 0
    def rpc_get_heading(self): return int(round(self.heading))
    def rpc_get_target(self):  return int(round(self.target))
    def rpc_get_steer(self):   return int(self.servo)
    def rpc_get_turns(self):   return self.turn_count
    def rpc_get_front_mm(self):return self.front_mm
    def rpc_ping(self):        return 1
    def rpc_set_start_button(self, on): return 1   # gate transparent in sim
    def rpc_arm_start(self):   return 1
    def rpc_start_ready(self): return 1
    def rpc_set_angle(self, a): self.hold_on = self.motor_on = False; self.servo = a; return a

    # ---- physics + control tick (dt seconds) ----
    def step(self, dt, world):
        # drive velocity: sketch duty mapping incl. MIN_PWM floor + laser block
        if self.motor_on and self.speed_pct > 0 and not (self.tof_blocked and self.drive_dir >= 0):
            duty = MOTOR_MIN_PWM + (MOTOR_SPEED - MOTOR_MIN_PWM) * self.speed_pct / 100.0
            self.v = VMAX * duty / MOTOR_SPEED * self.drive_dir
        else:
            self.v = 0.0

        # bicycle model: servo>CENTER = steer right = CW+ heading rate (forward)
        wheel = max(-WHEEL_MAX, min(WHEEL_MAX, (self.servo - CENTER) * SERVO_TO_WHEEL))
        head_dot = math.degrees(self.v / WHEELBASE) * math.tan(math.radians(wheel))
        self.true_heading += head_dot * dt
        thw = math.radians(-self.true_heading)          # world CCW radians
        self.x += self.v * 1000.0 * math.cos(thw) * dt
        self.y += self.v * 1000.0 * math.sin(thw) * dt

        # gyro (ideal) + deadband, integrate like the sketch
        rate = head_dot
        if -RATE_DEADBAND < rate < RATE_DEADBAND:
            rate = 0.0
        self.yaw_rate = rate
        self.heading += rate * dt

        # PD servo write (gated on hold, driveDir flip like the sketch)
        if self.hold_on:
            err = self.target - self.heading
            corr = CORRECTION_SIGN * self.drive_dir * (KP * err + KD * self.yaw_rate)
            s = int(round(CENTER - corr)) + self.steer_bias
            self.servo = max(CENTER - STEER_TRAVEL, min(CENTER + STEER_TRAVEL, s))

        # front laser + forward-block hysteresis (mirrors loop())
        d = world.raycast(self.x, self.y, thw)
        self.front_mm = -1 if d >= TOF_CLEAR_MM else int(d)
        if not self.tof_blocked and 0 <= self.front_mm < TOF_HALT_MM:
            self.tof_blocked = True
        elif self.tof_blocked and (self.front_mm < 0 or self.front_mm > TOF_RELEASE_MM):
            self.tof_blocked = False


# ---------------------------------------------------------------------------
# World: 3 m mat, 1 m corridor, orange/blue lines at every corner zone
# ---------------------------------------------------------------------------
class World:
    def __init__(self):
        s, i0, i1 = 3000.0, 1000.0, 2000.0
        self.segs = np.array([
            (0, 0, s, 0), (s, 0, s, s), (s, s, 0, s), (0, s, 0, 0),          # outer
            (i0, i0, i1, i0), (i1, i0, i1, i1), (i1, i1, i0, i1), (i0, i1, i0, i0),  # inner
        ], dtype=np.float64)
        t = 20.0
        # (x0,x1,y0,y1,color) — CCW run crosses BLUE entering each corner zone,
        # ORANGE leaving it (both exist per zone, like the real mat).
        self.blue = [(2000, 2000 + t, 0, 1000),     # BR entry (heading east)
                     (2000, 3000, 2000, 2000 + t),  # TR entry (heading north)
                     (1000 - t, 1000, 2000, 3000),  # TL entry (heading west)
                     (0, 1000, 1000 - t, 1000)]     # BL entry (heading south)
        self.orange = [(2000, 3000, 980, 1000),     # BR exit (heading north)
                       (2000, 2020, 2000, 3000),    # TR exit (heading west)
                       (0, 1000, 2000, 2020),       # TL exit (heading south)
                       (980, 1000, 0, 1000)]        # BL exit (heading east)

    def raycast(self, px, py, ang, dirs=None):
        """Min distance (mm) from (px,py) along ang (or array of angles)."""
        if dirs is None:
            dx, dy = np.array([math.cos(ang)]), np.array([math.sin(ang)])
        else:
            dx, dy = np.cos(dirs), np.sin(dirs)
        best = np.full(dx.shape, 1e9)
        for x1, y1, x2, y2 in self.segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            ok = np.abs(den) > 1e-9
            t = np.where(ok, ((x1 - px) * ey - (y1 - py) * ex) / np.where(ok, den, 1), 1e9)
            u = np.where(ok, ((x1 - px) * dy - (y1 - py) * dx) / np.where(ok, den, 1), -1)
            hit = ok & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
            best = np.where(hit & (t < best), t, best)
        return best[0] if dirs is None else best


# ---------------------------------------------------------------------------
# Camera renderer (320x240, matches vision.py expectations)
# ---------------------------------------------------------------------------
W, H = vision.FRAME_W, vision.FRAME_H
FX = FY = 277.0
CX, CY = W / 2.0, H / 2.0
CAM_H, WALL_H, PITCH = 80.0, 100.0, math.radians(10.0)

_rows = np.arange(H, dtype=np.float64)
_dep = PITCH + np.arctan((_rows - CY) / FY)              # depression per row
_ground = _dep > math.radians(0.6)
_F = np.where(_ground, CAM_H / np.tan(np.where(_ground, _dep, 1)), 1e9)  # fwd mm
_lat = (np.arange(W, dtype=np.float64) - CX) / FX        # lateral factor per col
_colang = np.arctan((np.arange(W) - CX) / FX)            # col angle offsets

COL_FLOOR = np.array([235, 235, 235], np.uint8)
COL_SKY = np.array([110, 110, 110], np.uint8)
COL_WALL = np.array([28, 28, 28], np.uint8)
COL_ORANGE = np.array([0, 140, 255], np.uint8)
COL_BLUE = np.array([255, 60, 0], np.uint8)


def render(world, x, y, thw):
    img = np.empty((H, W, 3), np.uint8)
    img[:] = COL_SKY
    # floor projection
    Fg = _F[:, None]
    Lg = Fg * _lat[None, :]
    wx = x + Fg * math.cos(thw) - Lg * math.sin(thw)
    wy = y + Fg * math.sin(thw) + Lg * math.cos(thw)
    g = _ground[:, None] & (Fg < 8000)
    on_mat = g & (wx >= 0) & (wx <= 3000) & (wy >= 0) & (wy <= 3000)
    img[on_mat] = COL_FLOOR
    for rects, col in ((world.blue, COL_BLUE), (world.orange, COL_ORANGE)):
        for x0, x1, y0, y1 in rects:
            m = on_mat & (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
            img[m] = col
    # walls: raycast per column, paint [v_top, v_base]
    d = world.raycast(x, y, None, dirs=thw - _colang)    # mm per column
    v_base = CY + FY * np.tan(np.arctan(CAM_H / d) - PITCH)
    v_top = CY + FY * np.tan(-np.arctan((WALL_H - CAM_H) / d) - PITCH)
    v0 = np.clip(v_top, 0, H - 1).astype(int)
    v1 = np.clip(v_base, 0, H - 1).astype(int)
    for u in range(W):
        if d[u] < 8000:
            img[v0[u]:v1[u] + 1, u] = COL_WALL
    return img


# ---------------------------------------------------------------------------
# Main run
# ---------------------------------------------------------------------------
def main():
    global MCU
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=200.0)
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)

    world = World()
    MCU = SimMCU(x=1500, y=500, heading_deg=0)          # bottom straight, facing east
    controller = fsm.RaceController(stage="open")
    pipe = vision.VisionPipeline(stage="open", on_command=controller.on_vision)

    vw = None
    if args.video:
        vw = cv2.VideoWriter(os.path.join(OUT_DIR, "run.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W + H, H))

    controller.start()
    dt = 1.0 / args.fps
    traj = []  # (x, y) per frame
    min_clear, unsticks, frames = 1e9, 0, int(args.seconds * args.fps)
    prev_state = controller.state

    for i in range(frames):
        frame = render(world, MCU.x, MCU.y, math.radians(-MCU.true_heading))
        cmd, _, _, _ = pipe.process(frame)
        controller.on_vision(cmd)
        MCU.step(dt, world)
        SimClock.t += dt
        traj.append((MCU.x, MCU.y))
        if MCU.front_mm >= 0:
            min_clear = min(min_clear, MCU.front_mm)
        if controller.state == "UNSTICK" and prev_state != "UNSTICK":
            unsticks += 1
        prev_state = controller.state

        if vw is not None:
            ann = frame.copy()
            if pipe._overlay:
                r, g_, wl, c = pipe._overlay
                vision.annotate(ann, r, g_, None, wl, c)
            top = np.full((H, H, 3), 40, np.uint8)
            k = H / 3000.0
            for x1, y1, x2, y2 in world.segs.astype(int):
                cv2.line(top, (int(x1 * k), H - 1 - int(y1 * k)),
                         (int(x2 * k), H - 1 - int(y2 * k)), (200, 200, 200), 1)
            for tx, ty in traj[::10]:
                cv2.circle(top, (int(tx * k), H - 1 - int(ty * k)), 1, (0, 200, 255), -1)
            cv2.circle(top, (int(MCU.x * k), H - 1 - int(MCU.y * k)), 3, (0, 0, 255), -1)
            vw.write(np.hstack([ann, top]))

        if controller.state == "STOPPED":
            break

    if vw is not None:
        vw.release()

    # ---- trajectory image ----
    S = 700
    k = S / 3200.0
    im = np.full((S, S, 3), 245, np.uint8)
    off = 100 * k
    for x1, y1, x2, y2 in world.segs:
        cv2.line(im, (int(off + x1 * k), S - 1 - int(off + y1 * k)),
                 (int(off + x2 * k), S - 1 - int(off + y2 * k)), (0, 0, 0), 2)
    for rects, col in ((world.blue, (255, 60, 0)), (world.orange, (0, 140, 255))):
        for x0, x1, y0, y1 in rects:
            cv2.rectangle(im, (int(off + x0 * k), S - 1 - int(off + y1 * k)),
                          (int(off + x1 * k), S - 1 - int(off + y0 * k)), col, -1)
    for tx, ty in traj[::3]:
        cv2.circle(im, (int(off + tx * k), S - 1 - int(off + ty * k)), 1, (180, 0, 180), -1)
    for (tt, tx, ty, deg) in MCU.log_turns:
        cv2.circle(im, (int(off + tx * k), S - 1 - int(off + ty * k)), 6, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(OUT_DIR, "trajectory.png"), im)

    # ---- report ----
    laps = MCU.turn_count / 4.0
    lines = [
        f"sim time            : {SimClock.t:.1f} s   ({i + 1} frames)",
        f"final FSM state     : {controller.state}",
        f"corners fired       : {MCU.turn_count}  (target 12)",
        f"laps completed      : {laps:.2f}",
        f"corner events       : " + ", ".join(f"{t:.1f}s({int(d):+d})" for t, _, _, d in MCU.log_turns),
        f"min front clearance : {min_clear if min_clear < 1e9 else 'n/a'} mm",
        f"un-stick pulses     : {unsticks}",
        f"latched direction   : {pipe.turn_direction} (blue=-90 expected on this CCW track)",
        f"final pos           : ({MCU.x:.0f}, {MCU.y:.0f}) mm  heading {MCU.true_heading:.0f} deg",
    ]
    report = "\n".join(lines)
    with open(os.path.join(OUT_DIR, "report.txt"), "w") as f:
        f.write(report + "\n")
    print(report)
    verdict = (controller.state == "STOPPED" and MCU.turn_count == 12
               and (min_clear >= 40 or min_clear == 1e9))
    print("\nVERDICT:", "PASS - 3 laps, 12 corners, clean stop" if verdict else "CHECK - see report")


if __name__ == "__main__":
    main()
