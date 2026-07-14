"""stage1_open.py — WRO Future Engineers · STAGE 1 · OPEN CHALLENGE entry point.

3 laps, NO obstacles. The car drives straight on the gyro heading-hold, centers in the
corridor on the SIDE ULTRASONICS, and turns at each corner on the first trigger to
fire: the orange/blue line (camera) or the ranger signature (front wall close + inside
open). The direction is latched at the first corner and reused; auto-stop after #12.

This file contains NO pillar and NO parking code — that is the whole point of the
stage split. The shared pieces (gyro steering on the MCU, line/wall perception in
vision.py, the lap state machine in fsm.py) are reused unchanged.

Select this file as the App Lab program for an Open-Challenge round.

    Camera (vision, stage="open") --> RaceController (fsm) --> bridge_io --> MCU

Every variable and function of the app is documented in src/DOCUMENTATION.txt.
"""
import threading
import time

from arduino.app_utils import App

from vision import VisionPipeline
from fsm import RaceController
import bridge_io as io

# ===========================================================================
# BOT CONFIGURATION — every knob of THIS file lives here, first lines of code.
# (Race tuning is at the top of fsm.py, camera tuning at the top of vision.py,
#  MCU pins/speeds at the top of sketch/sketch.ino — see src/DOCUMENTATION.txt)
# ===========================================================================
USE_START_BUTTON = True   # True  = wait for the physical START button + 3-2-1 countdown
                          #         on the LED matrix (WRO rule 9.6 competition start).
                          # False = auto-start START_DELAY_S after launch (bench tests).
START_DELAY_S    = 2.0    # s before auto-start (used only when USE_START_BUTTON is False)


# --- Standalone entry point (only when run directly, not when imported) -------
if __name__ == "__main__":
    controller = RaceController(stage="open")
    camera = VisionPipeline(stage="open", on_command=controller.on_vision)
    controller.pipeline = camera   # ranger direction latches sync the line latch

    def _begin():
        """Release the run: honour the start gate (physical button + countdown on
        the MCU) or the plain bench timer, then put the race controller in DRIVE."""
        io.set_start_button(USE_START_BUTTON)     # sync the toggle to the MCU + arm the gate
        if USE_START_BUTTON:
            print("[open] armed — press the START button (3-2-1 on the LED matrix)...", flush=True)
            while not io.start_ready():
                time.sleep(0.05)
        else:
            time.sleep(START_DELAY_S)
        controller.start()

    print("[open] Stage 1 — Open Challenge: starting camera + run", flush=True)
    io.start_heartbeat()   # MCU stops the motor if this program dies mid-run
    camera.start()
    threading.Thread(target=_begin, daemon=True).start()

    App.run()   # keep the program alive; work happens in the camera thread + FSM
