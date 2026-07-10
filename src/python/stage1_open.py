"""stage1_open.py — WRO Future Engineers · STAGE 1 · OPEN CHALLENGE entry point.

3 laps, NO obstacles. The car drives straight on the gyro heading-hold, turns at each
orange/blue corner line (turn direction latched from the FIRST line colour seen), and
auto-stops after corner #12.

This file contains NO pillar and NO parking code — that is the whole point of the
stage split. The shared pieces (gyro steering on the MCU, line/wall perception in
vision.py, the lap state machine in fsm.py) are reused unchanged.

Select this file as the App Lab program for an Open-Challenge round.

    Camera (vision, stage="open") --> RaceController (fsm) --> bridge_io --> MCU

NOTE: start is automatic a couple of seconds after launch so you can place the car.
For a competition start button, wire the UNO Q user button and call controller.start()
from its handler instead of the timer below.
"""
import threading
import time

from arduino.app_utils import App

from vision import VisionPipeline
from fsm import RaceController
import bridge_io as io

# --- Standalone entry point (only when run directly, not when imported) -------
if __name__ == "__main__":
    USE_START_BUTTON = True   # True = wait for the physical START button + 3-2-1 countdown
                              # (WRO rule 9.6). False = auto-start after START_DELAY_S.
    START_DELAY_S = 2.0       # used only when USE_START_BUTTON is False

    controller = RaceController(stage="open")
    camera = VisionPipeline(stage="open", on_command=controller.on_vision)

    def _begin():
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
