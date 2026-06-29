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

START_DELAY_S = 2.0   # time to place the car / clear hands after pressing Run

controller = RaceController(stage="open")
camera = VisionPipeline(stage="open", on_command=controller.on_vision)


def _begin():
    time.sleep(START_DELAY_S)
    controller.start()


print("[open] Stage 1 — Open Challenge: starting camera + run", flush=True)
camera.start()
threading.Thread(target=_begin, daemon=True).start()

App.run()   # keep the program alive; work happens in the camera thread + FSM
