# SPDX-License-Identifier: MPL-2.0
# prueba-hardware -- panel web de diagnostico:
#   - Modulino Buttons / 3x Distance / Motors: reportados por el sketch via
#     Bridge.notify, reenviados al navegador via WebUI.
#   - Servo y prueba de motor: el navegador manda un mensaje, aca se llama
#     Bridge.call() hacia las funciones que el sketch expuso con
#     Bridge.provide("set_servo_pct"/"test_motor_pct").
#   - Camara: WebUI.expose_camera sirve el streaming directo.
#   - Lidar: hilo propio leyendo /dev/ttyUSB0, se manda el barrido al panel.
#
# Ver logs con: arduino-app-cli app logs user:prueba-hardware

import threading
import time

from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from arduino.app_peripherals.camera import Camera

from lidar_ld19 import LidarLD19

ui = WebUI(port=8080)  # 7000 no pasa por el hotspot movil de Windows (192.168.137.x); 8080 si, como en el proyecto de la serpiente

# Camara opcional: si el hub USB se desconecto (p.ej. al soldar) no debe
# tumbar toda la app -- el resto de sensores (Modulino/gyro/lidar) no
# dependen de ella.
try:
    camera = Camera(resolution=(640, 480), fps=15)
    camera.start()
    ui.expose_camera("/camera", camera)
except Exception as e:
    print(f"[camera] no disponible, sigo sin ella: {e}")
    camera = None

lidar = LidarLD19()
lidar.start()

state = {
    "buttons_ok": False,
    "dist_izq_ok": False,
    "dist_centro_ok": False,
    "dist_der_ok": False,
    "motor_ok": False,
    "dist_izq_mm": None,
    "dist_centro_mm": None,
    "dist_der_mm": None,
    "btn_a": False,
    "btn_b": False,
    "btn_c": False,
    "gyro_ok": False,
    "gyro_heading": None,
    "race_state": "idle",
    "race_corners": 0,
    "camera_line": None,
    "lidar_front_mm": None,
}

_RACE_STATE_NAMES = {0: "idle", 1: "drive", 2: "turn", 3: "stop"}


def push_state():
    ui.send_message("state", state)


# --- Recibe lo que el sketch reporta por Bridge.notify ---

def on_dist_izq_mm(mm: float):
    state["dist_izq_mm"] = mm if mm >= 0 else None
    push_state()

def on_dist_centro_mm(mm: float):
    state["dist_centro_mm"] = mm if mm >= 0 else None
    push_state()

def on_dist_der_mm(mm: float):
    state["dist_der_mm"] = mm if mm >= 0 else None
    push_state()

def on_live_buttons(ok: int):
    state["buttons_ok"] = bool(ok)
    push_state()

def on_live_dist_izq(ok: int):
    state["dist_izq_ok"] = bool(ok)
    push_state()

def on_live_dist_centro(ok: int):
    state["dist_centro_ok"] = bool(ok)
    push_state()

def on_live_dist_der(ok: int):
    state["dist_der_ok"] = bool(ok)
    push_state()

def on_live_motor(ok: int):
    state["motor_ok"] = bool(ok)
    push_state()

def on_btn_a_live(v: int):
    state["btn_a"] = bool(v)
    push_state()

def on_btn_b_live(v: int):
    state["btn_b"] = bool(v)
    push_state()

def on_btn_c_live(v: int):
    state["btn_c"] = bool(v)
    push_state()

def on_live_gyro(v: int):
    state["gyro_ok"] = bool(v)
    push_state()

def on_gyro_heading(deg: float):
    state["gyro_heading"] = deg
    push_state()

def on_race_state(v: int):
    name = _RACE_STATE_NAMES.get(int(v), "?")
    if name != state["race_state"]:
        print(f"[race] estado -> {name}")
    state["race_state"] = name
    push_state()

def on_race_corners(n: int):
    if n != state["race_corners"]:
        print(f"[race] esquina {n}/12")
    state["race_corners"] = n
    push_state()

for name, cb in [
    ("dist_izq_mm", on_dist_izq_mm),
    ("dist_centro_mm", on_dist_centro_mm),
    ("dist_der_mm", on_dist_der_mm),
    ("live_buttons", on_live_buttons),
    ("live_dist_izq", on_live_dist_izq),
    ("live_dist_centro", on_live_dist_centro),
    ("live_dist_der", on_live_dist_der),
    ("live_motor", on_live_motor),
    ("btn_a_live", on_btn_a_live),
    ("btn_b_live", on_btn_b_live),
    ("btn_c_live", on_btn_c_live),
    ("live_gyro", on_live_gyro),
    ("gyro_heading", on_gyro_heading),
    ("race_state", on_race_state),
    ("race_corners", on_race_corners),
]:
    Bridge.provide(name, cb)


# --- Mensajes que llegan DESDE el navegador (control manual) ---

def on_set_servo(sid, data):
    pct = int(max(-100, min(100, data.get("pct", 0))))
    Bridge.call("set_servo_pct", pct)

def on_test_motor(sid, data):
    pct = int(max(-100, min(100, data.get("pct", 0))))
    Bridge.call("test_motor_pct", pct)
    if pct != 0:
        # apaga solo despues de un pulso -- evita que quede girando si el
        # navegador se desconecta a mitad de la prueba. 2.5s (antes 0.8s,
        # muy corto para sentir como maneja de verdad en el piso).
        def _stop_later():
            time.sleep(2.5)
            Bridge.call("test_motor_pct", 0)
        threading.Thread(target=_stop_later, daemon=True).start()

def on_race_stop(sid, data):
    Bridge.call("race_stop", 0)

ui.on_message("set_servo", on_set_servo)
ui.on_message("test_motor", on_test_motor)
ui.on_message("race_stop", on_race_stop)
ui.on_connect(lambda sid: push_state())


# --- Barrido del lidar hacia el panel + deteccion de esquina por lidar ---
# El sensor central de distancia esta descartado para detectar esquinas
# (montado muy bajo, ve el piso -- confirmado en pista, choco sin disparar
# el giro). El lidar va mas arriba y SI ve la pared real. Se usa el sector
# frontal (+-15 grados alrededor de 0 = adelante, misma convencion que el
# radar del panel) como reemplazo.
LIDAR_CORNER_MM = 400  # punto de partida -- subir/bajar segun pruebas reales
LIDAR_CORNER_COOLDOWN_S = 1.5
_last_lidar_corner_ts = 0.0

def lidar_loop():
    global _last_lidar_corner_ts
    while True:
        time.sleep(0.2)
        scan = lidar.get_scan()
        ui.send_message("lidar", {
            "scan": scan,
            "connected": lidar.connected,
            "packets_ok": lidar.packets_ok,
            "error": lidar.last_error,
        })

        front_mm = lidar.sector_min_mm(0, 15)
        state["lidar_front_mm"] = front_mm
        push_state()
        if front_mm is not None and front_mm < LIDAR_CORNER_MM:
            now_ts = time.monotonic()
            if now_ts - _last_lidar_corner_ts > LIDAR_CORNER_COOLDOWN_S:
                _last_lidar_corner_ts = now_ts
                print(f"[lidar_corner] pared a {front_mm}mm -> disparo giro")
                try:
                    Bridge.call("camera_corner", 0)
                except Exception as e:
                    print(f"[lidar_corner] Bridge.call fallo: {e}")

threading.Thread(target=lidar_loop, daemon=True).start()


# --- Deteccion de linea de esquina (naranja/azul) por color, via camara ---
# El sensor central de distancia ve el piso en vez de la pared real (montado
# muy bajo), asi que no puede detectar esquinas de forma confiable todavia.
# Como respaldo/alternativa, se usa la camara para ver cuando el robot cruza
# una de las lineas naranja/azul que marcan el borde de cada seccion en la
# pista (mismo concepto que el robot anterior). Al detectar una, se llama
# Bridge -> camera_corner en el sketch, que dispara el mismo giro que el
# sensor central hubiera disparado.
#
# OJO: los rangos HSV de abajo son un punto de partida generico -- la cinta/
# pintura real de la pista puede necesitar ajuste. Tunear viendo estado.
# camera_line en el panel mientras se pasa el robot sobre cada linea a mano.
CAMERA_LINE_COOLDOWN_S = 1.5
_last_camera_corner_ts = 0.0

def _detect_corner_line(frame) -> str | None:
    import cv2
    import numpy as np

    h, w = frame.shape[:2]
    # ROI: franja horizontal cerca del robot (no hasta el borde, para evitar
    # el propio cuerpo del chasis si aparece en cuadro), centrada.
    y0, y1 = int(h * 0.60), int(h * 0.90)
    x0, x1 = int(w * 0.20), int(w * 0.80)
    roi = frame[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Naranja: hue bajo, alta saturacion/valor.
    orange_mask = cv2.inRange(hsv, (5, 120, 100), (25, 255, 255))
    # Azul: hue medio-alto.
    blue_mask = cv2.inRange(hsv, (95, 100, 60), (130, 255, 255))

    roi_px = roi.shape[0] * roi.shape[1]
    orange_frac = cv2.countNonZero(orange_mask) / roi_px
    blue_frac = cv2.countNonZero(blue_mask) / roi_px

    THRESH = 0.12  # 12% del ROI con ese color -> se considera "cruzando la linea"
    if orange_frac > THRESH and orange_frac >= blue_frac:
        return "naranja"
    if blue_frac > THRESH:
        return "azul"
    return None

def camera_line_loop():
    global _last_camera_corner_ts
    if camera is None:
        return
    while True:
        time.sleep(0.15)
        try:
            frame = camera.capture()
        except Exception as e:
            print(f"[camera_line] error capturando: {e}")
            continue
        if frame is None:
            continue
        try:
            color = _detect_corner_line(frame)
        except Exception as e:
            print(f"[camera_line] error detectando: {e}")
            continue

        state["camera_line"] = color
        push_state()

        if color is not None:
            now_ts = time.monotonic()
            if now_ts - _last_camera_corner_ts > CAMERA_LINE_COOLDOWN_S:
                _last_camera_corner_ts = now_ts
                print(f"[camera_line] linea {color} detectada -> disparo giro")
                try:
                    Bridge.call("camera_corner", 0)
                except Exception as e:
                    print(f"[camera_line] Bridge.call fallo: {e}")

threading.Thread(target=camera_line_loop, daemon=True).start()

App.run()
