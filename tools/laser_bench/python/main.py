"""
Bench del láser VL53L0X — lado Python (app "laser").

El MCU (sketch/laser.ino) publica la distancia frontal por el RouterBridge como el
RPC `get_front_mm` (mismo contrato que el proyecto grande). Este lado:
  * hace polling de esa lectura y la GUARDA en _last_mm,
  * la imprime en el Monitor, y
  * la SIRVE en una web (puerto 7000) que App Lab abre con "Open Web UI".

Autocontenido a propósito: NO depende de bridge_io.py del proyecto grande (por eso
falló antes con `ModuleNotFoundError: No module named 'bridge_io'` — ese módulo no
está en la carpeta python/ de este app).
"""
import threading
import time

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI

WEB_UI_PORT = 7000                 # default de App Lab -> lo abre "Open Web UI"
ui = WebUI(port=WEB_UI_PORT)

_last_mm = -1                      # última distancia leída (mm); -1 = sin lectura válida


def api_distance():
    """GET /api/distance -> {mm, ok}. La página lo consulta ~5 Hz para el readout."""
    return {"mm": _last_mm, "ok": _last_mm >= 0}


ui.expose_api("GET", "/api/distance", api_distance)


def _poll_distance():
    """Pide get_front_mm al MCU en bucle, actualiza _last_mm y loguea en el Monitor."""
    global _last_mm
    time.sleep(2.0)                # deja que App.run() levante el bridge antes de llamar
    while True:
        try:
            mm = Bridge.call("get_front_mm")
            if mm is None:
                _last_mm = -1
                print("[laser] sin respuesta del MCU (¿sketch flasheado / RPC registrado?)", flush=True)
            else:
                _last_mm = int(mm)
                if _last_mm < 0:
                    print("[laser] sin lectura válida (-1 / TIMEOUT del VL53L0X)", flush=True)
                else:
                    print(f"[laser] front = {_last_mm} mm", flush=True)
        except Exception as e:     # noqa: BLE001 — nunca dejes que un RPC tumbe el bench
            _last_mm = -1
            print(f"[laser] RPC get_front_mm falló: {e!r}", flush=True)
        time.sleep(0.2)            # ~5 Hz


# Hilo demonio para el polling + App.run() para mantener vivo el proceso y el bridge.
threading.Thread(target=_poll_distance, daemon=True, name="laser-poll").start()

print(f"[web] abre 'Open Web UI' en App Lab (http://<board-ip>:{WEB_UI_PORT})", flush=True)
App.run()
