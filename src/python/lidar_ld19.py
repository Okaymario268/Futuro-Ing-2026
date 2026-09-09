# SPDX-License-Identifier: MPL-2.0
# Lector del lidar LDROBOT LD19 (protocolo estandar: paquetes de 47 bytes,
# 12 puntos por paquete, header 0x54 0x2C). Corre en un hilo propio, lee
# /dev/ttyUSB0 y mantiene el ultimo barrido de 360 grados en memoria.
#
# NOTA: no se valida el CRC8 del paquete -- la tabla exacta que usa el
# firmware puede variar entre lotes, y validar mal descartaria TODOS los
# paquetes en silencio. En su lugar nos apoyamos en la resincronizacion por
# cabecera (0x54 0x2C), que es robusta igual. Si en la practica se ven
# puntos claramente erraticos, ese es el primer lugar para revisar.

import math
import threading
import time

import serial

PACKET_LEN = 47
POINTS_PER_PACKET = 12


class LidarLD19:
    def __init__(self, port="/dev/ttyUSB0", baud=230400):
        self.port = port
        self.baud = baud
        self._ser = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        # Ultimo barrido: indice = grado (0..359), valor = distancia en mm o None
        self._scan = [None] * 360
        self.connected = False
        self.packets_ok = 0
        self.last_error = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get_scan(self):
        with self._lock:
            return list(self._scan)

    def _run(self):
        while self._running:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.5)
                self.connected = True
                self.last_error = None
                self._read_loop()
            except Exception as e:
                self.connected = False
                self.last_error = str(e)
                time.sleep(1.0)

    def _read_loop(self):
        buf = bytearray()
        while self._running:
            chunk = self._ser.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                # busca la cabecera 0x54 0x2C
                idx = buf.find(b"\x54\x2c")
                if idx < 0:
                    if len(buf) > 4096:
                        buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < PACKET_LEN:
                    break
                packet = bytes(buf[:PACKET_LEN])
                del buf[:PACKET_LEN]
                self._parse_packet(packet)

    def _parse_packet(self, p):
        speed = p[2] | (p[3] << 8)
        start_angle = (p[4] | (p[5] << 8)) / 100.0
        end_angle = (p[41] | (p[42] << 8)) / 100.0

        span = end_angle - start_angle
        if span < 0:
            span += 360.0
        step = span / max(1, POINTS_PER_PACKET - 1)

        with self._lock:
            for i in range(POINTS_PER_PACKET):
                off = 6 + i * 3
                dist_mm = p[off] | (p[off + 1] << 8)
                # intensity = p[off + 2]  (no usada por ahora)
                angle = (start_angle + step * i) % 360.0
                if dist_mm == 0:
                    continue
                self._scan[int(angle)] = dist_mm
            self.packets_ok += 1

    def sector_min_mm(self, center_deg, half_width_deg):
        """Distancia minima (mm) en un arco alrededor de center_deg. None si no hay datos."""
        with self._lock:
            best = None
            for d in range(-half_width_deg, half_width_deg + 1):
                a = int((center_deg + d) % 360)
                v = self._scan[a]
                if v is not None and (best is None or v < best):
                    best = v
            return best
