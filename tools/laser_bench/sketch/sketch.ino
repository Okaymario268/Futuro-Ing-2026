/*
 * Bench del láser VL53L0X — lado MCU (app "laser").
 * Lee el VL53L0X (breakout HW-843) con la librería Pololu (single-file, compila en
 * el core arduino:zephyr de la UNO Q) y PUBLICA la distancia por el RouterBridge
 * como el RPC `get_front_mm` -> el lado Python (python/main.py) hace polling y la
 * muestra en la Web UI.
 *
 * Wiring: VIN->3.3V  GND->GND  SDA->A4  SCL->A5  (mismos pines que el gyro; A4/A5 del
 * header, NO el conector Qwiic). Requiere core arduino:zephyr >= 0.55.2 para ver I2C.
 */
#include <Arduino_RouterBridge.h>   // App Lab: puente RPC con Python
#include <Wire.h>
#include <VL53L0X.h>                 // librería de Pololu

VL53L0X sensor;
int frontMM = -1;                    // última distancia en mm (-1 = sin lectura válida)

// RPC que Python invoca para recibir la distancia. Mismo nombre/contrato que usa el
// proyecto grande (bridge_io.get_front_mm / get_front_mm() del sketch.ino).
int get_front_mm() { return frontMM; }

void setup() {
  Bridge.begin();
  Monitor.begin(115200);
  Monitor.println("Erunning");

  Wire.begin();
  Wire.setTimeout(2500);             // un bus colgado da error en vez de congelarse

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Monitor.println("Error initializing VL53L0X");
    while (1) delay(10);
  }
  Monitor.println("VL53L0X initialized!");
  sensor.startContinuous();

  Bridge.provide_safe("get_front_mm", get_front_mm);   // <-- expone la lectura a Python
  Monitor.println("bridge listo: get_front_mm registrado");
}

void loop() {
  // RouterBridge despacha los provide_safe() aquí, en contexto de loop().
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    frontMM = -1;
    Monitor.println("TIMEOUT");
  } else {
    frontMM = (int)mm;
    Monitor.println("Distance: " + String(frontMM) + " mm");
  }
  delay(50);
}
