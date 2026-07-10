/*
 * HW-843 (VL53L0X ToF laser ranger) -- STANDALONE bench / bring-up test
 * ============================================================================
 * Purpose: prove the front laser works on its own, BEFORE trusting it inside the
 * full robot firmware. It does nothing but init the VL53L0X and stream distance
 * to the App Lab Monitor, so you can wave your hand / a wall in front of it and
 * watch the millimetres change. No servo, no motor, no gyro, no Python.
 *
 * Board : Arduino UNO Q  ->  MCU STM32U585 / Zephyr  ->  FQBN arduino:zephyr:unoq
 * Sensor: VL53L0X breakout sold as "HW-843" (STMicro 940 nm time-of-flight laser)
 * Driver: Adafruit "Adafruit_VL53L0X" library (declared in this folder's sketch.yaml)
 *
 * WIRING (identical to the robot -- shared I2C bus with the GY-521 gyro):
 *   VL53L0X  VIN/VCC -> 3.3V      (NOT 5V unless your breakout has a regulator)
 *   VL53L0X  GND     -> GND
 *   VL53L0X  SDA     -> SDA  (or A4)   UNO Q I2C data,  Wire
 *   VL53L0X  SCL     -> SCL  (or A5)   UNO Q I2C clock, Wire
 *   VL53L0X  XSHUT / GPIO1        -> leave unconnected (single sensor)
 *   NOTE: the dedicated SDA/SCL header pins (top, by AREF) and A4/A5 are the
 *   SAME I2C bus on the Uno header layout -- either pair works with Wire. The
 *   I2C scan in setup() confirms the sensor is seen no matter which you used.
 *   I2C address is 0x29 (fixed). The gyro at 0x68 does not clash, so you may
 *   leave the gyro wired in -- the I2C scan below will list both.
 *
 * HOW TO RUN
 *   Easiest: open this folder as a sketch in Arduino App Lab (it pulls the
 *   RouterBridge that feeds the Monitor panel), Upload, then open the Monitor
 *   at 115200. You should see the WHO/scan lines, then a distance every 200 ms.
 *
 * READING IT ON THE UNO Q
 *   >>> I2C needs the arduino:zephyr core >= 0.55.2 for the A4/A5 header pins
 *       (older cores don't see devices there -- ArduinoCore-zephyr #301). If the
 *       scan finds NOTHING, update the core first, then re-check wiring/3.3V.
 *   The on-board Qwiic connector has been reported flaky -- prefer A4/A5.
 *
 * If you build this with plain arduino-cli / the classic IDE instead of App Lab,
 * you won't have RouterBridge: swap the two Monitor lines in setup() for
 * Serial.begin(115200) and change every "Monitor." below to "Serial.".
 */
#include <Arduino_RouterBridge.h>   // gives us Monitor.* -> App Lab serial monitor
#include <Wire.h>

// Guard the driver exactly like the main sketch: a missing library then fails
// LOUDLY in the Monitor instead of refusing to compile with a cryptic error.
#if __has_include(<Adafruit_VL53L0X.h>)
  #include <Adafruit_VL53L0X.h>
  #define HAVE_TOF 1
#else
  #define HAVE_TOF 0
#endif

// --- Config (mirrors the robot so the bench numbers are representative) -------
#define TOF_BUS        Wire   // A4/A5 header on the UNO Q. Try Wire1/Wire2 only if
                              // the scan below can't find 0x29 on Wire.
#define PRINT_MS       200    // how often to print a reading (5 Hz -> readable)

// The robot blocks forward drive under HALT and releases past RELEASE (hysteresis).
// Shown per-reading so you can see EXACTLY when the real firmware would react.
#define TOF_HALT_MM    150
#define TOF_RELEASE_MM 200
#define TOF_CLEAR_MM   2000   // >= this (or a timeout) = "nothing in range / open"

#if HAVE_TOF
Adafruit_VL53L0X tof;
#endif

// --- I2C bus scan: the #1 debugging aid when a sensor "isn't found" -----------
// On the UNO Q, endTransmission() returns only 0 (ok) / non-zero (error), which
// is all a presence scan needs.
void i2cScan() {
  Monitor.println("I2C scan on the TOF_BUS pins...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    TOF_BUS.beginTransmission(addr);
    if (TOF_BUS.endTransmission(true) == 0) {
      found++;
      Monitor.print("  device at 0x");
      Monitor.print(addr, HEX);
      if (addr == 0x29) Monitor.print("  <- VL53L0X (HW-843)");
      if (addr == 0x68) Monitor.print("  <- MPU-6050 gyro");
      Monitor.println("");
    }
  }
  if (found == 0)
    Monitor.println("  NONE found -> check 3.3V/GND, SDA->A4, SCL->A5, and core >= 0.55.2");
}

void setup() {
  Bridge.begin();
  Monitor.begin(115200);
  delay(200);
  Monitor.println("");
  Monitor.println("=== HW-843 / VL53L0X standalone laser test ===");

  TOF_BUS.begin();
  i2cScan();

#if HAVE_TOF
  // begin(addr, debug, &bus, config). VL53L0X_SENSE_LONG_RANGE = the long-range
  // profile (signal-rate limit 0.1 MCPS, VCSEL pre/final 18/14, 33 ms budget) --
  // WRO walls are matte black and swallow the 940 nm beam, so trade a little
  // speed/accuracy for reach. Same effective knobs as the robot. Left in single-
  // shot mode so loop() can use the canonical blocking rangingTest() below.
  if (tof.begin(0x29, false, &TOF_BUS, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE)) {
    Monitor.println("VL53L0X init: OK  (0x29, single-shot, long-range)");
    Monitor.println("Move a hand / wall in front of it -- distance streams below:");
  } else {
    Monitor.println("VL53L0X init: FAILED. Sensor not answering at 0x29.");
    Monitor.println("  -> if the scan above listed 0x29 it's a driver issue;");
    Monitor.println("     if it did NOT, it's wiring/power/core (see header notes).");
  }
#else
  Monitor.println("Adafruit_VL53L0X.h NOT FOUND -> add 'Adafruit_VL53L0X (1.2.5)' to sketch.yaml libraries.");
#endif
}

void loop() {
#if HAVE_TOF
  static unsigned long lastMs = 0;
  static unsigned long oor    = 0;
  static uint16_t      mn = 0xFFFF, mx = 0;

  if (millis() - lastMs < PRINT_MS) return;   // throttle prints (blocking rangingTest ~5 Hz)
  lastMs = millis();

  // Canonical Adafruit pattern: one blocking single-shot measurement. RangeStatus
  // 4 = out of range / no valid target; anything else = a good RangeMilliMeter.
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);

  if (measure.RangeStatus == 4) {
    oor++;
    Monitor.println("out of range (no target / too far, or flaky wiring) total=" + String(oor));
    return;
  }

  uint16_t mm = measure.RangeMilliMeter;

  // Classify against the robot's real forward-drive logic so this bench test
  // tells you not just "does it read" but "would the robot react correctly".
  const char* zone;
  if (mm >= TOF_CLEAR_MM)          zone = "OPEN  (nothing in range)";
  else if (mm < TOF_HALT_MM)       zone = "BLOCK (robot halts forward)";
  else if (mm < TOF_RELEASE_MM)    zone = "NEAR  (hysteresis band)";
  else                             zone = "CLEAR (robot drives)";

  if (mm < mn) mn = mm;
  if (mx < mm) mx = mm;

  Monitor.println(String(mm) + " mm  (" + String(mm / 10.0f, 1) + " cm)   " + zone +
                  "   [min " + String(mn) + " / max " + String(mx) + " mm]");
#else
  delay(1000);   // library missing: nothing to do but keep the sketch alive
#endif
}
