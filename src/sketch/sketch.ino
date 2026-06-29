/*
 * FuturoIng - Arduino UNO Q (App Lab) - GY-521 (MPU-6050) yaw-gyro heading hold
 * ----------------------------------------------------------------------------
 * Keeps the existing architecture: the Linux/Python side serves the Web UI and
 * forwards commands over the RouterBridge RPC; provide_safe() handlers run in
 * loop() context (hardware-safe). loop() now ALSO runs a non-blocking PD steering
 * loop -- no delay() in loop(); the only delays are in setup() calibration.
 *
 *   Browser / camera (Python) -> Bridge.call -> RPC handler -> servo / motor / heading
 *
 * WHAT IT DOES
 *   - Reads the MPU-6050 gyro Z (yaw rate) over I2C with bare Wire (no vendor lib).
 *   - Calibrates the gyro zero-rate bias once in setup() (car held STILL).
 *   - Integrates yaw rate into a heading, and a 50 Hz PD loop steers the servo
 *     around CENTER=80 to drive the heading to a TARGET heading.
 *       * target = 0   -> drive straight (lane keeping)
 *       * turn(+/-90)  -> bump the target 90 deg -> the same loop sweeps the corner
 *     This makes the gyro the steering authority; the camera (Python side) only
 *     decides WHEN to call turn()/set_straight()/stop() -- it never touches the servo.
 *   - L298N DC drive motor: ENA=3, IN1=6, IN2=5 (simple forward / stop).
 *
 * RPCs: set_straight, turn, stop, get_heading, get_steer, get_target
 *       (+ existing manual set_angle / get_angle)
 *
 * >>> READ THIS ABOUT I2C ON THE UNO Q <<<
 *   The UNO Q's Zephyr core had a bug where the default Wire (A4/A5 header pins)
 *   did not see I2C devices (ArduinoCore-zephyr issue #301). It is FIXED in core
 *   >= 0.55.2: an MPU-6050 is then found at 0x68 on the A4/A5 header pins with
 *   plain Wire. If WHO_AM_I below does NOT print 0x68:
 *     1) update the arduino:zephyr core to the latest (>= 0.55.2), then
 *     2) if still failing, change  #define IMU_BUS  to Wire1 (or Wire2) and re-run.
 *   The on-board Qwiic connector has been reported flaky -- prefer the A4/A5 pins.
 *   Note: on the UNO Q, endTransmission() returns only 0 (ok) / 1 (any error).
 */
#include <Arduino_RouterBridge.h>
#include <Servo.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// I2C bus instance for the IMU  (change in ONE place if WHO_AM_I != 0x68)
// ---------------------------------------------------------------------------
#define IMU_BUS  Wire        // try Wire1, then Wire2, if Wire doesn't see 0x68

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
#define SERVO_PIN   9        // steering servo signal (PWM-capable on UNO Q)
#define MOTOR_ENA   3        // L298N ENA -> PWM speed   (verify PWM-capable, see README)
#define MOTOR_IN1   6        // L298N IN1 -> direction
#define MOTOR_IN2   5        // L298N IN2 -> direction

// ---------------------------------------------------------------------------
// Steering geometry (measured on YOUR car)
//   * CENTER is the ONLY fixed steering angle (you measured 79).
//   * servo angle > CENTER  -> steers RIGHT
//   * servo angle < CENTER  -> steers LEFT
//   There are NO hard-coded left/right angles. The PD loop computes the steering
//   angle continuously (the "best" angle for that instant), clamped only by the
//   mechanical travel below: a 90 deg corner saturates to full travel, a tiny
//   drift uses a tiny angle -- same loop, nothing fixed but the center.
// ---------------------------------------------------------------------------
#define CENTER       79      // servo angle for "straight ahead" (measured)
#define STEER_TRAVEL 35      // max mechanical deflection from CENTER (deg). SET THIS to
                             // your car's real steering limit -> clamp [CENTER-T, CENTER+T].
                             // Bigger travel = sharper possible corner (find the max that
                             // doesn't bind the linkage).

// ---------------------------------------------------------------------------
// PD heading-hold gains   steer = CENTER - SIGN*(KP*error + KD*yawRate)
//   error  = targetHeading - heading      (deg)
//   yawRate = current turn rate           (deg/s)  -> damping term
//   Flip CORRECTION_SIGN to -1 if the car steers the WRONG way (see tuning).
// ---------------------------------------------------------------------------
#define KP              1.2f
#define KD              0.10f
#define CORRECTION_SIGN (+1)

// ---------------------------------------------------------------------------
// Control-loop timing / drive
// ---------------------------------------------------------------------------
#define LOOP_MS     20       // 20 ms -> 50 Hz control update
#define MOTOR_SPEED 140      // L298N PWM duty 0..255 for the run (lower = gentler)
#define TARGET_TOL  5.0f     // deg: |targetHeading - heading| under this = "at target" (parking)

// ---------------------------------------------------------------------------
// MPU-6050 / GY-521 registers
// ---------------------------------------------------------------------------
#define MPU_ADDR        0x68 // AD0 low (GY-521 default)
#define REG_PWR_MGMT_1  0x6B // write 0x01 -> wake + PLL/X-gyro clock (stable)
#define REG_GYRO_CONFIG 0x1B // FS_SEL in bits [4:3]
#define REG_GYRO_ZOUT_H 0x47 // gyro Z high byte (low byte 0x48 auto-follows)
#define REG_WHO_AM_I    0x75 // reads 0x68 on a genuine MPU-6050

#define GYRO_FS_CONFIG  0x00    // 0x00 = +/-250 deg/s (finest resolution)
#define GYRO_SENS       131.0f  // LSB per deg/s at +/-250 deg/s

#define CAL_SAMPLES     1000    // stationary samples averaged for the bias
#define RATE_DEADBAND   0.5f    // ignore |rate| below this (deg/s) -> kills noise creep

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Servo servo;
int   currentAngle  = CENTER;   // last servo angle written
int   driveDir      = +1;       // +1 = forward, -1 = reverse (reverse used only for parking)

// --- Independent subsystem switches (motor / steering are decoupled) ---------
//   motorEnabled : L298N drive motor running?
//   holdEnabled  : gyro heading-hold steering active (the PD loop writes servo)?
// They are separate so you can drive without auto-steer, or steer on the bench
// without driving. set_straight()/stop() flip both at once for convenience.
bool  motorEnabled  = false;
bool  holdEnabled   = false;

float gyroZbias     = 0.0f;     // raw-count bias subtracted from every read
float heading       = 0.0f;     // integrated yaw, deg (0 = direction at run start)
float targetHeading = 0.0f;     // desired heading; turn() bumps it by +/-90
float yawRate       = 0.0f;     // last yaw rate, deg/s (also the D term)
int   lastSteer     = CENTER;   // last steer command written by the hold loop
int   turnCount     = 0;        // corners executed via turn() (handy for WRO laps)

// --- Camera (Linux/Python) inputs: the vision side sets these over the bridge ---
int   steerBias     = 0;        // deg added to the steer command (+ = right, - = left)
                                // camera uses it to hug a side past a red/green pillar
                                // or to veer off an approaching wall. 0 = no bias.
int   speedPct      = 100;      // 0..100 scaling of MOTOR_SPEED. Camera drops this as a
                                // wall/pillar gets close (proximity) -> 0 = halt but keep
                                // the gyro tracking. Lets corners be taken slower = safer.

unsigned long lastLoopUs = 0;   // for measured dt
unsigned long lastTickMs = 0;   // for LOOP_MS scheduling

// ===========================================================================
// I2C helpers (bare Wire -> Zephyr-portable, no Adafruit dependency)
// ===========================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  IMU_BUS.beginTransmission(MPU_ADDR);
  IMU_BUS.write(reg);
  IMU_BUS.write(val);
  IMU_BUS.endTransmission(true);
}

// Read raw signed gyro-Z. Returns false on an I2C error so a failed read never
// feeds 0xFF garbage into the bias average or the heading integral.
bool mpuReadGyroZ(int16_t &out) {
  IMU_BUS.beginTransmission(MPU_ADDR);
  IMU_BUS.write(REG_GYRO_ZOUT_H);
  if (IMU_BUS.endTransmission(false) != 0) return false;   // repeated start
  if (IMU_BUS.requestFrom(MPU_ADDR, 2) != 2) return false;
  uint8_t hi = IMU_BUS.read();
  uint8_t lo = IMU_BUS.read();
  out = (int16_t)((hi << 8) | lo);   // int16_t keeps the two's-complement sign
  return true;
}

void mpuInit() {
  IMU_BUS.begin();

  // Presence/sanity check: WHO_AM_I should read 0x68.
  uint8_t who = 0xFF;
  IMU_BUS.beginTransmission(MPU_ADDR);
  IMU_BUS.write(REG_WHO_AM_I);
  if (IMU_BUS.endTransmission(false) == 0 && IMU_BUS.requestFrom(MPU_ADDR, 1) == 1) {
    who = IMU_BUS.read();
  }
  Monitor.println("MPU WHO_AM_I = 0x" + String(who, HEX) +
                  (who == 0x68 ? "  (ok)"
                               : "  (NOT 0x68 -> update core >=0.55.2, or set IMU_BUS to Wire1/Wire2)"));

  mpuWrite(REG_PWR_MGMT_1, 0x01);            // wake + PLL/X-gyro clock
  delay(50);
  mpuWrite(REG_GYRO_CONFIG, GYRO_FS_CONFIG); // +/-250 deg/s
  delay(10);
}

// Average CAL_SAMPLES stationary readings -> gyroZbias (raw counts).
// Robot MUST be perfectly still. delay() here is fine (setup only).
void calibrateGyro() {
  Monitor.println("calibrating gyro bias - hold the car STILL...");
  delay(300);                                  // discard wake/settle transient
  double sum = 0.0;
  int got = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    int16_t gz;
    if (mpuReadGyroZ(gz)) { sum += gz; got++; }
    delay(2);
  }
  gyroZbias = got ? (float)(sum / got) : 0.0f;
  Monitor.println("gyroZbias = " + String(gyroZbias, 2) + " counts (" +
                  String(gyroZbias / GYRO_SENS, 3) + " deg/s) from " +
                  String(got) + " samples");
}

// ===========================================================================
// Motor (L298N) -- forward / stop
// ===========================================================================
void motorStop() {
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// Drive forward at MOTOR_SPEED scaled by speedPct (camera proximity throttle).
// Called whenever motorEnabled or speedPct changes, so the camera can slow/halt live.
void applyDrive() {
  if (!motorEnabled || speedPct <= 0) { motorStop(); return; }
  int spd = (int)((long)MOTOR_SPEED * speedPct / 100);
  if (driveDir >= 0) { digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);  }  // forward
  else               { digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); }  // reverse
  analogWrite(MOTOR_ENA, spd);
}

// Write a servo angle, clamped to the mechanical travel band around CENTER.
void writeSteer(int angle) {
  int lo = CENTER - STEER_TRAVEL, hi = CENTER + STEER_TRAVEL;
  if (angle < lo) angle = lo;
  if (angle > hi) angle = hi;
  currentAngle = angle;
  lastSteer    = angle;
  servo.write(angle);
}

// ===========================================================================
// RPC handlers (provide_safe -> run in loop() context, hardware-safe)
// ===========================================================================

// Manual control (kept). Uses full 0..180 and turns OFF auto so they never fight.
int set_angle(int angle) {
  holdEnabled  = false;     // manual slider -> drop auto steering...
  motorEnabled = false;     // ...and the motor, so nothing fights the slider
  motorStop();
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
  currentAngle = angle;
  servo.write(angle);
  Monitor.println("set_angle -> " + String(currentAngle));
  return currentAngle;
}
int get_angle() { return currentAngle; }

// Start a run: current direction becomes "straight" (heading & target = 0),
// center the servo, motor forward, enable the hold loop.
int set_straight() {
  heading       = 0.0f;
  targetHeading = 0.0f;
  yawRate       = 0.0f;
  turnCount     = 0;
  steerBias     = 0;
  speedPct      = 100;
  lastLoopUs    = micros();
  lastTickMs    = millis();
  writeSteer(CENTER);
  holdEnabled   = true;     // enable BOTH steering...
  motorEnabled  = true;     // ...and motor (convenience "go")
  applyDrive();
  Monitor.println("set_straight -> heading zeroed, motor + hold ENABLED");
  return 1;
}

// --- Independent toggles: motor and steering can be switched on their own -----

// Enable/disable just the L298N drive motor.
int set_motor(int on) {
  motorEnabled = on ? true : false;
  applyDrive();
  Monitor.println(String("set_motor -> ") + (motorEnabled ? "ON" : "OFF"));
  return motorEnabled ? 1 : 0;
}

// Enable/disable just the gyro heading-hold steering. Turning ON resets the loop
// timing (so the first dt isn't huge); turning OFF straightens to CENTER.
int set_steer(int on) {
  bool en = on ? true : false;
  if (en && !holdEnabled) { lastLoopUs = micros(); lastTickMs = millis(); }
  holdEnabled = en;
  if (!holdEnabled) writeSteer(CENTER);
  Monitor.println(String("set_steer -> ") + (holdEnabled ? "ON" : "OFF"));
  return holdEnabled ? 1 : 0;
}

// Camera proximity throttle: 0..100 % of MOTOR_SPEED. Drop toward 0 as a wall/
// pillar approaches; 0 halts but keeps the gyro tracking. Returns applied %.
int set_speed(int pct) {
  if (pct < 0)   pct = 0;
  if (pct > 100) pct = 100;
  speedPct = pct;
  applyDrive();
  return speedPct;
}

// Camera lateral bias: degrees added to the steer command (+ right / - left).
// Used to hug a side past a red/green pillar or to veer off an approaching wall.
// Clamped so a bias alone can't exceed the mechanical travel. Returns applied bias.
int nudge(int bias) {
  if (bias >  STEER_TRAVEL) bias =  STEER_TRAVEL;
  if (bias < -STEER_TRAVEL) bias = -STEER_TRAVEL;
  steerBias = bias;
  return steerBias;
}

// Execute a corner: shift the target heading by 'deg' (e.g. +90 / -90). The PD
// loop steers hard until the new heading is reached, then resumes straight on it.
// Sign of 'deg' must match your gyro's +Z sense; flip if turns go the wrong way.
int turn(int deg) {
  targetHeading += deg;
  turnCount++;
  Monitor.println("turn(" + String(deg) + ") -> target=" + String(targetHeading, 0) +
                  "  count=" + String(turnCount));
  return turnCount;
}

// Stop: disable motor + hold, recenter steering, clear camera inputs.
int stop_run() {
  motorEnabled = false;
  holdEnabled  = false;
  steerBias    = 0;
  speedPct     = 100;
  motorStop();
  writeSteer(CENTER);
  Monitor.println("stop -> motor off, servo recentered, hold DISABLED");
  return 0;
}

// Readouts for the UI / camera side (ints keep the bridge simple).
int get_heading() { return (int)lroundf(heading);       }   // deg
int get_steer()   { return lastSteer;                   }   // last servo angle
int get_target()  { return (int)lroundf(targetHeading); }   // deg
int get_motor()   { return motorEnabled ? 1 : 0;        }   // motor switch state
int get_hold()    { return holdEnabled  ? 1 : 0;        }   // steering switch state
int get_turns()   { return turnCount;                   }   // corners executed (lap counting)

// Drive direction for the parking maneuver: +1 forward, -1 reverse. Reverse swaps
// IN1/IN2 in applyDrive(); the gyro PD loop keeps steering to targetHeading either way.
int set_drive_dir(int dir) {
  driveDir = (dir < 0) ? -1 : +1;
  applyDrive();
  Monitor.println(String("set_drive_dir -> ") + (driveDir < 0 ? "REVERSE" : "FORWARD"));
  return driveDir;
}

// Parking helper: 1 when the heading has reached its target (within TARGET_TOL), else 0.
// The Python park sequence bumps targetHeading via turn() then polls this between segments.
int at_target() { return (fabsf(targetHeading - heading) < TARGET_TOL) ? 1 : 0; }

// ===========================================================================
// setup / loop
// ===========================================================================
void setup() {
  Bridge.begin();
  Monitor.begin(115200);

  servo.attach(SERVO_PIN);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorStop();

  // Visible steering self-test + center.
  servo.write(CENTER - 20); delay(400);
  servo.write(CENTER + 20); delay(400);
  servo.write(CENTER);      delay(400);
  currentAngle = CENTER;
  lastSteer    = CENTER;

  mpuInit();
  calibrateGyro();          // <-- keep the car STILL through this

  lastLoopUs = micros();
  lastTickMs = millis();

  Bridge.provide_safe("set_angle",    set_angle);    // manual (existing)
  Bridge.provide_safe("get_angle",    get_angle);    // manual (existing)
  Bridge.provide_safe("set_straight", set_straight); // convenience: motor + hold on
  Bridge.provide_safe("set_motor",    set_motor);    // toggle drive motor alone
  Bridge.provide_safe("set_steer",    set_steer);    // toggle steering hold alone
  Bridge.provide_safe("turn",         turn);
  Bridge.provide_safe("set_speed",    set_speed);    // camera proximity throttle
  Bridge.provide_safe("nudge",        nudge);        // camera lateral bias
  Bridge.provide_safe("stop",         stop_run);
  Bridge.provide_safe("get_heading",  get_heading);
  Bridge.provide_safe("get_steer",    get_steer);
  Bridge.provide_safe("get_target",   get_target);
  Bridge.provide_safe("get_motor",    get_motor);
  Bridge.provide_safe("get_hold",     get_hold);
  Bridge.provide_safe("get_turns",    get_turns);      // lap counting (Python FSM)
  Bridge.provide_safe("set_drive_dir", set_drive_dir); // forward/reverse (parking)
  Bridge.provide_safe("at_target",    at_target);      // heading-reached poll (parking)
  Monitor.println("bridge handlers registered; ready");
}

void loop() {
  // RouterBridge dispatches provide_safe() callbacks here. The hold loop runs on
  // a fixed schedule and is fully non-blocking (no delay()), so RPCs stay live.
  unsigned long now = millis();
  if (holdEnabled && (now - lastTickMs) >= LOOP_MS) {
    lastTickMs = now;

    // Measured dt (s) -- robust to irregular loop dispatch timing.
    unsigned long nowUs = micros();
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    if (dt <= 0.0f || dt > 0.2f) dt = LOOP_MS * 1e-3f;   // glitch guard

    // 1) measure yaw rate (skip integration on a failed read)
    int16_t gz;
    if (mpuReadGyroZ(gz)) {
      float rate = (gz - gyroZbias) / GYRO_SENS;
      if (rate > -RATE_DEADBAND && rate < RATE_DEADBAND) rate = 0.0f;
      yawRate  = rate;
      heading += yawRate * dt;            // 2) integrate to heading
    }

    // 3) PD correction toward the target heading, + camera lateral bias
    float error      = targetHeading - heading;
    float correction = CORRECTION_SIGN * (KP * error + KD * yawRate);
    writeSteer((int)lroundf(CENTER - correction) + steerBias);   // clamped inside
  }
}
