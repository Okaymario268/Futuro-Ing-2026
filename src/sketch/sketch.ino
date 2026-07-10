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
 *   - L298N DC drive motor: ENB=4, IN4=3, IN3=2 (simple forward / stop).
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
// Built-in 13x8 LED matrix (ships with recent arduino:zephyr cores). Guarded with
// __has_include so the sketch STILL COMPILES on cores that don't have the library
// yet -- without it the countdown just doesn't light up (it still logs 3..2..1 and
// starts the run). This keeps a missing library from breaking the WHOLE app.
#if __has_include(<Arduino_LED_Matrix.h>)
  #include <Arduino_LED_Matrix.h>
  #define HAVE_LED_MATRIX 1
#else
  #define HAVE_LED_MATRIX 0
#endif

// Front ranger: HC-SR04 ULTRASONIC (replaced the VL53L0X laser -- no library
// needed, and sound doesn't care that the WRO walls are matte black, which
// absorbed the laser's 940 nm beam). Driven interrupt-based + non-blocking
// below; pins/thresholds in the FRONT ULTRASONIC section.

// ---------------------------------------------------------------------------
// I2C bus instance for the IMU  (change in ONE place if WHO_AM_I != 0x68)
// ---------------------------------------------------------------------------
#define IMU_BUS  Wire        // try Wire1, then Wire2, if Wire doesn't see 0x68

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------
#define SERVO_PIN   9        // steering servo signal (PWM-capable on UNO Q)
#define MOTOR_ENB   4        // L298N ENB -> PWM speed   (MUST be PWM-capable! pin 4 is
                             // NOT a classic-Uno PWM pin -- if speed control doesn't
                             // work on the UNO Q, move ENB to a known PWM pin, e.g. 3/9)
#define MOTOR_IN4   3        // L298N IN4 -> direction
#define MOTOR_IN3   2        // L298N IN3 -> direction

// GY-521 (MPU-6050) gyro -- I2C, so it has NO settable pin number like the ones
// above. SDA/SCL are a shared bus FIXED by the Wire peripheral (IMU_BUS, above):
// on the UNO Q they are the A4/A5 header pins. These two defines are a WIRING
// REFERENCE only -- changing them does NOT re-route I2C (that's a hardware fact);
// to move the gyro to another bus, change IMU_BUS (Wire/Wire1/Wire2), not these.
#define GYRO_SDA_PIN A4      // wire GY-521 SDA -> A4   (fixed by IMU_BUS; reference)
#define GYRO_SCL_PIN A5      // wire GY-521 SCL -> A5   (fixed by IMU_BUS; reference)
//      GY-521 VCC -> 3.3V   GND -> GND   AD0 -> GND (I2C address 0x68, see MPU_ADDR)

// --- START BUTTON (WRO single-start-button) --------------------------------
// Wire a momentary push button:  one leg -> D7,  the other leg -> GND.
// It uses the internal pull-up (INPUT_PULLUP), so idle reads HIGH and PRESSED
// reads LOW -- no external resistor needed. When enabled, pressing it plays a
// 3-2-1 countdown on the built-in LED matrix, then releases the run.
#define START_BTN_PIN     7      // moved from D2 -> D2 is now MOTOR_IN3 (free: 7)

// --- FRONT ULTRASONIC (HC-SR04) ----------------------------------------------
// Wiring:  VCC -> 5V   GND -> GND   TRIG -> D8   ECHO -> D10 **THROUGH A DIVIDER**
//   !! The HC-SR04 is a 5 V sensor and its ECHO pin outputs 5 V, but the UNO Q
//   GPIOs are 3.3 V. Divide ECHO down: ECHO --[1 kOhm]--+--> D10, + --[2 kOhm]--> GND
//   (5 V * 2k/3k = 3.3 V). TRIG is fine driven directly at 3.3 V (>= 2.4 V = HIGH).
// Mount: front bumper CENTERLINE, ~50 mm high (mid-height of the 100 mm walls),
// aimed LEVEL. The ~15-deg sound cone is wider than the laser dot was -- keep it
// clear of the car's own bodywork/wheels or they echo back as a phantom wall.
//
// SAME JOB THE LASER HAD ("turned at the corner, then stopped going forward"):
// the camera's front band can read ~0 with a wall dead ahead, so the car keeps
// commanding 70% and stalls nose-first. The ranger doesn't care where the wall
// sits in the image: the MCU BLOCKS FORWARD drive under FRONT_HALT_MM (reverse
// stays allowed, so the FSM's un-stick reverse pulse works) and auto-releases
// past FRONT_RELEASE_MM. Measurement is interrupt-based (echo edges timestamped
// in an ISR) so nothing here ever blocks the bridge or the 50 Hz PD loop.
#define US_TRIG_PIN      8     // 10 us trigger pulse out
#define US_ECHO_PIN      10    // echo width in (VIA THE 1k/2k DIVIDER -- see above)
#define FRONT_HALT_MM    75    // block forward drive when a wall is closer (mm)
                               // (halved from 150 -- user: stop 50% closer; still
                               // above the HC-SR04's ~20 mm blind zone)
#define FRONT_RELEASE_MM 100   // un-block once clearance exceeds this (hysteresis)
#define FRONT_CLEAR_MM   2000  // readings >= this = nothing in range (report -1)
#define US_PING_MS       60    // ping period; >= 60 ms per datasheet so the previous
                               // burst's residual echoes can't bleed into the next
#define USE_START_BUTTON  1      // 1 = arm the button+countdown gate at boot;
                                 // 0 = no gate, runs start immediately (old behaviour).
                                 // Also switchable at runtime via set_start_button().
#define COUNTDOWN_MS      1000   // ms shown per number (3, 2, 1)

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
//   Flip CORRECTION_SIGN if the car steers the WRONG way (see tuning). NOTE:
//   GYRO_SIGN and CORRECTION_SIGN flip TOGETHER -- inverting the heading sense
//   alone turns the loop into positive feedback (it steers INTO the error).
// ---------------------------------------------------------------------------
#define KP              1.2f
#define KD              0.10f
#define CORRECTION_SIGN (-1)

// ---------------------------------------------------------------------------
// Control-loop timing / drive
// ---------------------------------------------------------------------------
#define LOOP_MS     20       // 20 ms -> 50 Hz control update
#define MOTOR_SPEED 255      // L298N PWM duty 0..255 at 100% speed. Full duty: the old
                             // 140 cap meant "100%" was really 55% duty, so vision's
                             // 60-70% commands landed near the L298N stall region and
                             // the motor died whenever the camera was ON. Set 200 for
                             // a gentler top end if full send is too fast.
#define MOTOR_MIN_PWM 70     // duty at 1% speed: the lowest duty that reliably TURNS your
                             // drivetrain (an L298N drops ~2V; low duty stalls, not crawls).
                             // Find it by lowering until the wheels stall, then add margin.
#define TARGET_TOL  5.0f     // deg: |targetHeading - heading| under this = "at target" (parking)
#define WATCHDOG_MS 1500     // motor auto-stop if Python stops pinging (crash guard).
                             // Armed by the FIRST "ping" RPC, so bench use without a
                             // pinger behaves exactly as before.

// ---------------------------------------------------------------------------
// MPU-6050 / GY-521 registers
// ---------------------------------------------------------------------------
#define MPU_ADDR        0x68 // AD0 low (GY-521 default)
#define REG_PWR_MGMT_1  0x6B // write 0x01 -> wake + PLL/X-gyro clock (stable)
#define REG_GYRO_CONFIG 0x1B // FS_SEL in bits [4:3]
#define REG_GYRO_ZOUT_H 0x47 // gyro Z high byte (low byte 0x48 auto-follows)
#define REG_WHO_AM_I    0x75 // reads 0x68 on a genuine MPU-6050

#define GYRO_FS_CONFIG  0x08    // 0x08 = +/-500 deg/s: headroom so a snappy 90-deg
                                // corner can't clip the rate (a clipped rate reads
                                // low -> heading undercounts -> turns never "arrive")
#define GYRO_SENS       65.5f   // LSB per deg/s at +/-500 deg/s

// Sign applied to the measured yaw rate so the heading follows the CODE-WIDE
// convention: turn RIGHT (CW) = heading INCREASES (+90 = right corner, matches
// vision's +right bias and ORANGE_TURN=+90). The MPU-6050 Z axis points UP, so
// by the right-hand rule a RIGHT turn reads NEGATIVE raw -> -1 flips it.
// Set +1 only if your module is mounted upside-down (and then also flip
// CORRECTION_SIGN -- the two always flip together, see the PD note above).
#define GYRO_SIGN       (-1)

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
unsigned long lastPingMs = 0;   // watchdog: last "ping" RPC (0 = not armed yet)

// --- Front ultrasonic state ---------------------------------------------------
// The ECHO ISR only timestamps edges; loop() converts to mm once per ping cycle.
volatile unsigned long usEchoRiseUs = 0;     // micros() at the echo rising edge
volatile unsigned long usEchoWidthUs = 0;    // last completed echo width (us)
volatile bool          usEchoDone   = false; // a full echo arrived since the last ping
bool frontBlocked = false;      // forward drive currently blocked (wall too close)
int  frontMM      = -1;         // last front distance in mm; -1 = no reading/echo lost

// --- Start-button gate + LED-matrix countdown --------------------------------
#if HAVE_LED_MATRIX
Arduino_LED_Matrix matrix;
#endif
bool startEnabled = USE_START_BUTTON;    // runtime toggle (default from the #define)
enum StartState { SP_WAIT, SP_COUNT, SP_FIRED };
StartState    startState  = SP_FIRED;    // SP_FIRED = "ready/ungated" until armed
unsigned long startT0     = 0;           // countdown start time
int           shownDigit  = -1;          // last digit drawn (redraw only on change)

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

// HC-SR04 echo ISR: timestamp the rising edge, compute the width on the falling
// edge. Kept tiny (two micros() reads) -- everything else happens in loop().
void usEchoISR() {
  if (digitalRead(US_ECHO_PIN)) {
    usEchoRiseUs = micros();
  } else {
    usEchoWidthUs = micros() - usEchoRiseUs;
    usEchoDone    = true;
  }
}

// Bring up the HC-SR04 front ultrasonic. No probe/handshake exists for this
// sensor: if it's absent or miswired, no echo ever arrives and frontMM simply
// stays -1 = the same camera-only fallback the laser had. If attachInterrupt
// is unavailable on this pin (beta Zephyr core), behaviour degrades the same
// way -- frontMM -1, never a false block.
void ultrasonicInit() {
  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN), usEchoISR, CHANGE);
  Monitor.println("HC-SR04 front ultrasonic: TRIG=D" + String(US_TRIG_PIN) +
                  " ECHO=D" + String(US_ECHO_PIN) + " (via 1k/2k divider), ping every " +
                  String(US_PING_MS) + " ms");
}

// ===========================================================================
// Motor (L298N) -- forward / stop
// ===========================================================================
void motorStop() {
  analogWrite(MOTOR_ENB, 0);
  digitalWrite(MOTOR_IN4, LOW);
  digitalWrite(MOTOR_IN3, LOW);
}

// Drive at speedPct scaled onto [MOTOR_MIN_PWM..MOTOR_SPEED] (camera proximity
// throttle). Mapping to a floor instead of 0 matters: 35% of 140 would be duty
// 49/255, which stalls a geared motor through an L298N instead of crawling.
// Called whenever motorEnabled or speedPct changes, so the camera can slow/halt live.
void applyDrive() {
  if (!motorEnabled || speedPct <= 0) { motorStop(); return; }
  if (frontBlocked && driveDir >= 0) { motorStop(); return; }   // ultrasonic: wall ahead ->
                                                                // forward blocked; REVERSE
                                                                // still allowed (un-stick)
  int spd = MOTOR_MIN_PWM + (int)((long)(MOTOR_SPEED - MOTOR_MIN_PWM) * speedPct / 100);
  if (driveDir >= 0) { digitalWrite(MOTOR_IN4, HIGH); digitalWrite(MOTOR_IN3, LOW);  }  // forward
  else               { digitalWrite(MOTOR_IN4, LOW);  digitalWrite(MOTOR_IN3, HIGH); }  // reverse
  analogWrite(MOTOR_ENB, spd);
}

// Write a servo angle, clamped to the mechanical travel band around CENTER.
// Skips the servo.write() when the angle is unchanged (less servo chatter at 50 Hz).
void writeSteer(int angle) {
  int lo = CENTER - STEER_TRAVEL, hi = CENTER + STEER_TRAVEL;
  if (angle < lo) angle = lo;
  if (angle > hi) angle = hi;
  if (angle != currentAngle) servo.write(angle);
  currentAngle = angle;
  lastSteer    = angle;
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

// Manual bench-jog steering for the Web UI:  d = -1 left, 0 center, +1 right.
// Disables the gyro hold so it won't fight you, but LEAVES THE MOTOR as-is (unlike
// set_angle, which stops it) -- so you can drive forward and steer at the same time.
// Uses CENTER +/- STEER_TRAVEL, so Left/Right always match the sketch's geometry.
int steer_dir(int d) {
  holdEnabled = false;
  int a = CENTER + ((d < 0) ? -STEER_TRAVEL : (d > 0) ? STEER_TRAVEL : 0);
  writeSteer(a);
  Monitor.println("steer_dir -> " + String(a));
  return currentAngle;
}

// Execute a corner: shift the target heading by 'deg' (e.g. +90 / -90). The PD
// loop steers hard until the new heading is reached, then resumes straight on it.
// Convention (normalized by GYRO_SIGN): +deg = RIGHT (CW), -deg = LEFT (CCW).
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
int get_front_mm(){ return frontMM;                     }   // front ultrasonic, mm (-1 = none)
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

// Watchdog heartbeat: Python pings ~2 Hz during a run. If pings stop while the motor
// is on (Python crashed mid-run), loop() stops the motor instead of letting the car
// drive away into a wall. First ping arms it; never pinging = old behaviour.
int ping() {
  lastPingMs = millis();
  return 1;
}

// ===========================================================================
// Start button + LED-matrix 3-2-1 countdown
// ---------------------------------------------------------------------------
// Non-blocking gate for the autonomous start. Flow: arm_start() -> waits for a
// press on START_BTN_PIN -> plays 3,2,1 on the built-in matrix -> latches FIRED.
// Python (or the Web UI) polls start_ready() and only begins the run once it's 1.
// When disabled (startEnabled=false) the gate is transparent: start_ready()=1.
// ===========================================================================

// 8 rows x 13 cols, 1 = lit. Digits are 5 wide, drawn at column offset 4 (centred).
static const uint8_t DIGIT_3[8][13] = {
  {0,0,0,0, 1,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 0,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 1,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,0, 0,0,0,0},
};
static const uint8_t DIGIT_2[8][13] = {
  {0,0,0,0, 1,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,1, 0,0,0,0},
  {0,0,0,0, 1,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 1,0,0,0,0, 0,0,0,0},
  {0,0,0,0, 1,0,0,0,0, 0,0,0,0},
  {0,0,0,0, 1,1,1,1,1, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,0, 0,0,0,0},
};
static const uint8_t DIGIT_1[8][13] = {
  {0,0,0,0, 0,0,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,1,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,0,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,0,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,0,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,0,1,0,0, 0,0,0,0},
  {0,0,0,0, 0,1,1,1,0, 0,0,0,0},
  {0,0,0,0, 0,0,0,0,0, 0,0,0,0},
};

#if HAVE_LED_MATRIX
void showPattern(const uint8_t p[8][13]) {
  uint8_t buf[8 * 13];
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 13; c++)
      buf[r * 13 + c] = p[r][c] ? 255 : 0;   // full brightness for lit pixels
  matrix.draw(buf);
}
void clearMatrix() { matrix.clear(); }
#else
void showPattern(const uint8_t p[8][13]) { (void)p; }   // no matrix on this core -> no-op
void clearMatrix() {}
#endif

// Runs every loop() iteration; fully non-blocking (millis-based). Advances the
// WAIT -> COUNT(3,2,1) -> FIRED state machine and drives the matrix.
void serviceStart() {
  static unsigned long btnLowSince = 0;

  if (startState == SP_WAIT) {
    if (digitalRead(START_BTN_PIN) == LOW) {           // active-low, debounced
      if (btnLowSince == 0) btnLowSince = millis();
      else if (millis() - btnLowSince > 30) {
        startState = SP_COUNT;
        startT0    = millis();
        shownDigit = -1;
        Monitor.println("start button pressed -> 3..2..1");
      }
    } else {
      btnLowSince = 0;
    }
  } else if (startState == SP_COUNT) {
    unsigned long el = millis() - startT0;
    int digit = (el < COUNTDOWN_MS) ? 3 : (el < 2 * COUNTDOWN_MS) ? 2
              : (el < 3 * COUNTDOWN_MS) ? 1 : 0;
    if (digit != shownDigit) {
      shownDigit = digit;
      if      (digit == 3) showPattern(DIGIT_3);
      else if (digit == 2) showPattern(DIGIT_2);
      else if (digit == 1) showPattern(DIGIT_1);
      else { clearMatrix(); startState = SP_FIRED; Monitor.println("countdown done -> GO"); }
    }
  }
}

// Arm (or, if disabled, immediately release) the start gate. Called at boot and
// whenever the run stops, so the button is ready for the next start.
int arm_start() {
  clearMatrix();
  if (startEnabled) {
    startState = SP_WAIT;
    shownDigit = -1;
    Monitor.println("start armed -> waiting for button");
  } else {
    startState = SP_FIRED;              // transparent gate
  }
  return (startState == SP_FIRED) ? 1 : 0;
}

// Toggle the whole button+countdown behavior on/off, then re-arm with the setting.
int set_start_button(int on) {
  startEnabled = on ? true : false;
  Monitor.println(String("set_start_button -> ") + (startEnabled ? "ON" : "OFF"));
  return arm_start();
}

// Python/Web UI poll this: 1 once the countdown has finished (or gate disabled).
int start_ready() { return (startState == SP_FIRED) ? 1 : 0; }

// ===========================================================================
// setup / loop
// ===========================================================================
void setup() {
  Bridge.begin();
  Monitor.begin(115200);

  servo.attach(SERVO_PIN);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  motorStop();

#if HAVE_LED_MATRIX
  matrix.begin();
  matrix.setGrayscaleBits(8);          // draw() takes 0..255 brightness per pixel
  matrix.clear();
#endif
  pinMode(START_BTN_PIN, INPUT_PULLUP);  // button: other leg to GND, pressed = LOW

  // Visible steering self-test + center.
  servo.write(CENTER - 20); delay(400);
  servo.write(CENTER + 20); delay(400);
  servo.write(CENTER);      delay(400);
  currentAngle = CENTER;
  lastSteer    = CENTER;

  mpuInit();
  calibrateGyro();          // <-- keep the car STILL through this
  ultrasonicInit();         // front HC-SR04 (TRIG/ECHO pins; frontMM -1 if absent)

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
  Bridge.provide_safe("steer_dir",    steer_dir);    // manual jog steer (keeps motor)
  Bridge.provide_safe("stop",         stop_run);
  Bridge.provide_safe("get_heading",  get_heading);
  Bridge.provide_safe("get_steer",    get_steer);
  Bridge.provide_safe("get_target",   get_target);
  Bridge.provide_safe("get_motor",    get_motor);
  Bridge.provide_safe("get_hold",     get_hold);
  Bridge.provide_safe("get_turns",    get_turns);      // lap counting (Python FSM)
  Bridge.provide_safe("get_front_mm", get_front_mm);   // front ultrasonic distance (mm)
  Bridge.provide_safe("set_drive_dir", set_drive_dir); // forward/reverse (parking)
  Bridge.provide_safe("at_target",    at_target);      // heading-reached poll (parking)
  Bridge.provide_safe("ping",         ping);           // watchdog heartbeat (crash guard)
  Bridge.provide_safe("set_start_button", set_start_button); // toggle button+countdown gate
  Bridge.provide_safe("arm_start",        arm_start);        // re-arm the gate
  Bridge.provide_safe("start_ready",      start_ready);      // poll: countdown finished?

  startEnabled = USE_START_BUTTON;
  arm_start();                                        // reflect the gate on the matrix at boot
  Monitor.println("bridge handlers registered; ready");
}

void loop() {
  // RouterBridge dispatches provide_safe() callbacks here. The hold loop runs on
  // a fixed schedule and is fully non-blocking (no delay()), so RPCs stay live.
  serviceStart();               // start-button gate + LED countdown (non-blocking)

  unsigned long now = millis();

  // Watchdog: armed once Python has pinged; trips if pings stop while driving.
  if (lastPingMs != 0 && motorEnabled && (now - lastPingMs) > WATCHDOG_MS) {
    lastPingMs   = 0;            // disarm until the next ping (no re-trip spam)
    motorEnabled = false;
    motorStop();
    Monitor.println("WATCHDOG: no ping from Python -> motor stopped");
  }

  // Front-ultrasonic service: once per US_PING_MS, harvest the echo the ISR
  // captured from the PREVIOUS ping, then fire the next one. Fully non-blocking
  // (the only "wait" is the 10 us trigger pulse), so the bridge and the PD loop
  // never stall.
  static unsigned long lastPingMs2 = 0;
  if ((now - lastPingMs2) >= US_PING_MS) {
    lastPingMs2 = now;

    // 1) harvest: echo width (us) -> distance (mm). Speed of sound 343 m/s,
    //    out and back -> mm = us * 0.343 / 2 = us * 0.1715. No echo since the
    //    last ping = nothing in range (or sensor absent) -> -1, same as before.
    if (usEchoDone) {
      usEchoDone = false;
      long mm = (long)(usEchoWidthUs * 0.1715f);
      frontMM = (mm <= 0 || mm >= FRONT_CLEAR_MM) ? -1 : (int)mm;
    } else {
      frontMM = -1;
    }

    // 2) hysteresis: block forward when a wall is close, release once clearly past.
    if (!frontBlocked && frontMM >= 0 && frontMM < FRONT_HALT_MM) {
      frontBlocked = true;
      applyDrive();
      Monitor.println("US: wall at " + String(frontMM) + " mm -> forward BLOCKED");
    } else if (frontBlocked && (frontMM < 0 || frontMM > FRONT_RELEASE_MM)) {
      frontBlocked = false;
      applyDrive();
      Monitor.println("US: clear (" + String(frontMM) + " mm) -> forward released");
    }

    // 3) next ping -- but only if the echo line is idle: some HC-SR04 clones
    //    hold ECHO high ~200 ms after a lost echo, and triggering into that
    //    would corrupt the ISR's edge pairing.
    if (digitalRead(US_ECHO_PIN) == LOW) {
      digitalWrite(US_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(US_TRIG_PIN, LOW);
    }
  }

  // The gyro tick runs ALWAYS (not only while the hold is enabled) so the Web UI
  // orientation compass stays live even when idle -- rotate the bot by hand and
  // watch it move. Only the SERVO WRITE below is gated on holdEnabled.
  if ((now - lastTickMs) >= LOOP_MS) {
    lastTickMs = now;

    // Measured dt (s) -- robust to irregular loop dispatch timing.
    unsigned long nowUs = micros();
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    if (dt <= 0.0f || dt > 0.2f) dt = LOOP_MS * 1e-3f;   // glitch guard

    // 1) measure yaw rate (skip integration on a failed read)
    int16_t gz;
    if (mpuReadGyroZ(gz)) {
      float rate = GYRO_SIGN * (gz - gyroZbias) / GYRO_SENS;   // + = turning RIGHT
      if (rate > -RATE_DEADBAND && rate < RATE_DEADBAND) rate = 0.0f;
      yawRate  = rate;
      heading += yawRate * dt;            // 2) integrate to heading
    }

    // 3) PD correction toward the target heading, + camera lateral bias.
    //    driveDir flips the correction in REVERSE: the yaw response to steering
    //    inverts when backing up (yaw_rate ~ v*tan(steer), v < 0), so without the
    //    flip the loop becomes POSITIVE feedback while reverse-parking and diverges.
    if (holdEnabled) {
      float error      = targetHeading - heading;
      float correction = CORRECTION_SIGN * driveDir * (KP * error + KD * yawRate);
      writeSteer((int)lroundf(CENTER - correction) + steerBias);   // clamped inside
    }
  }
}
