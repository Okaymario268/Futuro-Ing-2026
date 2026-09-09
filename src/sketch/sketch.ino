// SPDX-License-Identifier: MPL-2.0
// prueba-hardware: panel de diagnostico + primera version del Open
// Challenge (WRO 2026 Future Engineers). Boton A arranca la carrera,
// boton C para en cualquier momento (parada de emergencia / reset).

#include <Arduino_Modulino.h>
#include <Servo.h>
#include <Arduino_RouterBridge.h>
#include <Wire.h>

const uint8_t PIN_SERVO = 5;      // cambiado de 3 a 5 para probar si el pin 3 quedo danado
const int SERVO_CENTER = 77;  // calibrado: 90 hacia -39 en el slider quedaba recto
const int SERVO_RANGE = 35;
// Si el auto gira al lado contrario del que pide el codigo, cambiar esto
// a -1 (probar primero con el robot en el aire, ruedas sin tocar piso).
const int STEER_SIGN = -1;  // invertido: el auto giraba hacia la pared en vez de alejarse (confirmado en pista 2026-09-08)
// Maximo real confirmado en pista (simetrico izq/der) -- pasado de esto el
// varillaje se traba contra su tope mecanico, no hace falta ni conviene
// pedirle 100%.
const int TURN_STEER_PCT = 80;

ModulinoHub hub;
ModulinoButtons buttons;
// izq/der intercambiados fisicamente tras el rediseno de chasis 2026-09-02
ModulinoDistance distIzq(hub.port(2));
ModulinoDistance distCentro(hub.port(7));
ModulinoDistance distDer(hub.port(3));
ModulinoMotors motors;
Servo servoDireccion;

bool prevA = false, prevB = false, prevC = false;
unsigned long lastReport = 0;

// --- MPU-6050 (I2C crudo por Wire1, soldado a los pads SDA/SCL de paso
// del Modulino Hub -- comparte el MISMO bus que usa Modulino internamente
// en la UNO Q, evita el conflicto Wire/Wire1 documentado antes) ---
// WHO_AM_I en 0x68, despertar (sale de sleep), rango +-250 grados/s, gyro Z
// integrado a heading. La direccion 0x68 no choca con el Hub (0x70) ni con
// Buttons/Motors/Distance.
const uint8_t MPU_ADDR = 0x68;
const float GYRO_SENS_250 = 131.0f;  // LSB por grado/s en el rango +-250
bool gyroOk = false;
float gyroBiasZ = 0;     // grados/s, promedio en quieto al arrancar
float headingDeg = 0;
unsigned long lastGyroMicros = 0;

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

// NOTA: se evita a proposito el patron beginTransmission+endTransmission(false)
// (reinicio repetido) -- en este driver de I2C parecia trabar el bus ENTERO
// (afectaba tambien a Modulino) cuando el MPU-6050 no respondia. En su lugar
// se cierra la transaccion de escritura con STOP normal antes de leer.
bool mpuReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(true) != 0) return false;
  uint8_t got = Wire1.requestFrom((int)MPU_ADDR, (int)n);
  if (got < n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire1.available()) return false;
    buf[i] = Wire1.read();
  }
  return true;
}

bool mpuBegin() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x75);  // WHO_AM_I
  if (Wire1.endTransmission(true) != 0) return false;
  uint8_t got = Wire1.requestFrom((int)MPU_ADDR, 1);
  if (got < 1 || !Wire1.available()) return false;
  uint8_t who = Wire1.read();
  if (who != 0x68) return false;

  if (!mpuWriteReg(0x6B, 0x00)) return false;  // PWR_MGMT_1: salir de sleep
  delay(50);
  if (!mpuWriteReg(0x1B, 0x00)) return false;  // GYRO_CONFIG: +-250 deg/s
  return true;
}

float mpuReadGyroZdps() {
  uint8_t buf[2];
  if (!mpuReadBytes(0x47, buf, 2)) return NAN;  // GYRO_ZOUT_H/L
  int16_t raw = (buf[0] << 8) | buf[1];
  return raw / GYRO_SENS_250;
}

// Calibracion de bias -- el robot DEBE estar quieto durante este medio
// segundo (pasa en setup(), antes de reportar "listo").
void mpuCalibrate() {
  const int N = 200;
  float sum = 0;
  int ok = 0;
  for (int i = 0; i < N; i++) {
    float v = mpuReadGyroZdps();
    if (!isnan(v)) { sum += v; ok++; }
    delay(3);
  }
  gyroBiasZ = ok > 0 ? sum / ok : 0;
}

float normalizeAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// --- Control manual desde el panel web (no la logica de navegacion final) ---
void set_servo_pct(int pct) {
  pct = constrain(pct, -100, 100) * STEER_SIGN;
  servoDireccion.write(SERVO_CENTER + (pct * SERVO_RANGE) / 100);
}

// invert=false -> adelante fisico, invert=true -> reversa fisica
// (motor EV3 nuevo cableado directo al Modulino 2026-09-02 -- polaridad
// quedo al reves del motor anterior, se invirtio la logica)
void test_motor_pct(int pct) {
  if (!(bool)motors) return;
  pct = constrain(pct, -100, 100);
  motors.setInvertA(pct < 0);
  motors.setSpeedA(abs(pct));
}

// ==================== Open Challenge (primera version) ====================
// Centrado por diferencia laser izq/der (PD simple, solo P por ahora),
// esquina detectada por el laser central bajo un umbral, giro a 90 grados
// EXACTOS con el giroscopio (heading absoluto), direccion del giro elegida
// por el lado con mas espacio libre. Boton A arranca, boton C para siempre.
enum RaceState { RACE_IDLE, RACE_DRIVE, RACE_TURN, RACE_STOP };
RaceState raceState = RACE_IDLE;

const int CRUISE_PCT = 55;         // subido de 45 a peticion del usuario
const int TURN_PCT = 55;           // velocidad durante el giro
// El sensor central, montado bajo, ve el piso a corta distancia (60-160mm
// tipico) en vez de la pared real -- disparar por "menos de X" no sirve.
// En su lugar se dispara solo dentro de un RANGO (200-300mm) a peticion del
// usuario.
// El ruido de piso varia segun la superficie: ~150mm en el cuarto de
// pruebas, pero ~47mm sobre el tapete real de la pista -- se deja el
// umbral por debajo del peor caso visto (47mm) para evitar el disparo
// falso, aceptando poco margen de reaccion real.
const float CORNER_FRONT_TRIGGER_MM = 35;
const float TURN_TARGET_DEG = 90.0f;
const float HEADING_TOL_DEG = 8.0f;
// Respaldo de seguridad -- tope maximo razonable para un solo giro dada la
// ronda de 3 min (180s) con 12 esquinas: 10s por giro deja margen de sobra
// para las rectas (12*10s=120s, quedan 60s) sin arriesgar quedar trabado
// para siempre si el gyro nunca cierra el angulo.
const unsigned long TURN_TIMEOUT_MS = 10000;
const unsigned long CORNER_COOLDOWN_MS = 1500; // tras un giro, ignora el centro un rato -- evita reentrar en bucle si no alcanzo a alejarse de la pared
unsigned long cornerCooldownUntil = 0;
const int MAX_CORNERS = 12;        // 3 vueltas x 4 esquinas
const float WALL_KP = 0.15f;       // % de giro por mm de diferencia izq-der
const int WALL_STEER_MAX = 60;     // no saturar el servo a fondo en recta
// El reglamento WRO 2026 FE fija el corredor en 600 o 1000mm (+-100mm en
// la Final) -- ninguna lectura lateral real del pasillo deberia superar
// esto. Si un laser da mas, es ruido/glitch (o ve un hueco/interseccion),
// se descarta para el centrado en vez de mandar un volantazo falso.
const float MAX_PLAUSIBLE_SIDE_MM = 1200.0f;

int cornerCount = 0;
float raceTargetHeading = 0;
bool turnLeft = false;
unsigned long turnStartMs = 0;
bool idleStopSent = false;  // evita repetir motors.setSpeedA(0) en cada loop mientras esta idle/stop
unsigned long lastMotorCmdMs = 0;  // throttle de escritura I2C al motor durante drive/turn (ver mas abajo)
const unsigned long MOTOR_CMD_INTERVAL_MS = 100;  // 10 escrituras/seg alcanza de sobra, evita saturar el bus I2C

// Copia global de las ultimas lecturas laterales -- para que el disparo de
// esquina por camara (Bridge, ver mas abajo) tenga datos frescos con que
// elegir la direccion del giro, igual que el disparo por sensor central.
bool g_niPlausible = false, g_ndPlausible = false;
float g_dIzq = -1, g_dDer = -1;

// Logica compartida para iniciar un giro de esquina, sin importar si lo
// disparo el sensor central o la camara (linea naranja/azul) desde Python.
void startCornerTurn() {
  turnLeft = (g_niPlausible && g_ndPlausible) ? (g_dIzq > g_dDer) : true;  // gira hacia el lado con mas espacio
  raceTargetHeading = normalizeAngle(headingDeg + (turnLeft ? -TURN_TARGET_DEG : TURN_TARGET_DEG));
  turnStartMs = millis();
  cornerCount++;
  raceState = RACE_TURN;
}

void raceStop() {
  raceState = RACE_STOP;
  if ((bool)motors) motors.setSpeedA(0);
  set_servo_pct(0);
  idleStopSent = true;  // ya se mando el stop, no repetir en el loop
}

void raceStart() {
  idleStopSent = false;  // volvera a idle en algun momento, hay que poder re-armar el stop
  cornerCount = 0;
  raceState = RACE_DRIVE;
}
// ============================================================================

void setup() {
  Bridge.begin();

  Modulino.begin();

  bool okBtn = buttons.begin();
  bool okIzq = distIzq.begin();
  bool okCentro = distCentro.begin();
  bool okDer = distDer.begin();
  bool okMotor = motors.begin();
  if (okMotor) {
    motors.setStepperModeEnabled(false);
    motors.setDecay(ModulinoMotors::DecayMode::FAST);  // frenado mas abrupto (SLOW dejaba el auto "planeando" al soltar)
    motors.stop();
  }

  servoDireccion.attach(PIN_SERVO);
  servoDireccion.write(SERVO_CENTER);

  gyroOk = mpuBegin();
  if (gyroOk) {
    mpuCalibrate();  // robot quieto medio segundo aqui
    headingDeg = 0;
    lastGyroMicros = micros();
  }

  Bridge.notify("selftest_buttons", okBtn ? 1 : 0);
  Bridge.notify("selftest_dist_izq", okIzq ? 1 : 0);
  Bridge.notify("selftest_dist_centro", okCentro ? 1 : 0);
  Bridge.notify("selftest_dist_der", okDer ? 1 : 0);
  Bridge.notify("selftest_motor", okMotor ? 1 : 0);
  Bridge.notify("selftest_gyro", gyroOk ? 1 : 0);

  Bridge.provide("set_servo_pct", set_servo_pct);
  Bridge.provide("test_motor_pct", test_motor_pct);
  Bridge.provide("race_stop", [](int unused) { (void)unused; raceStop(); raceState = RACE_IDLE; });
  // Python detecto una linea naranja/azul de esquina por camara -- dispara
  // el mismo giro que el sensor central, solo si estamos manejando y no en
  // cooldown (evita doble disparo si el sensor Y la camara ven la esquina
  // casi al mismo tiempo).
  Bridge.provide("camera_corner", [](int unused) {
    (void)unused;
    if (raceState == RACE_DRIVE && millis() > cornerCooldownUntil) {
      startCornerTurn();
    }
  });

  Bridge.notify("ready", 1);
}

void loop() {
  unsigned long now = millis();

  // Guarda en vivo: solo confiamos en un modulo si sigue respondiendo AHORA
  // (operator bool() de Module), no en lo que dijo begin() al arrancar.
  bool buttonsAlive = (bool)buttons;
  bool motorAlive = (bool)motors;

  // Integra heading cada ciclo (no throttleado -- la precision del
  // integrador depende de leerlo seguido).
  if (gyroOk) {
    unsigned long tnow = micros();
    float dt = (tnow - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = tnow;
    float rate = mpuReadGyroZdps();
    if (!isnan(rate) && dt > 0 && dt < 0.5f) {
      headingDeg += (rate - gyroBiasZ) * dt;
    }
  }

  // Lectura de distancias CADA CICLO (no cada 500ms) -- el control en
  // tiempo real necesita datos frescos; el bloque de reporte de abajo
  // solo manda una foto al panel cada 500ms, ya no vuelve a leer aqui.
  bool ni = distIzq.available();
  bool nc = distCentro.available();
  bool nd = distDer.available();
  float dIzq = ni ? distIzq.get() : -1.0f;
  float dCentro = nc ? distCentro.get() : -1.0f;
  float dDer = nd ? distDer.get() : -1.0f;

  bool a = false, b = false, c = false;
  if (buttonsAlive) {
    buttons.update();
    a = buttons.isPressed('A') == HIGH;
    b = buttons.isPressed('B') == HIGH;
    c = buttons.isPressed('C') == HIGH;
    buttons.setLeds(a, b, c);
  }

  // Boton C: parada de emergencia / reset, en CUALQUIER estado.
  if (c && !prevC) {
    raceStop();
    raceState = RACE_IDLE;
  }
  // Boton A: arranca el Open Challenge (solo si esta en IDLE, hay boton,
  // motor y giroscopio disponibles -- sin gyro no hay como girar 90 grados).
  if (a && !prevA && raceState == RACE_IDLE && buttonsAlive && motorAlive && gyroOk) {
    raceStart();
  }
  prevA = a;
  prevB = b;
  prevC = c;

  // --- FSM de la carrera ---
  if (raceState == RACE_DRIVE) {
    if (!motorAlive) {
      raceStop();
    } else {
      float steer = 0;
      bool niPlausible = ni && dIzq <= MAX_PLAUSIBLE_SIDE_MM;
      bool ndPlausible = nd && dDer <= MAX_PLAUSIBLE_SIDE_MM;
      g_niPlausible = niPlausible; g_ndPlausible = ndPlausible; g_dIzq = dIzq; g_dDer = dDer;
      if (niPlausible && ndPlausible) {
        float error = dIzq - dDer;  // >0: pared izq mas lejos -> corregir a la izq
        steer = constrain(error * WALL_KP, -WALL_STEER_MAX, WALL_STEER_MAX);
      }
      set_servo_pct((int)steer);  // PWM puro, no I2C -- se puede llamar cada ciclo sin problema
      // El motor SI es I2C -- no reescribir la misma velocidad miles de veces
      // por segundo (eso fue lo que colgo el bus antes). Alcanza con refrescar
      // cada MOTOR_CMD_INTERVAL_MS.
      if (now - lastMotorCmdMs > MOTOR_CMD_INTERVAL_MS) {
        lastMotorCmdMs = now;
        motors.setInvertA(false);  // false = adelante fisico con el motor EV3 nuevo
        motors.setSpeedA(CRUISE_PCT);
      }

      // Reactivado con umbral bajo (ver constante arriba) -- el lidar quedo
      // descartado tambien (autoobstruido en los 360 grados con el chasis
      // nuevo). La linea de camara sigue activa como respaldo en paralelo.
      if (nc && dCentro > 0 && dCentro < CORNER_FRONT_TRIGGER_MM && now > cornerCooldownUntil) {
        startCornerTurn();
      }
    }
  } else if (raceState == RACE_TURN) {
    if (now - lastMotorCmdMs > MOTOR_CMD_INTERVAL_MS) {
      lastMotorCmdMs = now;
      motors.setInvertA(false);  // false = adelante fisico con el motor EV3 nuevo
      motors.setSpeedA(TURN_PCT);
    }
    set_servo_pct(turnLeft ? -TURN_STEER_PCT : TURN_STEER_PCT);
    // Corta por giroscopio (angulo real alcanzado) -- vuelta a la sugerencia
    // original. TURN_TIMEOUT_MS queda como respaldo de seguridad por si el
    // gyro no cierra el angulo (para no quedar trabado girando para siempre).
    float diff = normalizeAngle(raceTargetHeading - headingDeg);
    bool timedOut = (now - turnStartMs) > TURN_TIMEOUT_MS;
    if (fabs(diff) < HEADING_TOL_DEG || timedOut) {
      set_servo_pct(0);
      cornerCooldownUntil = now + CORNER_COOLDOWN_MS;
      if (cornerCount >= MAX_CORNERS) {
        raceStop();
      } else {
        raceState = RACE_DRIVE;
      }
    }
  } else {  // RACE_IDLE o RACE_STOP
    // OJO: antes esto llamaba motors.setSpeedA(0) (y ademas (bool)motors,
    // que tambien es I2C) en CADA iteracion del loop sin parar -- eso
    // satura el bus I2C con miles de escrituras redundantes por segundo y
    // termino colgando el bus entero (y con el, el loop() completo, lo que
    // tambien tumba Bridge). Ahora solo se manda el stop UNA VEZ al entrar
    // al estado, con un flag, igual que hace raceStop().
    if (!idleStopSent) {
      if ((bool)motors) motors.setSpeedA(0);
      idleStopSent = true;
    }
  }

  // Estado en vivo de los 3 botones + carrera, cada 100ms.
  static unsigned long lastBtnReport = 0;
  if (now - lastBtnReport > 100) {
    lastBtnReport = now;
    Bridge.notify("btn_a_live", a ? 1 : 0);
    Bridge.notify("btn_b_live", b ? 1 : 0);
    Bridge.notify("btn_c_live", c ? 1 : 0);
    Bridge.notify("live_gyro", gyroOk ? 1 : 0);
    Bridge.notify("gyro_heading", headingDeg);
    Bridge.notify("race_state", (int)raceState);
    Bridge.notify("race_corners", cornerCount);
  }

  if (now - lastReport > 500) {
    lastReport = now;
    Bridge.notify("dist_izq_mm", ni ? dIzq : -1.0f);
    Bridge.notify("dist_centro_mm", nc ? dCentro : -1.0f);
    Bridge.notify("dist_der_mm", nd ? dDer : -1.0f);
    // Repite el estado de presencia cada ciclo -- el log rota rapido y el
    // resumen inicial de setup() se pierde en corridas largas.
    Bridge.notify("live_buttons", buttonsAlive ? 1 : 0);
    Bridge.notify("live_dist_izq", (bool)distIzq ? 1 : 0);
    Bridge.notify("live_dist_centro", (bool)distCentro ? 1 : 0);
    Bridge.notify("live_dist_der", (bool)distDer ? 1 : 0);
    Bridge.notify("live_motor", motorAlive ? 1 : 0);
  }
}
