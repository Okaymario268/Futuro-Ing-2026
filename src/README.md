> 📂 **Source code** — Arduino sketch (C/C++) driving the servo & DC motor, and the Python Web UI for the camera and data handling.

# FuturoIng — Servo angle control with a web UI (Arduino UNO Q · App Lab)

An **Arduino App Lab** app for the **Arduino UNO Q**. Drag a slider in your
browser and the servo on **pin D9** moves to that angle in real time.

```
Browser slider ──socket.io──▶ Python (Web UI brick) ──RouterBridge RPC──▶ sketch ──▶ servo on D9
```

- **Python / Linux side** serves the web UI (a slider + dial) and forwards each
  angle to the MCU over the RouterBridge.
- **Sketch / MCU side** receives the angle via RPC and calls `servo.write(angle)`.

## Project layout

```
FuturoIng_Arduino/
├── app.yaml                # App Lab manifest — declares the `arduino:web_ui` brick
├── assets/
│   └── index.html          # the web UI (slider, live dial, presets) — served on port 7000
├── python/
│   ├── main.py             # Web UI + Bridge.call("set_angle", angle) -> MCU
│   └── requirements.txt
└── sketch/
    ├── sketch.ino          # servo on D9, set_angle()/get_angle() RPC handlers
    └── sketch.yaml         # declares Servo 1.3.0 + board (the library-resolution fix)
```

## How it works

**Sketch ([`sketch/sketch.ino`](sketch/sketch.ino))** — registers RPC handlers with
`Bridge.provide_safe(...)`:

```cpp
Bridge.provide_safe("set_angle", set_angle);   // servo.write(angle), returns applied angle
Bridge.provide_safe("get_angle", get_angle);   // lets the UI sync on connect
```

> `provide_safe` (not `provide`) is required for anything that touches hardware.
> It runs the callback in the `loop()` context; `provide` runs it on a background
> RPC thread where `servo.write()` / GPIO can misbehave. `loop()` stays empty —
> the bridge dispatches the safe callbacks from inside the loop itself.

**Python ([`python/main.py`](python/main.py))** — the Web UI brick exposes simple
REST endpoints; a handler parameter becomes a query parameter (FastAPI):

```python
ui = WebUI()
ui.expose_api("GET", "/api/angle", api_set_angle)  # /api/angle?angle=120
ui.expose_api("GET", "/api/state", api_state)      # current angle, for sync
# inside api_set_angle(angle: int):
applied = Bridge.call("set_angle", angle)          # python -> MCU
return {"angle": applied}
```

**UI ([`assets/index.html`](assets/index.html))** — calls
`fetch("/api/angle?angle=N")` as the slider moves and reads `/api/state` on load.

> **Why REST and not websockets?** This brick is FastAPI + `fastapi_socketio` and
> does **not** serve a Socket.IO *client* library (`/socket.io/socket.io.js` 404s),
> so an `io()` page hangs on "Connecting…". Plain `fetch()` is what the working
> community dashboards use, and it's rock-solid here.

## The library fix (why `Servo.h` wasn't found)

Arduino App Lab does **not** read the classic Arduino IDE library folder
(`...\Documentos\Arduino\libraries\`). It resolves each project's C++/MCU
libraries from **[`sketch/sketch.yaml`](sketch/sketch.yaml)**. `Servo 1.3.0` is the
first version with Zephyr / UNO Q support. App Lab needs the **profiles** form
(a plain `default_fqbn` + `libraries` file fails with *"Missing Profile name"*):

```yaml
profiles:
  default:
    fqbn:                       # leave blank — App Lab fills it from the connected UNO Q
    platforms:
      - platform: arduino:zephyr
    libraries:
      - Servo (1.3.0)
default_profile: default
```

`Arduino_RouterBridge` (the RPC bridge) ships with the App Lab template, so it is
not pinned here.

## Run it

1. Open **Arduino App Lab** → open this `FuturoIng_Arduino` app.
   - **If App Lab won't open this hand-made folder:** create a **New App**, add the
     **Web UI** brick (Bricks panel) and the **Servo** library (sketch editor →
     *Add Library*), then paste in the contents of `sketch/`, `python/` and `assets/`.
2. Connect the UNO Q, select it, and click **Run** (compiles + uploads the sketch,
   starts the Python program).
3. Open the app's **Web UI** (or `http://<board-ip>:7000`) and drag the slider.

### Wiring

| Servo wire | Connect to |
|------------|------------|
| Signal (orange/white) | **D9** |
| V+ (red) | external **5–6 V** supply (not the 3V3/5V pin for anything but a tiny servo) |
| GND (brown/black) | supply GND **and** a board GND (shared ground) |

## Verify

- **Compile:** build succeeds with no `Servo.h: No such file or directory`.
- **UI:** the page shows **Connected** (green dot); moving the slider updates the
  big angle readout and the dial needle.
- **Hardware:** the servo physically tracks the slider.

## UNO Q / Zephyr gotchas

- **PWM pins:** on Zephyr `servo.attach(pin)` needs a pin wired to a hardware PWM
  channel in the UNO Q device tree — not every Uno digital pin qualifies. If the
  servo doesn't move on **D9**, change `SERVO_PIN` in `sketch.ino` to another PWM
  header pin (e.g. D3 or D6) and re-run.
- Keep **Servo ≥ 1.3.0** in `sketch.yaml`. If *Add Library* can't find Servo,
  update App Lab and the `arduino:zephyr` core.
- `Monitor` (App Lab's serial monitor), not the classic IDE Serial Monitor.

---

## STAGE 2 — full FuturoIng robot port (reference)

The full robot (servo steering + L298N DC motor) reuses the same pattern: keep the
`Arduino_RouterBridge` handlers for anything the UI/Python should drive, and add the
motor pins. Faithful port of `Futuroing2026.ino`:

```cpp
#include <Arduino_RouterBridge.h>
#include <Servo.h>

// Motor (L298N)
int MotorVelocityPin = 3;   // ENA / PWM speed  -> must be PWM-capable on the UNO Q
int MotorPin1        = 6;   // IN1 direction
int MotorPin2        = 5;   // IN2 direction
int CurrentVelocity  = 150;

// Servo steering angles
int CenterServoRotation = 138;
int LeftServoRotation   = 125;
int RightServoRotation  = 150;
int ServoPin            = 9;   // (was 11 on the classic build; verify PWM-capable)

Servo servo;

void MotorForward()  { digitalWrite(MotorPin1, HIGH); digitalWrite(MotorPin2, LOW);  }
void MotorBackward() { digitalWrite(MotorPin1, LOW);  digitalWrite(MotorPin2, HIGH); }
void MotorStop()     { digitalWrite(MotorPin1, LOW);  digitalWrite(MotorPin2, LOW);  }
void ServoRotation(int Angle) { servo.write(Angle); }

void setup() {
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(MotorVelocityPin, OUTPUT);
  analogWrite(MotorVelocityPin, CurrentVelocity);

  Bridge.begin();
  Monitor.begin(115200);
  servo.attach(ServoPin);
  servo.write(CenterServoRotation);
  // Bridge.provide_safe("steer", ...);  // expose steering/drive to the UI as needed
}

void loop() {
  // obstacle-avoidance / steering logic
}
```
