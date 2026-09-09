const ui = new WebUI();

const distEl = { izq: document.querySelector('#dist-izq'), centro: document.querySelector('#dist-centro'), der: document.querySelector('#dist-der') };
const dotEl = {
  izq: document.querySelector('#dot-izq'),
  centro: document.querySelector('#dot-centro'),
  der: document.querySelector('#dot-der'),
  btn: document.querySelector('#dot-btn'),
  motor: document.querySelector('#dot-motor'),
  lidar: document.querySelector('#dot-lidar'),
  gyro: document.querySelector('#dot-gyro'),
};
const lightEl = { a: document.querySelector('#light-a'), b: document.querySelector('#light-b'), c: document.querySelector('#light-c') };
const gyroHeadingEl = document.querySelector('#gyro-heading');
const raceStateEl = document.querySelector('#race-state');
const raceCornersEl = document.querySelector('#race-corners');
const cameraLineEl = document.querySelector('#camera-line');
const lidarFrontEl = document.querySelector('#lidar-front');

function setDot(el, ok) {
  el.classList.remove('ok', 'bad');
  el.classList.add(ok ? 'ok' : 'bad');
}

ui.on_message('state', (s) => {
  distEl.izq.textContent = s.dist_izq_mm != null ? Math.round(s.dist_izq_mm) : '--';
  distEl.centro.textContent = s.dist_centro_mm != null ? Math.round(s.dist_centro_mm) : '--';
  distEl.der.textContent = s.dist_der_mm != null ? Math.round(s.dist_der_mm) : '--';

  setDot(dotEl.izq, s.dist_izq_ok);
  setDot(dotEl.centro, s.dist_centro_ok);
  setDot(dotEl.der, s.dist_der_ok);
  setDot(dotEl.btn, s.buttons_ok);
  setDot(dotEl.motor, s.motor_ok);

  lightEl.a.classList.toggle('pressed', !!s.btn_a);
  lightEl.b.classList.toggle('pressed', !!s.btn_b);
  lightEl.c.classList.toggle('pressed', !!s.btn_c);

  setDot(dotEl.gyro, s.gyro_ok);
  gyroHeadingEl.textContent = s.gyro_heading != null ? s.gyro_heading.toFixed(1) : '--';

  raceStateEl.textContent = s.race_state || '--';
  raceCornersEl.textContent = s.race_corners != null ? s.race_corners : 0;
  cameraLineEl.textContent = s.camera_line || '--';
  lidarFrontEl.textContent = s.lidar_front_mm != null ? Math.round(s.lidar_front_mm) : '--';
});

document.querySelector('#race-stop-btn').addEventListener('click', () => {
  ui.send_message('race_stop', {});
});

// --- Servo ---
const slider = document.querySelector('#servo-slider');
const servoValue = document.querySelector('#servo-value');
let lastSent = 0;

function sendServo(pct) {
  const now = Date.now();
  if (now - lastSent < 60) return; // no saturar el bridge
  lastSent = now;
  ui.send_message('set_servo', { pct });
}

slider.addEventListener('input', () => {
  const pct = parseInt(slider.value, 10);
  servoValue.textContent = pct;
  sendServo(pct);
});

document.querySelector('#servo-center-btn').addEventListener('click', () => {
  slider.value = 0;
  servoValue.textContent = 0;
  ui.send_message('set_servo', { pct: 0 });
});

// --- Motor ---
document.querySelectorAll('.motor-buttons button').forEach((btn) => {
  btn.addEventListener('click', () => {
    const pct = parseInt(btn.dataset.pct, 10);
    ui.send_message('test_motor', { pct });
  });
});

// --- Lidar ---
const canvas = document.querySelector('#lidar-canvas');
const ctx = canvas.getContext('2d');
const statusText = document.querySelector('#lidar-status-text');
const MAX_MM = 3000;
const CENTER = 180;
const RADIUS_PX = 165;

// --- Overlay de proximidad sobre la camara (solo visual, no maneja) ---
// Estimado para la Logitech C930e (~70 grados horizontales). Si al
// probarlo se ve desalineado con lo que realmente ve la camara, ajustar
// este numero.
const CAMERA_FOV_DEG = 70;
const OVERLAY_MAX_MM = 1500; // rango cercano -- lo lejano no importa tanto aqui
const cameraImg = document.querySelector('#camera');
const overlay = document.querySelector('#camera-overlay');
const octx = overlay.getContext('2d');

function syncOverlaySize() {
  overlay.width = cameraImg.clientWidth;
  overlay.height = cameraImg.clientHeight;
}
window.addEventListener('resize', syncOverlaySize);
cameraImg.addEventListener('load', syncOverlaySize);
syncOverlaySize();

function drawCameraOverlay(scan) {
  syncOverlaySize();
  const w = overlay.width, h = overlay.height;
  if (!w || !h) return;
  octx.clearRect(0, 0, w, h);
  if (!scan || !scan.length) return;

  const half = CAMERA_FOV_DEG / 2;
  octx.beginPath();
  let started = false;
  let anyClose = false;
  for (let deg = -half; deg <= half; deg += 1) {
    // el lidar guarda 0-359, angulo 0 = adelante (igual convencion que el radar)
    const idx = ((Math.round(deg) % 360) + 360) % 360;
    const mm = scan[idx];
    const x = ((deg + half) / CAMERA_FOV_DEG) * w;
    let y;
    if (mm == null) {
      y = h; // sin dato -- linea al piso
    } else {
      const clamped = Math.min(Math.max(mm, 0), OVERLAY_MAX_MM);
      y = h - (clamped / OVERLAY_MAX_MM) * h; // mas cerca = mas arriba
      if (clamped < OVERLAY_MAX_MM * 0.25) anyClose = true;
    }
    if (!started) { octx.moveTo(x, y); started = true; }
    else octx.lineTo(x, y);
  }
  octx.strokeStyle = anyClose ? '#e5484d' : '#3ecf6b';
  octx.lineWidth = 2;
  octx.stroke();
}

ui.on_message('lidar', (msg) => {
  setDot(dotEl.lidar, !!msg.connected);
  statusText.textContent = msg.connected
    ? `conectado — ${msg.packets_ok} paquetes`
    : (msg.error || 'sin conexión');
  drawCameraOverlay(msg.scan);

  ctx.clearRect(0, 0, 360, 360);

  // anillos de referencia
  ctx.strokeStyle = '#2a2d33';
  ctx.lineWidth = 1;
  [1000, 2000, 3000].forEach((mm) => {
    const r = (mm / MAX_MM) * RADIUS_PX;
    ctx.beginPath();
    ctx.arc(CENTER, CENTER, r, 0, Math.PI * 2);
    ctx.stroke();
  });

  // robot al centro, "adelante" = hacia arriba (angulo 0)
  ctx.fillStyle = '#f5b400';
  ctx.beginPath();
  ctx.arc(CENTER, CENTER, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#f5b400';
  ctx.beginPath();
  ctx.moveTo(CENTER, CENTER);
  ctx.lineTo(CENTER, CENTER - 14);
  ctx.stroke();

  ctx.fillStyle = '#3ecf6b';
  const scan = msg.scan || [];
  for (let deg = 0; deg < scan.length; deg++) {
    const mm = scan[deg];
    if (mm == null) continue;
    const r = Math.min(mm / MAX_MM, 1) * RADIUS_PX;
    const rad = (deg - 90) * (Math.PI / 180); // 0 grados = arriba
    const x = CENTER + r * Math.cos(rad);
    const y = CENTER + r * Math.sin(rad);
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
});
