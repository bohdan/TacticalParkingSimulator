// touch-controls-demo.ts — standalone playground for alternative steer/distance drag
// gestures (see the design discussion this came out of). Not part of the game; a single
// static car + one editable move, so the four control schemes below can be compared
// side by side without any of the rest of game.ts's state. Uses the REAL kernel (CAR
// dims, advance()) so the geometry/feel matches the actual game exactly.
//
// Schemes:
//   A — combined arc-drag: today's game.ts mechanic (arcToPoint), one drag controls
//       steer+distance together via the circle tangent to the start heading.
//   B — front/back zones: touch near the car's front bumper -> steer only; near the
//       rear bumper -> distance only; outside both zones, fall back to either A or C
//       (toggle in the panel). Zone radius has a minimum on-screen size so it's not
//       finger-tip-sized on a phone.
//   C — direction-lock: no zones at all. Any drag starts "pending"; once it moves past
//       a small deadzone, whichever axis (lateral vs longitudinal, in the car's local
//       frame) the drag moved further along wins for the rest of that gesture.
//   D — radial twin-stick: touch-down point is the stick's centre; the vector from
//       there to the current pointer decomposes into forward/back (-> distance,
//       absolute, like a real analog stick) and lateral (-> steer).
import { CAR, setVehicle, rad, deg, advance, carPoly } from './physics-compat.js';

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const ctx = canvas.getContext('2d')!;
let SCALE = 28; // px per metre

function resize() {
  canvas.width = innerWidth;
  canvas.height = innerHeight;
}
resize();
window.addEventListener('resize', resize);

function w2c(x: number, y: number) { return { x: canvas.width / 2 + x * SCALE, y: canvas.height / 2 + y * SCALE }; }
function c2w(cx: number, cy: number) { return { x: (cx - canvas.width / 2) / SCALE, y: (cy - canvas.height / 2) / SCALE }; }

// ── state ──────────────────────────────────────────────────────────────────
setVehicle('default');
const pose = { x: 0, y: 0, h: -Math.PI / 2 }; // pointing "up" on screen
let steer = 0, dist = 0; // current edited move: degrees, metres
let mode: 'A' | 'B' | 'C' | 'D' = 'A';
let showZones = true;
let fallbackMode: 'combined' | 'lock' = 'combined';
let scrollNudge = true;

const MODE_DESC: Record<string, string> = {
  A: 'Grab anywhere; the ghost’s rear axle follows your finger along the arc through that point. Steer and distance change together.',
  B: 'Grab near the front bumper to steer only, near the rear bumper for distance only. Elsewhere, falls back to the scheme picked below.',
  C: 'Grab anywhere. After a small deadzone, whichever direction you moved further (lateral vs. along the car) locks in for the rest of the drag.',
  D: 'Touch down anywhere to plant a virtual stick. Push away from that point: forward/back = distance, left/right = steer — absolute, like a joystick.',
};

function frontPoint(p: any) { const d = CAR.wb + CAR.fOver; return { x: p.x + Math.cos(p.h) * d, y: p.y + Math.sin(p.h) * d }; }
function rearPoint(p: any) { const d = CAR.rOver; return { x: p.x - Math.cos(p.h) * d, y: p.y - Math.sin(p.h) * d }; }
function zoneRadius() { return Math.max(CAR.wid * 0.9, 42 / SCALE); } // world units; 42px min touch target

// Local (car-frame) components of a WORLD POINT relative to pose (forward, lateral).
function localOf(p: any, wx: number, wy: number) {
  const dx = wx - p.x, dy = wy - p.y;
  const c = Math.cos(p.h), s = Math.sin(p.h);
  return { lx: dx * c + dy * s, ly: -dx * s + dy * c };
}
// Local (car-frame) components of a WORLD DELTA VECTOR (no translation).
function localDelta(p: any, dx: number, dy: number) {
  const c = Math.cos(p.h), s = Math.sin(p.h);
  return { lx: dx * c + dy * s, ly: -dx * s + dy * c };
}

// Mode A: fit the constant-steer circle from `p` through world point (wx,wy).
function arcToPoint(p: any, wx: number, wy: number) {
  const { lx, ly } = localOf(p, wx, wy);
  if (Math.abs(ly) < 0.05) return { steer: 0, dist: lx };
  const R = (lx * lx + ly * ly) / (2 * ly);
  const th = Math.atan2(lx / R, (R - ly) / R);
  return { steer: deg(Math.atan(CAR.wb / R)), dist: R * th };
}

const DEG_PER_METRE = 60; // mode B/C steer sensitivity: 1m lateral drag = 60deg (clamped to maxSteer)
const DEADZONE_PX = 10;

type Axis = 'combined' | 'steer' | 'dist' | 'pending';
let drag: null | {
  x: number; y: number;           // client coords at drag start
  lastX: number; lastY: number;   // live client coords (for the mode-D stick line)
  tx: number; ty: number;         // world target for mode A (ghost end pose)
  startSteer: number; startDist: number;
  axis: Axis;
} = null;

function ghostEndPose() { return Math.abs(dist) > 0.005 ? advance(pose, rad(steer), dist) : { ...pose }; }

function onDown(clientX: number, clientY: number) {
  const w = c2w(clientX, clientY);
  const end = ghostEndPose();
  let axis: Axis = 'combined';
  if (mode === 'A') axis = 'combined';
  else if (mode === 'D') axis = 'combined'; // mode D uses its own onMove math; axis unused
  else if (mode === 'C') axis = 'pending';
  else { // mode B
    const F = frontPoint(pose), R = rearPoint(pose);
    const zr = zoneRadius();
    const dF = Math.hypot(w.x - F.x, w.y - F.y);
    const dR = Math.hypot(w.x - R.x, w.y - R.y);
    if (dF < zr && dF <= dR) axis = 'steer';
    else if (dR < zr) axis = 'dist';
    else axis = fallbackMode === 'combined' ? 'combined' : 'pending';
  }
  drag = { x: clientX, y: clientY, lastX: clientX, lastY: clientY, tx: end.x, ty: end.y, startSteer: steer, startDist: dist, axis };
}

function onMove(clientX: number, clientY: number) {
  if (!drag) return;
  drag.lastX = clientX; drag.lastY = clientY;
  const dxPx = clientX - drag.x, dyPx = clientY - drag.y;
  const wdx = dxPx / SCALE, wdy = dyPx / SCALE;

  if (mode === 'D') {
    const r = Math.hypot(dxPx, dyPx);
    if (r < DEADZONE_PX) { steer = drag.startSteer; dist = drag.startDist; return; }
    const { lx, ly } = localDelta(pose, wdx, wdy);
    dist = lx; // absolute: stick position IS the distance, like a real analog stick
    const steerFrac = Math.max(-1, Math.min(1, ly / (120 / SCALE)));
    steer = steerFrac * CAR.maxSteer;
    return;
  }

  if (drag.axis === 'pending') {
    if (Math.hypot(dxPx, dyPx) < DEADZONE_PX) return;
    const { lx, ly } = localDelta(pose, wdx, wdy);
    drag.axis = Math.abs(ly) > Math.abs(lx) ? 'steer' : 'dist';
  }

  if (drag.axis === 'combined') {
    const a = arcToPoint(pose, drag.tx + wdx, drag.ty + wdy);
    steer = a.steer; dist = a.dist;
  } else if (drag.axis === 'steer') {
    const { ly } = localDelta(pose, wdx, wdy);
    const raw = drag.startSteer + ly * DEG_PER_METRE;
    const maxD = CAR.maxSteer;
    steer = Math.max(-maxD, Math.min(maxD, raw));
  } else if (drag.axis === 'dist') {
    const { lx } = localDelta(pose, wdx, wdy);
    dist = drag.startDist + lx;
  }
}

// ── pointer wiring ───────────────────────────────────────────────────────────
canvas.addEventListener('pointerdown', e => { canvas.setPointerCapture(e.pointerId); onDown(e.clientX, e.clientY); });
canvas.addEventListener('pointermove', e => onMove(e.clientX, e.clientY));
canvas.addEventListener('pointerup', () => { drag = null; });
canvas.addEventListener('pointercancel', () => { drag = null; });
canvas.addEventListener('wheel', e => {
  if (!scrollNudge) return;
  e.preventDefault();
  dist += -e.deltaY / 200;
}, { passive: false });

// ── panel wiring ─────────────────────────────────────────────────────────────
function setMode(m: 'A' | 'B' | 'C' | 'D') {
  mode = m; drag = null;
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.toggle('on', (b as HTMLElement).dataset.mode === m));
  (document.getElementById('modeDesc') as HTMLElement).textContent = MODE_DESC[m];
  const hint = document.getElementById('hud-hint') as HTMLElement;
  hint.textContent = m === 'B' ? 'Front zone = steer, rear zone = distance, elsewhere = fallback.'
    : m === 'C' ? 'Drag any direction — first movement decides steer vs. distance.'
    : m === 'D' ? 'Touch down, then push the virtual stick.'
    : 'Drag anywhere — steer and distance change together.';
}
document.querySelectorAll('.mode-btn').forEach(b => b.addEventListener('click', () => setMode((b as HTMLElement).dataset.mode as any)));
setMode('A');

(document.getElementById('showZones') as HTMLInputElement).addEventListener('change', e => showZones = (e.target as HTMLInputElement).checked);
(document.getElementById('fallbackMode') as HTMLSelectElement).addEventListener('change', e => fallbackMode = (e.target as HTMLSelectElement).value as any);
(document.getElementById('scrollNudge') as HTMLInputElement).addEventListener('change', e => scrollNudge = (e.target as HTMLInputElement).checked);
(document.getElementById('vehicle') as HTMLSelectElement).addEventListener('change', e => setVehicle((e.target as HTMLSelectElement).value));
document.getElementById('resetBtn')!.addEventListener('click', () => { steer = 0; dist = 0; });

// ── render ───────────────────────────────────────────────────────────────────
function drawCar(p: any, fill: string, stroke: string, lw = 2) {
  const poly = carPoly(p);
  ctx.beginPath();
  const p0 = w2c(poly[0].x, poly[0].y); ctx.moveTo(p0.x, p0.y);
  for (let i = 1; i < poly.length; i++) { const pt = w2c(poly[i].x, poly[i].y); ctx.lineTo(pt.x, pt.y); }
  ctx.closePath();
  ctx.fillStyle = fill; ctx.fill();
  ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke();
}

function draw() {
  ctx.fillStyle = '#0e1117';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // faint 1m grid
  ctx.strokeStyle = 'rgba(255,255,255,0.04)'; ctx.lineWidth = 1;
  const w0 = c2w(0, 0), w1 = c2w(canvas.width, canvas.height);
  ctx.beginPath();
  for (let x = Math.floor(w0.x); x <= Math.ceil(w1.x); x++) { const a = w2c(x, w0.y), b = w2c(x, w1.y); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); }
  for (let y = Math.floor(w0.y); y <= Math.ceil(w1.y); y++) { const a = w2c(w0.x, y), b = w2c(w1.x, y); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); }
  ctx.stroke();

  // trail: sample the arc from pose to the current (steer,dist)
  if (Math.abs(dist) > 0.005) {
    ctx.beginPath();
    const n = 40;
    for (let i = 0; i <= n; i++) {
      const pp = advance(pose, rad(steer), dist * i / n);
      const c = w2c(pp.x, pp.y);
      if (i === 0) ctx.moveTo(c.x, c.y); else ctx.lineTo(c.x, c.y);
    }
    ctx.strokeStyle = dist < 0 ? 'rgba(255,159,67,0.9)' : 'rgba(69,196,255,0.9)';
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  // mode B zone circles
  if (mode === 'B' && showZones) {
    const F = frontPoint(pose), R = rearPoint(pose), zr = zoneRadius();
    for (const [pt, label, color] of [[F, 'STEER', '#4af'], [R, 'DIST', '#fa5']] as any) {
      const c = w2c(pt.x, pt.y);
      ctx.beginPath(); ctx.setLineDash([5, 4]);
      ctx.arc(c.x, c.y, zr * SCALE, 0, 2 * Math.PI);
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = color; ctx.font = '11px monospace'; ctx.textAlign = 'center';
      ctx.fillText(label, c.x, c.y - zr * SCALE - 6);
    }
  }

  // mode D stick visualization: deadzone ring + a line from touch-down to the live pointer
  if (mode === 'D' && drag) {
    ctx.beginPath(); ctx.arc(drag.x, drag.y, DEADZONE_PX, 0, 2 * Math.PI);
    ctx.strokeStyle = '#666'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(drag.x, drag.y); ctx.lineTo(drag.lastX, drag.lastY);
    ctx.strokeStyle = '#8ef'; ctx.lineWidth = 2; ctx.stroke();
    ctx.beginPath(); ctx.arc(drag.x, drag.y, 4, 0, 2 * Math.PI); ctx.fillStyle = '#889'; ctx.fill();
    ctx.beginPath(); ctx.arc(drag.lastX, drag.lastY, 6, 0, 2 * Math.PI); ctx.fillStyle = '#8ef'; ctx.fill();
  }

  // static start car + ghost
  drawCar(pose, 'rgba(120,140,170,0.35)', 'rgba(180,200,230,0.9)');
  const end = advance(pose, rad(steer), dist);
  drawCar(end, 'rgba(255,224,80,0.22)', 'rgba(255,224,80,0.9)');

  // readout
  const axisLabel = mode === 'D' ? '—' : (drag ? drag.axis : '—');
  (document.getElementById('hud-text') as HTMLElement).textContent =
    `steer: ${steer.toFixed(1)}°  |  dist: ${dist.toFixed(2)} m  |  axis: ${axisLabel}`;
  (document.getElementById('steerVal') as HTMLElement).textContent = `${steer.toFixed(1)}°`;
  (document.getElementById('distVal') as HTMLElement).textContent = `${dist.toFixed(2)} m`;
  (document.getElementById('axisVal') as HTMLElement).textContent = String(axisLabel);

  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
