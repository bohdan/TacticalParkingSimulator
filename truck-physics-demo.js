"use strict";

// ─── tuneable parameters (driven by panel) ───────────────────────────────────
let SCALE        = 21;
let WORLD_W = 0, WORLD_H = 0;

const CAB_LEN  = 5.5, CAB_WID  = 2.5, CAB_ROVER  = 0.8;
const TRL_WID  = 2.5, TRL_ROVER = 0.8;
const MAX_SPEED = 5.0, ACCEL = 3.0, STEER_RATE = 60 * Math.PI / 180;

let CAB_WB    = 4.0;
let HITCH_OFF = 0.4;
let TRL_WB    = 8.0;
let TRL_LEN   = TRL_WB + TRL_ROVER + 0.2;   // front overhang = 0.2 m fixed

let MAX_STEER  = 35 * Math.PI / 180;
let MAX_ART    = 75 * Math.PI / 180;

let GHOST_COUNT  = 10;
let PROJ_DIST    = 20;
let PROJ_STEPS   = 80;   // recalculated when PROJ_DIST changes
let ARROW_EVERY  = 4;    // metres between direction arrows on trail

let showGhosts = true;
let showTrail  = true;

// ─── canvas ──────────────────────────────────────────────────────────────────
const canvas = document.getElementById('canvas');
const ctx    = canvas.getContext('2d');

function resize() {
  canvas.width  = window.innerWidth;
  canvas.height = window.innerHeight;
  WORLD_W = canvas.width  / SCALE;
  WORLD_H = canvas.height / SCALE;
}
resize();
window.addEventListener('resize', resize);

// ─── state ───────────────────────────────────────────────────────────────────
let state, keys = {};

function reset() {
  state = {
    cx: WORLD_W * 0.62, cy: WORLD_H * 0.5,
    th: Math.PI, tt: Math.PI,
    speed: 0, steer: 0, blocked: false,
  };
  const hw = hitch(state);
  state.trx = hw.x - TRL_WB * Math.cos(state.tt);
  state.try = hw.y - TRL_WB * Math.sin(state.tt);
}

function hitch(s) {
  return {
    x: s.cx - HITCH_OFF * Math.cos(s.th),
    y: s.cy - HITCH_OFF * Math.sin(s.th),
  };
}

function normAngle(a) {
  while (a >  Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// ─── physics step ────────────────────────────────────────────────────────────
function step(dt) {
  const steerInput = (keys['ArrowLeft']  || keys['a'] || keys['A'] ? -1 : 0)
                   + (keys['ArrowRight'] || keys['d'] || keys['D'] ?  1 : 0);
  state.steer += steerInput * STEER_RATE * dt;
  state.steer = Math.max(-MAX_STEER, Math.min(MAX_STEER, state.steer));

  const driveInput = (keys['ArrowUp']   || keys['w'] || keys['W'] ?  1 : 0)
                   + (keys['ArrowDown'] || keys['s'] || keys['S'] ? -1 : 0);
  state.speed += driveInput * ACCEL * dt;
  state.speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, state.speed));
  if (!driveInput) state.speed *= Math.pow(0.1, dt);

  const v = state.speed;
  const δ = state.steer;

  const prev = { cx: state.cx, cy: state.cy, th: state.th,
                 tt: state.tt, trx: state.trx, try: state.try };

  const dth = (Math.abs(δ) > 0.0001) ? v * Math.tan(δ) / CAB_WB * dt : 0;
  state.th += dth;
  state.cx += v * Math.cos(state.th) * dt;
  state.cy += v * Math.sin(state.th) * dt;

  const hw = hitch(state);
  state.tt  = Math.atan2(hw.y - state.try, hw.x - state.trx);
  state.trx = hw.x - TRL_WB * Math.cos(state.tt);
  state.try = hw.y - TRL_WB * Math.sin(state.tt);

  const β = normAngle(state.th - state.tt);
  if (Math.abs(β) > MAX_ART) {
    Object.assign(state, prev);
    state.blocked = true;
  } else {
    state.blocked = false;
  }

  state.cx = Math.max(1, Math.min(WORLD_W - 1, state.cx));
  state.cy = Math.max(1, Math.min(WORLD_H - 1, state.cy));
}

// ─── path projection ─────────────────────────────────────────────────────────
function simPath(s, steer, totalDist, steps) {
  let p = { cx: s.cx, cy: s.cy, th: s.th, tt: s.tt, trx: s.trx, try: s.try };
  const ds = totalDist / steps;
  const cab = [{ x: p.cx,  y: p.cy,  th: p.th }];
  const trl = [{ x: p.trx, y: p.try, th: p.tt }];
  for (let i = 0; i < steps; i++) {
    p.th += (Math.abs(steer) > 0.0001 ? Math.tan(steer) / CAB_WB * ds : 0);
    p.cx += Math.cos(p.th) * ds;
    p.cy += Math.sin(p.th) * ds;
    const hw = { x: p.cx - HITCH_OFF * Math.cos(p.th), y: p.cy - HITCH_OFF * Math.sin(p.th) };
    const tt  = Math.atan2(hw.y - p.try, hw.x - p.trx);
    if (Math.abs(normAngle(p.th - tt)) > MAX_ART) break;   // stop at articulation limit
    p.tt  = tt;
    p.trx = hw.x - TRL_WB * Math.cos(p.tt);
    p.try = hw.y - TRL_WB * Math.sin(p.tt);
    cab.push({ x: p.cx,  y: p.cy,  th: p.th });
    trl.push({ x: p.trx, y: p.try, th: p.tt });
  }
  return { cab, trl };
}

function drawTrailWithArrows(pts, color) {
  if (pts.length < 2) return;
  ctx.save();

  // dashed polyline
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 5]);
  ctx.globalAlpha = 0.3;
  ctx.beginPath();
  ctx.moveTo(wx(pts[0].x), wy(pts[0].y));
  for (let i = 1; i < pts.length; i++) ctx.lineTo(wx(pts[i].x), wy(pts[i].y));
  ctx.stroke();
  ctx.setLineDash([]);

  // direction chevrons every ARROW_EVERY metres
  const stepDist = PROJ_DIST / PROJ_STEPS;
  const gap = Math.max(1, Math.round(ARROW_EVERY / stepDist));
  ctx.globalAlpha = 0.55;
  ctx.fillStyle = color;
  for (let i = gap; i < pts.length; i += gap) {
    const p   = pts[i];
    const ang = p.th;
    const sz  = 0.22 * SCALE;
    ctx.save();
    ctx.translate(wx(p.x), wy(p.y));
    ctx.rotate(ang);
    ctx.beginPath();
    ctx.moveTo(sz, 0);
    ctx.lineTo(-sz * 0.55, -sz * 0.42);
    ctx.lineTo(-sz * 0.55,  sz * 0.42);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
  ctx.restore();
}

function drawGhostRect(x, y, heading, length, frontOver, width, color, alpha, arrowDir) {
  const rearLen = length - frontOver;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.translate(wx(x), wy(y));
  ctx.rotate(heading);
  ctx.beginPath();
  ctx.rect(-rearLen * SCALE, -width / 2 * SCALE, length * SCALE, width * SCALE);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.stroke();
  // direction arrow on the leading face
  const tipX  = (arrowDir > 0 ? frontOver : -rearLen) * SCALE;
  const baseX = tipX - arrowDir * 0.55 * SCALE;
  const aw    = 0.26 * SCALE;
  ctx.beginPath();
  ctx.moveTo(tipX, 0);
  ctx.lineTo(baseX, -aw);
  ctx.lineTo(baseX,  aw);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

function drawProjection(s) {
  const cabFront = CAB_LEN  - CAB_ROVER;
  const trlFront = TRL_LEN  - TRL_ROVER;
  for (const [dir, cabCol, trlCol] of [[1, '#4af', '#2b8'], [-1, '#f84', '#f42']]) {
    const path = simPath(s, s.steer, dir * PROJ_DIST, PROJ_STEPS);
    const n    = path.cab.length;

    if (showTrail) {
      drawTrailWithArrows(path.cab, cabCol);
      drawTrailWithArrows(path.trl, trlCol);
    }

    if (showGhosts && n > 1) {
      for (let g = 1; g <= GHOST_COUNT; g++) {
        const idx   = Math.min(Math.round(g / GHOST_COUNT * (n - 1)), n - 1);
        const alpha = 0.55 - 0.04 * (g - 1);
        const c = path.cab[idx];
        const t = path.trl[idx];
        if (!c || !t) continue;
        drawGhostRect(c.x, c.y, c.th, CAB_LEN, cabFront, CAB_WID, cabCol, alpha, dir);
        drawGhostRect(t.x, t.y, t.th, TRL_LEN, trlFront, TRL_WID, trlCol, alpha, dir);
      }
    }
  }
}

// ─── rendering ───────────────────────────────────────────────────────────────
function wx(x) { return x * SCALE; }
function wy(y) { return y * SCALE; }

function drawRect(x, y, heading, length, frontOver, width, fill, stroke) {
  ctx.save();
  ctx.translate(wx(x), wy(y));
  ctx.rotate(heading);
  ctx.beginPath();
  ctx.rect(-(length - frontOver) * SCALE, -width / 2 * SCALE, length * SCALE, width * SCALE);
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke(); }
  ctx.restore();
}

function drawCab(s) {
  const frontOver = CAB_LEN - CAB_ROVER;
  drawRect(s.cx, s.cy, s.th, CAB_LEN, frontOver, CAB_WID, '#1a2a4a', '#4a6a9a');
  ctx.save();
  ctx.translate(wx(s.cx), wy(s.cy));
  ctx.rotate(s.th);
  ctx.strokeStyle = '#6af';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  const wsy = CAB_WID * 0.5 * 0.7 * SCALE;
  const wsx = (frontOver - 0.8) * SCALE;
  ctx.moveTo(wsx, -wsy); ctx.lineTo(wsx, wsy);
  ctx.stroke();
  ctx.restore();
}

function drawTrailer(s) {
  const frontOver = TRL_LEN - TRL_ROVER;
  drawRect(s.trx, s.try, s.tt, TRL_LEN, frontOver, TRL_WID, '#888880', '#aaa');
  ctx.save();
  ctx.translate(wx(s.trx), wy(s.try));
  ctx.rotate(s.tt);
  ctx.strokeStyle = '#555';
  ctx.lineWidth = 1;
  const hw2 = TRL_WID * 0.5 * SCALE;
  ctx.beginPath();
  ctx.moveTo(-TRL_ROVER * SCALE, -hw2);
  ctx.lineTo(-TRL_ROVER * SCALE,  hw2);
  ctx.stroke();
  ctx.restore();
}

function drawHitch(s) {
  const hw = hitch(s);
  ctx.beginPath();
  ctx.arc(wx(hw.x), wy(hw.y), 4, 0, Math.PI * 2);
  ctx.fillStyle = '#f55';
  ctx.fill();
}

function drawGround() {
  ctx.fillStyle = '#1e2830';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#252f38';
  ctx.lineWidth = 0.5;
  for (let x = 0; x <= WORLD_W; x += 5) {
    ctx.beginPath(); ctx.moveTo(wx(x), 0); ctx.lineTo(wx(x), wy(WORLD_H)); ctx.stroke();
  }
  for (let y = 0; y <= WORLD_H; y += 5) {
    ctx.beginPath(); ctx.moveTo(0, wy(y)); ctx.lineTo(wx(WORLD_W), wy(y)); ctx.stroke();
  }
}

function jackknife() {
  let a = (state.th - state.tt) * 180 / Math.PI;
  while (a >  180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

function updateHUD() {
  const jk   = jackknife();
  const warn  = Math.abs(jk) > 30;
  const el    = document.getElementById('hud-text');
  const suffix = state.blocked ? '  ✖ BLOCKED' : (warn ? '  ⚠ JACKKNIFE' : '');
  el.textContent = `speed: ${state.speed.toFixed(1)} m/s  |  steer: ${(state.steer * 180 / Math.PI).toFixed(0)}°  |  jackknife: ${jk.toFixed(0)}°${suffix}`;
  el.style.color = state.blocked ? '#f55' : (warn ? '#fa0' : '#888');
}

// ─── loop ────────────────────────────────────────────────────────────────────
let last = null;
function loop(ts) {
  const dt = last ? Math.min((ts - last) / 1000, 0.05) : 0;
  last = ts;
  step(dt);
  drawGround();
  drawProjection(state);
  drawTrailer(state);
  drawCab(state);
  drawHitch(state);
  updateHUD();
  requestAnimationFrame(loop);
}

// ─── input ───────────────────────────────────────────────────────────────────
document.addEventListener('keydown', e => {
  keys[e.key] = true;
  if (e.key === 'r' || e.key === 'R') reset();
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) e.preventDefault();
});
document.addEventListener('keyup', e => { keys[e.key] = false; });

// ─── panel wiring ────────────────────────────────────────────────────────────
function wire(id, callback) {
  document.getElementById(id).addEventListener('input', callback);
}

wire('zoom', e => {
  SCALE = +e.target.value;
  resize();
  document.getElementById('zoomVal').textContent = SCALE + ' px/m';
});
wire('showGhosts', e => { showGhosts = e.target.checked; });
wire('showTrail',  e => { showTrail  = e.target.checked; });

wire('projDist',   e => {
  PROJ_DIST  = +e.target.value;
  PROJ_STEPS = Math.round(PROJ_DIST / 0.25);
  document.getElementById('projDistVal').textContent = PROJ_DIST + ' m';
});
wire('ghostCount', e => {
  GHOST_COUNT = +e.target.value;
  document.getElementById('ghostCountVal').textContent = GHOST_COUNT;
});
wire('arrowEvery', e => {
  ARROW_EVERY = +e.target.value;
  document.getElementById('arrowEveryVal').textContent = ARROW_EVERY + ' m';
});
wire('maxSteer', e => {
  MAX_STEER = +e.target.value * Math.PI / 180;
  state.steer = Math.max(-MAX_STEER, Math.min(MAX_STEER, state.steer));
  document.getElementById('maxSteerVal').textContent = e.target.value + '°';
});
wire('maxArt', e => {
  MAX_ART = +e.target.value * Math.PI / 180;
  document.getElementById('maxArtVal').textContent = e.target.value + '°';
});
wire('cabWb', e => {
  CAB_WB = +e.target.value;
  document.getElementById('cabWbVal').textContent = CAB_WB.toFixed(1) + ' m';
});
wire('trlWb', e => {
  TRL_WB  = +e.target.value;
  TRL_LEN = TRL_WB + TRL_ROVER + 0.2;
  // reposition trailer to match new length
  const hw = hitch(state);
  state.trx = hw.x - TRL_WB * Math.cos(state.tt);
  state.try = hw.y - TRL_WB * Math.sin(state.tt);
  document.getElementById('trlWbVal').textContent = TRL_WB.toFixed(1) + ' m';
});
wire('hitchOff', e => {
  HITCH_OFF = +e.target.value;
  document.getElementById('hitchOffVal').textContent = HITCH_OFF.toFixed(1) + ' m';
});

// ─── start ───────────────────────────────────────────────────────────────────
reset();
requestAnimationFrame(loop);
