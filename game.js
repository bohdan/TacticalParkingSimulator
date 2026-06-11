'use strict';

/* ===================== Car & math ===================== */

// All world units are meters. Pose = {x, y, h}; (x,y) is the REAR AXLE
// center, h is heading in radians (0 = +x, y axis points down on screen).
const CAR = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, maxSteer: 35 };
CAR.fOver = CAR.len - CAR.wb - CAR.rOver;

const MAX_MOVE_DIST = 12;    // m, per planned move
const SAMPLE_STEP = 0.06;    // m, collision sampling along path

const rad = d => d * Math.PI / 180;
const deg = r => r * 180 / Math.PI;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

function normAng(a) {
  a %= 2 * Math.PI;
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

// Kinematic bicycle model: constant steering angle traces a circular arc
// of the rear axle around the instantaneous center of rotation.
// steer in radians, s = signed arc length (negative = reverse).
function advance(p, steer, s) {
  if (Math.abs(steer) < 1e-4) {
    return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
  }
  const R = CAR.wb / Math.tan(steer);
  const cx = p.x - Math.sin(p.h) * R;
  const cy = p.y + Math.cos(p.h) * R;
  const h2 = p.h + s / R;
  return { x: cx + Math.sin(h2) * R, y: cy - Math.cos(h2) * R, h: h2 };
}

function carPoly(p, inf = 0) {
  const c = Math.cos(p.h), s = Math.sin(p.h);
  const x0 = -CAR.rOver - inf, x1 = CAR.wb + CAR.fOver + inf;
  const y0 = -CAR.wid / 2 - inf, y1 = CAR.wid / 2 + inf;
  const pt = (x, y) => ({ x: p.x + c * x - s * y, y: p.y + s * x + c * y });
  return [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
}

/* ===================== Collision (SAT, convex) ===================== */

function polysCollide(A, B) {
  for (const [P, Q] of [[A, B], [B, A]]) {
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      const nx = b.y - a.y, ny = a.x - b.x;
      let minP = Infinity, maxP = -Infinity, minQ = Infinity, maxQ = -Infinity;
      for (const v of P) { const d = v.x * nx + v.y * ny; if (d < minP) minP = d; if (d > maxP) maxP = d; }
      for (const v of Q) { const d = v.x * nx + v.y * ny; if (d < minQ) minQ = d; if (d > maxQ) maxQ = d; }
      if (maxP < minQ || maxQ < minP) return false;
    }
  }
  return true;
}

function pointInPoly(pt, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i], b = poly[j];
    if ((a.y > pt.y) !== (b.y > pt.y) &&
        pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

function centroid(poly) {
  let x = 0, y = 0;
  for (const v of poly) { x += v.x; y += v.y; }
  return { x: x / poly.length, y: y / poly.length };
}

// Approximate the point of contact for the hit marker.
function contactPoint(carP, obsP) {
  for (const v of carP) if (pointInPoly(v, obsP)) return v;
  for (const v of obsP) if (pointInPoly(v, carP)) return v;
  const c = centroid(obsP);
  let best = carP[0], bd = Infinity;
  for (const v of carP) {
    const d = (v.x - c.x) ** 2 + (v.y - c.y) ** 2;
    if (d < bd) { bd = d; best = v; }
  }
  return best;
}

// Sample the arc of one move; stop at the first colliding pose.
function simulateMove(start, steer, dist, obstacles) {
  const n = Math.max(2, Math.ceil(Math.abs(dist) / SAMPLE_STEP));
  const pts = [start];
  let hit = null;
  for (let i = 1; i <= n; i++) {
    const p = advance(start, steer, dist * i / n);
    const poly = carPoly(p);
    for (let oi = 0; oi < obstacles.length; oi++) {
      if (polysCollide(poly, obstacles[oi].poly)) {
        hit = { pose: p, point: contactPoint(poly, obstacles[oi].poly) };
        break;
      }
    }
    if (hit) break;
    pts.push(p);
  }
  return { pts, end: pts[pts.length - 1], hit };
}

/* ===================== Levels ===================== */

function rectPoly(x, y, w, h) {
  return [{ x, y }, { x: x + w, y }, { x: x + w, y: y + h }, { x, y: y + h }];
}

function obbPoly(cx, cy, w, h, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const pt = (x, y) => ({ x: cx + c * x - s * y, y: cy + s * x + c * y });
  return [pt(-w / 2, -h / 2), pt(w / 2, -h / 2), pt(w / 2, h / 2), pt(-w / 2, h / 2)];
}

const LEVELS = [
  {
    name: 'First Steps', mode: 'moves', w: 18, h: 12,
    start: { x: 2.5, y: 9.8, h: -Math.PI / 2 },
    goal: { cx: 14.2, cy: 2.8, w: 5.8, h: 2.6, heads: [0], tol: 12 },
    walls: [{ x: 7, y: 5, w: 2.6, h: 7 }],
    cars: [],
    starThresh: [2, 4],
    hint: 'Get around the block into the green zone.',
    solution: [{steer:22,dist:10.5},{steer:0,dist:3}],
    starThreshQuick: [8, 13],
  },
  {
    name: 'Parallel Squeeze', mode: 'moves', w: 22, h: 13,
    start: { x: 2.6, y: 7.0, h: 0 },
    goal: { cx: 10.15, cy: 9.4, w: 6.4, h: 2.2, heads: [0], tol: 12 },
    walls: [
      { x: 0, y: 10.5, w: 22, h: 2.5, kind: 'curb' },
      { x: 0, y: 0, w: 22, h: 1.6, kind: 'curb' },
    ],
    cars: [
      { cx: 4.6, cy: 9.5, h: 0 },
      { cx: 15.7, cy: 9.5, h: 0 },
      { cx: 9.5, cy: 2.55, h: Math.PI },
    ],
    starThresh: [3, 5],
    hint: 'Reverse into the gap at the curb.',
    solution: [{steer:0,dist:10.75},{steer:35,dist:-3},{steer:-35,dist:-3}],
    starThreshQuick: [12, 18],
  },
  {
    name: 'Tight Bay', mode: 'moves', w: 20, h: 13,
    start: { x: 16.5, y: 8.6, h: Math.PI },
    goal: { cx: 9.4, cy: 2.85, w: 2.7, h: 4.8, heads: [90, -90], tol: 10 },
    walls: [],
    cars: [2.4, 4.7, 7.0, 11.8, 14.1, 16.4].map(
      cx => ({ cx, cy: 2.85, h: Math.PI / 2 })),
    starThresh: [3, 5],
    hint: 'Back into the empty bay (either direction).',
    solution: [{steer:0,dist:10.6},{steer:35,dist:-6.3},{steer:0,dist:-2.9}],
    starThreshQuick: [13, 20],
  },
  {
    name: 'Dead End', mode: 'dist', w: 22, h: 13,
    start: { x: 3.2, y: 6.7, h: 0 },
    goal: { cx: 4.6, cy: 6.7, w: 8, h: 6.6, heads: [180], tol: 12 },
    walls: [
      { x: 0, y: 0, w: 22, h: 3.3 },
      { x: 0, y: 10.1, w: 22, h: 2.9 },
      { x: 18.4, y: 0, w: 3.6, h: 13 },
    ],
    cars: [],
    starThresh: [19, 27],
    hint: 'Turn around to face the way you came.',
    solution: [{steer:12,dist:3},{steer:-35,dist:3},{steer:35,dist:-3},{steer:-35,dist:1.5},{steer:0,dist:-1.5},{steer:-35,dist:5}],
    starThreshQuick: [20, 30],
  },
  {
    name: 'Battle Park', mode: 'moves', w: 24, h: 13,
    start: { x: 2.6, y: 7.0, h: 0 },
    goal: { cx: 9.29, cy: 9.22, w: 5.0, h: 2.0, heads: [0], tol: 10 },
    walls: [
      { x: 0, y: 10.5, w: 24, h: 2.5, kind: 'curb' },
      { x: 0, y: 0,    w: 24, h: 1.6, kind: 'curb' },
    ],
    cars: [
      { cx: 4.8,  cy: 9.4, h: 0 },
      { cx: 15.7, cy: 9.4, h: 0 },
      { cx: 9.5,  cy: 2.4, h: Math.PI },
      { cx: 21.0, cy: 9.4, h: 0 },
    ],
    starThresh: [3, 5], starThreshQuick: [14, 20],
    hint: 'Barely 9 cm from Car A — pure parallel precision.',
    solution: [
      { steer: 0,   dist: 10.75 },
      { steer: 35,  dist: -3 },
      { steer: -35, dist: -3 },
    ],
  },
];

function buildLevel(def) {
  const obstacles = [];
  const B = 0.45; // border wall thickness
  obstacles.push({ kind: 'border', poly: rectPoly(-B, -B, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', poly: rectPoly(-B, def.h, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', poly: rectPoly(-B, 0, B, def.h) });
  obstacles.push({ kind: 'border', poly: rectPoly(def.w, 0, B, def.h) });
  for (const r of def.walls) {
    obstacles.push({ kind: r.kind || 'wall', rect: r, poly: rectPoly(r.x, r.y, r.w, r.h) });
  }
  for (const c of def.cars) {
    obstacles.push({ kind: 'car', pose: c, poly: obbPoly(c.cx, c.cy, CAR.len, CAR.wid, c.h) });
  }
  return Object.assign({ obstacles }, def);
}

function inGoal(pose, goal) {
  const okHead = goal.heads.some(
    hd => Math.abs(normAng(pose.h - rad(hd))) <= rad(goal.tol));
  if (!okHead) return false;
  const x0 = goal.cx - goal.w / 2, x1 = goal.cx + goal.w / 2;
  const y0 = goal.cy - goal.h / 2, y1 = goal.cy + goal.h / 2;
  return carPoly(pose).every(
    v => v.x >= x0 && v.x <= x1 && v.y >= y0 && v.y <= y1);
}

const DRIVE_SPEED = 3.0;   // m/s
const STEER_RATE_DS = 60;  // degrees per second
const DIR_CHANGE_T = 1.5;  // seconds per direction reversal

function planTime(mvs) {
  let t = 0, prevDeg = 0, prevSign = 0;
  for (let i = 0; i < mvs.length; i++) {
    const m = mvs[i];
    const d = deg(m.steer);
    t += Math.abs(d - prevDeg) / STEER_RATE_DS;
    if (i > 0 && Math.sign(m.dist) !== prevSign) t += DIR_CHANGE_T;
    t += Math.abs(m.dist) / DRIVE_SPEED;
    prevDeg = d; prevSign = Math.sign(m.dist);
  }
  return t;
}

/* ===================== Game state ===================== */

const $ = id => document.getElementById(id);
const cv = $('cv'), ctx = cv.getContext('2d');

let levelIdx = clamp(parseInt(localStorage.getItem('parking.level') || '0', 10) || 0, 0, LEVELS.length - 1);
let level = buildLevel(LEVELS[levelIdx]);

let moves = [];        // [{steer (rad), dist (m)}]
let planSims = [];     // cached simulateMove result per move
let editSteer = 0;     // degrees, from slider
let editDist = 0;      // meters, signed
let editSim = null;
let editSimOpp = null; // preview for the opposite direction (same |dist|, opposite sign)
let editIdx = null;    // index of the move being tweaked (null = composing next move)

let anim = null;       // {samples, cum, total, t0, speed}
let scoringMode = 'precise'; // 'precise' | 'quick'
let view = { scale: 1, ox: 0, oy: 0 };

function planEnd() {
  return planSims.length ? planSims[planSims.length - 1].end : level.start;
}

function recomputePlan() {
  planSims = [];
  let pose = level.start;
  for (const m of moves) {
    const sim = simulateMove(pose, m.steer, m.dist, level.obstacles);
    planSims.push(sim);
    pose = sim.end;
  }
  recomputeEdit();
  updateHUD();
}

function recomputeEdit() {
  const startPose = editIdx !== null
    ? (editIdx === 0 ? level.start : planSims[editIdx - 1].end)
    : planEnd();
  if (Math.abs(editDist) > 0.01) {
    editSim    = simulateMove(startPose, rad(editSteer),  editDist, level.obstacles);
    editSimOpp = simulateMove(startPose, rad(editSteer), -editDist, level.obstacles);
  } else {
    editSim = editSimOpp = null;
  }
  $('addBtn').disabled = !editSim || !!editSim.hit || !!anim;
  $('addBtn').textContent = editIdx !== null ? `Update #${editIdx + 1}` : 'Add move';
}

function planStats() {
  let dist = 0;
  for (const m of moves) dist += Math.abs(m.dist);
  return { moves: moves.length, dist, time: planTime(moves) };
}

function computeStars(st) {
  if (scoringMode === 'quick') {
    const thresh = level.starThreshQuick || [999, 9999];
    return st.time <= thresh[0] ? 3 : st.time <= thresh[1] ? 2 : 1;
  }
  const v = level.mode === 'dist' ? st.dist : st.moves;
  const t = level.starThresh;
  return v <= t[0] ? 3 : v <= t[1] ? 2 : 1;
}

/* ===================== HUD ===================== */

function starStr(n, total = 3) {
  let s = '';
  for (let i = 0; i < total; i++) s += i < n ? '★' : '☆';
  return s;
}

function updateHUD() {
  $('lvName').textContent = `${levelIdx + 1}/${LEVELS.length} · ${level.name}`;
  $('objective').textContent =
    (level.mode === 'dist' ? 'Shortest distance wins' : 'Fewest moves wins') +
    ' · ' + level.hint;
  const st = planStats();
  const best = loadBest();
  const t = st.time.toFixed(1);
  const modeLabel = scoringMode === 'quick' ? `<span class="quick-badge">⏱ QUICK</span>` : '';
  $('stats').innerHTML =
    `${modeLabel}Moves <b>${st.moves}</b> · ${st.dist.toFixed(1)} m · ~${t}s` +
    (best ? ` · Best <span class="star">${starStr(best.stars)}</span> ` +
      (scoringMode === 'quick' ? `${(best.time||0).toFixed(1)}s` :
       level.mode === 'dist' ? `${best.dist.toFixed(1)} m` : `${best.moves} moves`) : '');
  $('modeBtn').textContent = scoringMode === 'precise' ? '★ Precise' : '⏱ Quick';
  $('undoBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('undoBtn').textContent = editIdx !== null ? 'Cancel' : 'Undo';
  $('resetBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('goBtn').disabled = (moves.length === 0 && (!editSim || !!editSim.hit)) || !!anim;
}

function loadBest() {
  try { return JSON.parse(localStorage.getItem(`parking.best.${scoringMode}.${levelIdx}`)); }
  catch (e) { return null; }
}

function saveBest(st, stars) {
  const prev = loadBest();
  const metric = scoringMode === 'quick' ? st.time :
    level.mode === 'dist' ? st.dist : st.moves;
  const prevMetric = prev ? (scoringMode === 'quick' ? (prev.time||999) :
    level.mode === 'dist' ? prev.dist : prev.moves) : Infinity;
  if (!prev || stars > prev.stars || (stars === prev.stars && metric < prevMetric)) {
    localStorage.setItem(`parking.best.${scoringMode}.${levelIdx}`,
      JSON.stringify({ moves: st.moves, dist: st.dist, time: st.time, stars }));
  }
}

let toastTimer = null;
function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.remove('hidden');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add('hidden'), 1800);
}

/* ===================== Rendering ===================== */

function fitView() {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
  }
  const m = 8; // px margin
  view.scale = Math.min((w - 2 * m) / level.w, (h - 2 * m) / level.h);
  view.ox = (w - level.w * view.scale) / 2;
  view.oy = (h - level.h * view.scale) / 2;
  view.dpr = dpr;
}

function worldTransform() {
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
  ctx.translate(view.ox, view.oy);
  ctx.scale(view.scale, view.scale);
}

function screenTransform() {
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
}

function toScreen(p) {
  return { x: view.ox + p.x * view.scale, y: view.oy + p.y * view.scale };
}

function drawPoly(poly) {
  ctx.beginPath();
  ctx.moveTo(poly[0].x, poly[0].y);
  for (let i = 1; i < poly.length; i++) ctx.lineTo(poly[i].x, poly[i].y);
  ctx.closePath();
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function drawCarBody(pose, opts) {
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  const x0 = -CAR.rOver, len = CAR.len, w = CAR.wid;

  if (opts.wheels) {
    ctx.fillStyle = '#10131a';
    for (const [wx, wy, a] of [
      [0, -0.74, 0], [0, 0.74, 0],
      [CAR.wb, -0.74, opts.steer || 0], [CAR.wb, 0.74, opts.steer || 0],
    ]) {
      ctx.save();
      ctx.translate(wx, wy);
      ctx.rotate(a);
      ctx.fillRect(-0.33, -0.13, 0.66, 0.26);
      ctx.restore();
    }
  }

  roundRect(x0, -w / 2, len, w, 0.3);
  ctx.fillStyle = opts.fill;
  ctx.fill();
  if (opts.stroke) {
    ctx.lineWidth = 0.07;
    ctx.strokeStyle = opts.stroke;
    ctx.stroke();
  }

  if (opts.detail) {
    // windshield + rear window
    ctx.fillStyle = 'rgba(8,12,18,0.45)';
    roundRect(CAR.wb * 0.42, -w / 2 + 0.22, 0.85, w - 0.44, 0.15);
    ctx.fill();
    roundRect(x0 + 0.45, -w / 2 + 0.25, 0.6, w - 0.5, 0.15);
    ctx.fill();
    // headlights
    ctx.fillStyle = '#ffe9a8';
    ctx.fillRect(x0 + len - 0.18, -w / 2 + 0.15, 0.12, 0.3);
    ctx.fillRect(x0 + len - 0.18, w / 2 - 0.45, 0.12, 0.3);
  }
  ctx.restore();
}

function drawGhost(pose, color, steer = 0) {
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  ctx.lineWidth = 0.07;
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  for (const [wx, wy, a] of [
    [0, -0.74, 0], [0, 0.74, 0],
    [CAR.wb, -0.74, steer], [CAR.wb, 0.74, steer],
  ]) {
    ctx.save();
    ctx.translate(wx, wy);
    ctx.rotate(a);
    ctx.strokeRect(-0.33, -0.13, 0.66, 0.26);
    ctx.restore();
  }
  ctx.restore();
  ctx.setLineDash([]);

  drawPoly(carPoly(pose));
  ctx.lineWidth = 0.07;
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  ctx.stroke();
  ctx.setLineDash([]);
  // heading notch at the nose
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const nx = pose.x + c * (CAR.wb + CAR.fOver), ny = pose.y + s * (CAR.wb + CAR.fOver);
  ctx.beginPath();
  ctx.moveTo(nx + c * 0.45, ny + s * 0.45);
  ctx.lineTo(nx - s * 0.3, ny + c * 0.3);
  ctx.lineTo(nx + s * 0.3, ny - c * 0.3);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function drawPath(pts, color, dashed) {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.lineWidth = 0.09;
  ctx.strokeStyle = color;
  if (dashed) ctx.setLineDash([0.35, 0.25]);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawArrow(x, y, ang, len, color) {
  const c = Math.cos(ang), s = Math.sin(ang);
  ctx.beginPath();
  ctx.moveTo(x - c * len / 2, y - s * len / 2);
  ctx.lineTo(x + c * len / 2, y + s * len / 2);
  ctx.lineWidth = 0.12;
  ctx.strokeStyle = color;
  ctx.stroke();
  const tx = x + c * len / 2, ty = y + s * len / 2;
  ctx.beginPath();
  ctx.moveTo(tx + c * 0.45, ty + s * 0.45);
  ctx.lineTo(tx - s * 0.28, ty + c * 0.28);
  ctx.lineTo(tx + s * 0.28, ty - c * 0.28);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function draw(now) {
  fitView();
  screenTransform();
  ctx.fillStyle = '#171a21';
  ctx.fillRect(0, 0, cv.clientWidth, cv.clientHeight);

  worldTransform();

  // asphalt
  ctx.fillStyle = '#23272f';
  ctx.fillRect(0, 0, level.w, level.h);
  // 1 m grid
  ctx.lineWidth = 0.02;
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.beginPath();
  for (let x = 1; x < level.w; x++) { ctx.moveTo(x, 0); ctx.lineTo(x, level.h); }
  for (let y = 1; y < level.h; y++) { ctx.moveTo(0, y); ctx.lineTo(level.w, y); }
  ctx.stroke();

  // goal zone
  const g = level.goal;
  ctx.fillStyle = 'rgba(61,220,132,0.10)';
  ctx.fillRect(g.cx - g.w / 2, g.cy - g.h / 2, g.w, g.h);
  ctx.lineWidth = 0.1;
  ctx.strokeStyle = '#3ddc84';
  ctx.setLineDash([0.45, 0.3]);
  ctx.strokeRect(g.cx - g.w / 2, g.cy - g.h / 2, g.w, g.h);
  ctx.setLineDash([]);
  for (const hd of g.heads) {
    drawArrow(g.cx, g.cy, rad(hd), Math.min(g.w, g.h) * 0.45, 'rgba(61,220,132,0.7)');
  }

  // obstacles
  for (const o of level.obstacles) {
    if (o.kind === 'car') {
      drawCarBody({ x: o.pose.cx - Math.cos(o.pose.h) * (CAR.len / 2 - CAR.rOver),
                    y: o.pose.cy - Math.sin(o.pose.h) * (CAR.len / 2 - CAR.rOver),
                    h: o.pose.h },
                  { fill: '#737d8c', stroke: '#525a66', detail: true, wheels: true });
    } else {
      drawPoly(o.poly);
      ctx.fillStyle = o.kind === 'curb' ? '#3a4148' : '#39404e';
      ctx.fill();
      ctx.lineWidth = 0.06;
      ctx.strokeStyle = o.kind === 'curb' ? '#4d565e' : '#4a5568';
      ctx.stroke();
    }
  }

  // start pad
  drawPoly(carPoly(level.start));
  ctx.lineWidth = 0.05;
  ctx.strokeStyle = 'rgba(255,255,255,0.22)';
  ctx.setLineDash([0.2, 0.2]);
  ctx.stroke();
  ctx.setLineDash([]);

  // committed plan: paths + ghosts
  // When editing move editIdx: skip that move's arc (replaced by live preview),
  // and dim any moves that come after it (their start poses will shift on commit).
  for (let i = 0; i < planSims.length; i++) {
    if (editIdx !== null && i === editIdx) continue;
    const sim = planSims[i];
    const dimmed = editIdx !== null && i > editIdx;
    ctx.save();
    if (dimmed) ctx.globalAlpha = 0.3;
    drawPath(sim.pts, moves[i].dist >= 0 ? 'rgba(69,196,255,0.85)' : 'rgba(255,159,67,0.85)',
             moves[i].dist < 0);
    ctx.restore();
  }
  for (let i = 0; i < planSims.length; i++) {
    if (editIdx !== null && i === editIdx) continue;
    const dimmed = editIdx !== null && i > editIdx;
    const isAnchor = editIdx !== null ? i === editIdx - 1 : i === planSims.length - 1 && !editSim;
    ctx.save();
    if (dimmed) ctx.globalAlpha = 0.3;
    drawGhost(planSims[i].end, isAnchor
      ? 'rgba(233,240,250,0.85)' : 'rgba(160,175,195,0.5)', moves[i].steer);
    ctx.restore();
  }

  // opposite-direction ghost (same |dist|, opposite sign) — shown dim so the
  // player can see both options at once without it competing with the active arc
  if (editSimOpp) {
    const oppFwd = editDist < 0; // opposite of current direction
    ctx.save();
    ctx.globalAlpha = 0.32;
    drawPath(editSimOpp.pts,
             oppFwd ? 'rgba(69,196,255,0.9)' : 'rgba(255,159,67,0.9)', !oppFwd);
    drawGhost(editSimOpp.end,
              oppFwd ? 'rgba(69,196,255,0.9)' : 'rgba(255,159,67,0.9)', rad(editSteer));
    ctx.restore();
  }

  // live edit preview (active direction)
  let hitInfo = null;
  if (editSim) {
    const bad = !!editSim.hit;
    drawPath(editSim.pts,
             bad ? 'rgba(255,82,82,0.9)'
                 : editDist >= 0 ? 'rgba(69,196,255,0.95)' : 'rgba(255,159,67,0.95)',
             editDist < 0);
    drawGhost(bad ? editSim.hit.pose : editSim.end,
              bad ? 'rgba(255,82,82,0.95)' : 'rgba(233,240,250,0.95)', rad(editSteer));
    if (bad) hitInfo = editSim.hit;
  }

  // collision marker (pulsing)
  if (hitInfo) {
    const t = (now % 900) / 900;
    const p = hitInfo.point;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 0.25 + t * 0.6, 0, 2 * Math.PI);
    ctx.lineWidth = 0.09;
    ctx.strokeStyle = `rgba(255,82,82,${1 - t})`;
    ctx.stroke();
    ctx.beginPath();
    for (let i = 0; i < 8; i++) {
      const a = i * Math.PI / 4, r1 = 0.14, r2 = 0.34;
      ctx.moveTo(p.x + Math.cos(a) * r1, p.y + Math.sin(a) * r1);
      ctx.lineTo(p.x + Math.cos(a) * r2, p.y + Math.sin(a) * r2);
    }
    ctx.lineWidth = 0.07;
    ctx.strokeStyle = '#ff5252';
    ctx.stroke();
  }

  // the car: animated along the plan, or sitting at start
  let carPose = level.start, carSteer = rad(editSteer);
  if (anim) {
    const trav = Math.min(anim.total, (now - anim.t0) / 1000 * anim.speed);
    const i = sampleAt(anim, trav);
    carPose = anim.samples[i].pose;
    carSteer = anim.samples[i].steer;
    if (trav >= anim.total) finishRun();
  }
  drawCarBody(carPose, { fill: '#4fc3f7', stroke: '#1c5f80', detail: true,
                         wheels: true, steer: carSteer });

  // move numbers (screen space so text stays crisp)
  screenTransform();
  ctx.font = '700 12px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let i = 0; i < planSims.length; i++) {
    const e = planSims[i].end;
    const sp = toScreen({ x: e.x + Math.cos(e.h) * CAR.wb / 2,
                          y: e.y + Math.sin(e.h) * CAR.wb / 2 });
    const selected = editIdx === i;
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 9, 0, 2 * Math.PI);
    ctx.fillStyle = selected ? '#ffd700' : 'rgba(20,25,33,0.85)';
    ctx.fill();
    ctx.fillStyle = selected ? '#1a1400' : '#cfd9e6';
    ctx.fillText(String(i + 1), sp.x, sp.y + 0.5);
  }

  requestAnimationFrame(draw);
}

/* ===================== Run / animate ===================== */

function startRun() {
  if (!moves.length || anim) return;
  editIdx = null;
  const samples = [];
  const cum = [];
  let total = 0;
  for (let i = 0; i < planSims.length; i++) {
    const sim = planSims[i];
    const step = Math.abs(moves[i].dist) / (sim.pts.length - 1);
    for (let j = (i === 0 ? 0 : 1); j < sim.pts.length; j++) {
      if (j > 0) total += step;
      samples.push({ pose: sim.pts[j], steer: moves[i].steer });
      cum.push(total);
    }
  }
  const speed = clamp(total / 3, 2.5, 7); // whole run in ~3 s
  anim = { samples, cum, total, t0: performance.now(), speed };
  updateHUD();
}

function sampleAt(a, trav) {
  let lo = 0, hi = a.cum.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (a.cum[mid] < trav) lo = mid + 1; else hi = mid;
  }
  return lo;
}

function finishRun() {
  anim = null;
  updateHUD();
  const end = planEnd();
  if (inGoal(end, level.goal)) {
    const st = planStats();
    const stars = computeStars(st);
    saveBest(st, stars);
    $('ovTitle').textContent = 'Parked!';
    $('ovStars').innerHTML =
      starStr(stars).replace(/☆/g, '<span class="dim">★</span>');
    $('ovStats').textContent = `${st.moves} moves · ${st.dist.toFixed(1)} m`;
    $('ovTip').textContent = stars === 3 ? 'Perfect!' :
      (level.mode === 'dist'
        ? `3★ ≤ ${level.starThresh[0]} m`
        : `3★ ≤ ${level.starThresh[0]} moves`);
    $('ovNext').style.display = levelIdx < LEVELS.length - 1 ? '' : 'none';
    $('overlay').classList.remove('hidden');
  } else {
    toast('Not parked yet — keep planning');
  }
}

/* ===================== Level switching ===================== */

function setLevel(i) {
  levelIdx = (i + LEVELS.length) % LEVELS.length;
  localStorage.setItem('parking.level', String(levelIdx));
  level = buildLevel(LEVELS[levelIdx]);
  moves = [];
  anim = null;
  editIdx = null;
  setEdit(0, 0);
  recomputePlan();
}

/* ===================== Input ===================== */

const steerEl = $('steer'), distEl = $('dist');

function setEdit(steerDeg, dist) {
  editSteer = clamp(Math.abs(steerDeg) <= 2 ? 0 : steerDeg, -CAR.maxSteer, CAR.maxSteer);
  editDist = Math.abs(dist) < 0.15 ? 0 : clamp(dist, -MAX_MOVE_DIST, MAX_MOVE_DIST);
  steerEl.value = editSteer;
  distEl.value = editDist;
  $('steerVal').textContent = editSteer === 0 ? '0°'
    : `${Math.abs(editSteer)}° ${editSteer < 0 ? 'left' : 'right'}`;
  $('distVal').textContent = editDist === 0 ? '—'
    : `${editDist < 0 ? 'Rev' : 'Fwd'} ${Math.abs(editDist).toFixed(1)} m`;
  recomputeEdit();
  updateHUD();
}

steerEl.addEventListener('input', () => setEdit(parseFloat(steerEl.value), editDist));
distEl.addEventListener('input', () => setEdit(editSteer, parseFloat(distEl.value)));

$('addBtn').addEventListener('click', () => {
  if (!editSim || editSim.hit || anim) return;
  if (editIdx !== null) {
    moves[editIdx] = { steer: rad(editSteer), dist: editDist };
    editIdx = null;
    setEdit(editSteer, 0);
    recomputePlan();
  } else {
    moves.push({ steer: rad(editSteer), dist: editDist });
    setEdit(editSteer, 0); // keep steering, ready for the next move
    recomputePlan();
  }
});

$('undoBtn').addEventListener('click', () => {
  if (anim) return;
  if (editIdx !== null) {
    editIdx = null;
    setEdit(0, 0);
    return;
  }
  if (Math.abs(editDist) >= 0.01) {
    setEdit(editSteer, 0);
  } else if (moves.length) {
    const m = moves.pop();
    setEdit(deg(m.steer), 0);
    recomputePlan();
  }
});

$('resetBtn').addEventListener('click', () => {
  if (anim) return;
  editIdx = null;
  moves = [];
  setEdit(0, 0);
  recomputePlan();
});

$('goBtn').addEventListener('click', () => {
  if (anim) return;
  // Auto-commit a valid pending move so the player doesn't have to press Add first
  if (editSim && !editSim.hit && editIdx === null) {
    moves.push({ steer: rad(editSteer), dist: editDist });
    editIdx = null;
    recomputePlan();
  }
  if (!moves.length) { toast('Add some moves first'); return; }
  startRun();
});

$('prevLv').addEventListener('click', () => setLevel(levelIdx - 1));
$('nextLv').addEventListener('click', () => setLevel(levelIdx + 1));

$('solBtn').addEventListener('click', showSolution);
$('modeBtn').addEventListener('click', toggleMode);
$('helpBtn').addEventListener('click', () => $('helpOverlay').classList.remove('hidden'));
$('helpClose').addEventListener('click', () => $('helpOverlay').classList.add('hidden'));

$('ovImprove').addEventListener('click', () => $('overlay').classList.add('hidden'));
$('ovRetry').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  moves = [];
  setEdit(0, 0);
  recomputePlan();
});
$('ovNext').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  setLevel(levelIdx + 1);
});

function selectMove(i) {
  if (anim) return;
  if (editIdx === i) { editIdx = null; setEdit(0, 0); return; }
  editIdx = i;
  const m = moves[i];
  setEdit(deg(m.steer), m.dist);
}

function showSolution() {
  if (!level.solution) { toast('No solution on record for this level'); return; }
  if (anim) return;
  editIdx = null;
  moves = level.solution.map(m => ({ steer: rad(m.steer), dist: m.dist }));
  setEdit(0, 0);
  recomputePlan();
  toast('Optimal solution loaded — hit Run to see it');
}

function toggleMode() {
  scoringMode = scoringMode === 'precise' ? 'quick' : 'precise';
  updateHUD();
}

// Drag directly on the canvas: vertical = distance, horizontal = steering.
// A tap (minimal movement) on a move badge selects it for tweaking.
let drag = null;
cv.addEventListener('pointerdown', e => {
  if (anim) return;
  cv.setPointerCapture(e.pointerId);
  drag = { x: e.clientX, y: e.clientY, steer: editSteer, dist: editDist, moved: false };
});
cv.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.abs(dx) > 5 || Math.abs(dy) > 5) drag.moved = true;
  if (!drag.moved) return;
  const steer = drag.steer + dx / (cv.clientWidth * 0.45) * CAR.maxSteer * 2;
  const dist = drag.dist - dy / view.scale * 1.25;
  setEdit(steer, dist);
});
cv.addEventListener('pointerup', e => {
  if (drag && !drag.moved) {
    const cx = e.clientX, cy = e.clientY;
    let hit = -1;
    for (let i = 0; i < planSims.length; i++) {
      const ep = planSims[i].end;
      const sp = toScreen({ x: ep.x + Math.cos(ep.h) * CAR.wb / 2,
                             y: ep.y + Math.sin(ep.h) * CAR.wb / 2 });
      if (Math.hypot(cx - sp.x, cy - sp.y) < 16) { hit = i; break; }
    }
    if (hit >= 0) selectMove(hit);
    else if (editIdx !== null) { editIdx = null; setEdit(0, 0); }
  }
  drag = null;
});
cv.addEventListener('pointercancel', () => { drag = null; });

document.addEventListener('gesturestart', e => e.preventDefault());

/* ===================== Boot ===================== */

setLevel(levelIdx);
requestAnimationFrame(draw);
