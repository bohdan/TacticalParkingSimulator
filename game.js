'use strict';

/* ===================== Car & math ===================== */

// All world units are meters. Pose = {x, y, h}; (x,y) is the REAR AXLE
// center, h is heading in radians (0 = +x, y axis points down on screen).
const CAR = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, maxSteer: 35 };
CAR.fOver = CAR.len - CAR.wb - CAR.rOver;

// Fixed sedan dimensions used for static obstacle cars regardless of player vehicle
const SEDAN = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, fOver: 0.85, maxSteer: 35 };

const VEHICLES = {
  default: { len: 4.4,  wid: 1.8,  wb: 2.7,  rOver: 0.85, maxSteer: 35 },
  miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
  bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
};

function setVehicle(name) {
  const v = VEHICLES[name] || VEHICLES.default;
  CAR.len = v.len; CAR.wid = v.wid; CAR.wb = v.wb;
  CAR.rOver = v.rOver; CAR.maxSteer = v.maxSteer;
  CAR.fOver = CAR.len - CAR.wb - CAR.rOver;
}

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

function ptSegDist(px, py, ax, ay, bx, by) {
  const dx = bx-ax, dy = by-ay, l2 = dx*dx+dy*dy;
  if (!l2) return Math.hypot(px-ax, py-ay);
  const t = Math.max(0, Math.min(1, ((px-ax)*dx+(py-ay)*dy)/l2));
  return Math.hypot(px-ax-t*dx, py-ay-t*dy);
}

function goalPoly(g) {
  return g.ang
    ? obbPoly(g.cx, g.cy, g.w, g.h, g.ang)
    : [{ x: g.cx-g.w/2, y: g.cy-g.h/2 }, { x: g.cx+g.w/2, y: g.cy-g.h/2 },
       { x: g.cx+g.w/2, y: g.cy+g.h/2 }, { x: g.cx-g.w/2, y: g.cy+g.h/2 }];
}

// Min distance from any car corner to the goal zone boundary.
// Maximised when the car is centred in the spot.
function parkingClearance(pose) {
  const cp = carPoly(pose);
  const zone = goalPoly(level.goal);
  let minGap = Infinity;
  for (const v of cp)
    for (let j = 0; j < zone.length; j++) {
      const a = zone[j], b = zone[(j+1) % zone.length];
      minGap = Math.min(minGap, ptSegDist(v.x, v.y, a.x, a.y, b.x, b.y));
    }
  return isFinite(minGap) ? minGap : 0;
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

// Editor test level: passed via URL hash (#try=<base64url>) by editor.html.
// Scoped to this tab/URL only, so it never reorders the real level list.
let testLevelLoaded = false;
(()=>{
  // Clean up the old localStorage mechanism, which permanently reordered
  // levels (the ★ test level got prepended on every load until cleared).
  try { localStorage.removeItem('parkplanner_testlevel'); } catch (e) {}
  const m = location.hash.match(/[#&]try=([A-Za-z0-9\-_]+)/);
  if (!m) return;
  try {
    let s = m[1].replace(/-/g, '+').replace(/_/g, '/');
    while (s.length % 4) s += '=';
    const lv = JSON.parse(new TextDecoder().decode(Uint8Array.from(atob(s), c => c.charCodeAt(0))));
    lv.name = '★ ' + lv.name;
    lv._isTest = true;
    LEVELS.unshift(lv);
    testLevelLoaded = true;
  } catch (e) { /* ignore malformed data */ }
})();

function buildLevel(def) {
  const obstacles = [];
  const B = 0.45; // border wall thickness
  obstacles.push({ kind: 'border', poly: rectPoly(-B, -B, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', poly: rectPoly(-B, def.h, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', poly: rectPoly(-B, 0, B, def.h) });
  obstacles.push({ kind: 'border', poly: rectPoly(def.w, 0, B, def.h) });
  for (const r of def.walls) {
    const poly = r.ang != null
      ? obbPoly(r.cx, r.cy, r.w, r.h, r.ang)
      : rectPoly(r.x, r.y, r.w, r.h);
    obstacles.push({ kind: r.kind || 'wall', rect: r, poly });
  }
  for (const c of def.cars) {
    const sp = (c.type && VEHICLES[c.type])
      ? { ...VEHICLES[c.type], fOver: VEHICLES[c.type].len - VEHICLES[c.type].wb - VEHICLES[c.type].rOver }
      : SEDAN;
    obstacles.push({ kind: 'car', pose: c, carSpec: sp, poly: obbPoly(c.cx, c.cy, sp.len, sp.wid, c.h) });
  }
  return Object.assign({ obstacles }, def);
}

function inGoal(pose, goal) {
  const okHead = goal.heads.some(
    hd => Math.abs(normAng(pose.h - rad(hd))) <= rad(goal.tol));
  if (!okHead) return false;
  const poly = goalPoly(goal);
  return carPoly(pose).every(v => pointInPoly(v, poly));
}

// ── Leaderboard (Supabase) ─────────────────────────────────────────────────
// 1. Go to https://supabase.com and sign in with GitHub (no new account needed)
// 2. Create a project, open the SQL editor, and run:
//      create table leaderboard (
//        id bigserial primary key, player text not null,
//        level int not null, level_name text not null,
//        stars int not null, moves int, dist real, time_s real,
//        mode text not null, submitted_at timestamptz default now()
//      );
//      alter table leaderboard enable row level security;
//      create policy "public read"   on leaderboard for select using (true);
//      create policy "public insert" on leaderboard for insert
//        with check (char_length(player) between 1 and 20);
// 3. Fill in Project Settings → API → Project URL and anon/public key below.
const LB_URL = 'https://qvjorkpzlwvswsptkwyn.supabase.co';
const LB_KEY = 'sb_publishable_geHaaCkSfPilYWV3fYQHQA_KZdYNrpC';

const V_MAX = 3.0;         // m/s top speed
const ACCEL = 2.0;         // m/s² acceleration / braking
const STEER_RATE_DS = 60;  // degrees per second
const DIR_CHANGE_T = 1.5;  // seconds per direction reversal

// Time for one move: trapezoid profile (accel from rest → cruise → brake to rest)
function moveTime(dist) {
  const d = Math.abs(dist);
  const dFull = V_MAX * V_MAX / ACCEL; // dist needed to reach V_MAX and brake
  if (d >= dFull) return 2 * V_MAX / ACCEL + (d - dFull) / V_MAX;
  return 2 * Math.sqrt(d / ACCEL);
}

function planTime(mvs) {
  let t = 0, prevDeg = 0, prevSign = 0;
  for (let i = 0; i < mvs.length; i++) {
    const m = mvs[i];
    const d = deg(m.steer);
    t += Math.abs(d - prevDeg) / STEER_RATE_DS;
    if (i > 0 && Math.sign(m.dist) !== prevSign) t += DIR_CHANGE_T;
    t += moveTime(m.dist);
    prevDeg = d; prevSign = Math.sign(m.dist);
  }
  return t;
}

/* ===================== Game state ===================== */

const $ = id => document.getElementById(id);
const cv = $('cv'), ctx = cv.getContext('2d');

// A "cutscene" level isn't a parking puzzle — it plays a briefing animation.
const isCutscene = def => !!def && def.type === 'cutscene';

let levelIdx = testLevelLoaded
  ? 0  // start on the ★ test level when one was passed in
  : clamp(parseInt(localStorage.getItem('parking.level') || '0', 10) || 0, 0, LEVELS.length - 1);
let level = isCutscene(LEVELS[levelIdx]) ? null : buildLevel(LEVELS[levelIdx]);

let moves = [];        // [{steer (rad), dist (m)}]
let planSims = [];     // cached simulateMove result per move
let editSteer = 0;     // degrees, from slider
let editDist = 0;      // meters, signed
let distMin = 0, distMax = 0; // drivable range at current steer/start pose
let editSim = null;
let editSimOpp = null; // preview for the opposite direction (same |dist|, opposite sign)
let editIdx = null;    // index of the move being tweaked (null = composing next move)

let anim = null;       // {samples, cum, total, t0, speed}
let pendingLb = null;  // {levelIdx, stars, st} — awaiting leaderboard submit
let solutionUsed = false; // viewing the solution locks leaderboard until Reset
let view = { scale: 1, ox: 0, oy: 0 };

function planEnd() {
  return planSims.length ? planSims[planSims.length - 1].end : level.start;
}

// How far the car can roll from `pose` before hitting something — at most
// one full circle when steering, the field span when straight. Rounded down
// to the slider step so 0 stays reachable on the range input.
function driveLimit(pose, steer, dir) {
  const R = Math.abs(steer) < 1e-4 ? Infinity : Math.abs(CAR.wb / Math.tan(steer));
  const cap = Math.min(level.w + level.h, 2 * Math.PI * R);
  const n = Math.max(2, Math.ceil(cap / SAMPLE_STEP));
  const sim = simulateMove(pose, steer, dir * cap, level.obstacles);
  return Math.floor((sim.pts.length - 1) / n * cap * 10) / 10;
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
  const s = rad(editSteer);
  // Fixed symmetric range so the distance value is preserved when the player
  // adjusts the steering angle. Collision is shown via yellow highlight instead.
  const fieldRange = Math.max(40, Math.ceil(Math.hypot(level.w, level.h)));
  distMax = fieldRange; distMin = -fieldRange;
  distEl.min = distMin; distEl.max = distMax; distEl.value = editDist;
  distEl.style.setProperty('--zero', '50%');
  if (Math.abs(editDist) > 0.01) {
    editSim    = simulateMove(startPose, s,  editDist, level.obstacles);
    editSimOpp = simulateMove(startPose, s, -editDist, level.obstacles);
  } else {
    editSim = editSimOpp = null;
  }
  const hit = !!(editSim?.hit);
  distEl.classList.toggle('hit', hit);
  $('distVal').textContent = editDist === 0 ? '—'
    : `${editDist < 0 ? 'Rev' : 'Fwd'} ${Math.abs(editDist).toFixed(1)} m${hit ? ' ⚠' : ''}`;
  $('addBtn').disabled = !editSim || editSim.pts.length < 2 || !!anim;
  $('addBtn').textContent = editIdx !== null ? `Update #${editIdx + 1}` : 'Add move';
}

function planStats() {
  let dist = 0;
  for (const m of moves) dist += Math.abs(m.dist);
  return { moves: moves.length, dist, time: planTime(moves) };
}

function computeStars(st) {
  const thresh = level.starThreshQuick || [999, 9999];
  return st.time <= thresh[0] ? 3 : st.time <= thresh[1] ? 2 : 1;
}

/* ===================== HUD ===================== */

function starStr(n, total = 3) {
  let s = '';
  for (let i = 0; i < total; i++) s += i < n ? '★' : '☆';
  return s;
}

function updateHUD() {
  $('lvName').textContent = `${levelIdx + 1}/${LEVELS.length} · ${level.name}`;
  $('objective').textContent = `${level.tier} · Quick time`;
  const planning = moves.length > 0 || Math.abs(editDist) > 0.01;
  const best = loadBest();
  if (planning) {
    const st = planStats();
    $('stats').innerHTML =
      `Moves <b>${st.moves}</b> · ${st.dist.toFixed(1)} m · ~${st.time.toFixed(1)}s` +
      (best ? ` · Best <span class="star">${starStr(best.stars)}</span>` : '');
  } else {
    $('stats').innerHTML = escHtml(level.tut || level.hint) +
      (best ? ` · <span class="star">${starStr(best.stars)}</span>` : '');
  }
  $('undoBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('undoBtn').textContent = editIdx !== null ? 'Cancel' : '↶ Undo';
  $('resetBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('goBtn').disabled = (moves.length === 0 && (!editSim || editSim.pts.length < 2)) || !!anim;
}

function loadBest() {
  try { return JSON.parse(localStorage.getItem(`parking.best.${levelIdx}`)); }
  catch (e) { return null; }
}

function saveBest(st, stars) {
  const prev = loadBest();
  const prevTime = prev ? (prev.time || 999) : Infinity;
  if (!prev || stars > prev.stars || (stars === prev.stars && st.time < prevTime)) {
    localStorage.setItem(`parking.best.${levelIdx}`,
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

function drawCarBody(pose, opts, spec) {
  spec = spec || CAR;
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  const x0 = -spec.rOver, len = spec.len, w = spec.wid;
  // wheel offset from car centerline (inset by ~0.16 m regardless of vehicle width)
  const wy = w / 2 - 0.16;
  // wheel box scales with vehicle length; bus wheels are larger
  const wl = Math.min(0.9, len * 0.075), wt = Math.min(0.18, w * 0.10);

  if (opts.wheels) {
    ctx.fillStyle = '#10131a';
    for (const [wx, wya, a] of [
      [0, -wy, 0], [0, wy, 0],
      [spec.wb, -wy, opts.steer || 0], [spec.wb, wy, opts.steer || 0],
    ]) {
      ctx.save();
      ctx.translate(wx, wya);
      ctx.rotate(a);
      ctx.fillRect(-wl / 2, -wt, wl, wt * 2);
      ctx.restore();
    }
  }

  roundRect(x0, -w / 2, len, w, Math.min(0.3, w * 0.17));
  ctx.fillStyle = opts.fill;
  ctx.fill();
  if (opts.stroke) {
    ctx.lineWidth = 0.07;
    ctx.strokeStyle = opts.stroke;
    ctx.stroke();
  }

  if (opts.detail) {
    // windshield and rear window, scaled to vehicle length
    const wsX = x0 + len * 0.30;  // windshield x
    const rwX = x0 + len * 0.09;  // rear window x
    const glH = w - 0.44;
    ctx.fillStyle = 'rgba(8,12,18,0.45)';
    roundRect(wsX, -w / 2 + 0.22, Math.min(0.85, len * 0.20), glH, 0.15);
    ctx.fill();
    roundRect(rwX, -w / 2 + 0.25, Math.min(0.6, len * 0.13), glH, 0.15);
    ctx.fill();
    // headlights
    ctx.fillStyle = '#ffe9a8';
    ctx.fillRect(x0 + len - 0.18, -w / 2 + 0.15, 0.12, Math.min(0.3, w * 0.17));
    ctx.fillRect(x0 + len - 0.18,  w / 2 - 0.45, 0.12, Math.min(0.3, w * 0.17));
  }
  ctx.restore();
}

function drawGhost(pose, color, steer = 0) {
  const wy = CAR.wid / 2 - 0.13;
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  ctx.lineWidth = 0.07;
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  for (const [wx, wya, a] of [
    [0, -wy, 0], [0, wy, 0],
    [CAR.wb, -wy, steer], [CAR.wb, wy, steer],
  ]) {
    ctx.save();
    ctx.translate(wx, wya);
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

function drawPath(pts, color, dashed, lw = 0.09) {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.lineWidth = lw;
  ctx.strokeStyle = color;
  if (dashed) ctx.setLineDash([0.35, 0.25]);
  ctx.stroke();
  ctx.setLineDash([]);
}

// Arc guides: always shown when wheels are turned and no animation running.
// Four solid curves = all 4 bounding-box corners swept path.
// Four dashed curves = all 4 wheel tracks (front-left, front-right, rear-left, rear-right).
function drawArcGuides(pose, steerRad) {
  const N = 60;
  const fwdLimit = driveLimit(pose, steerRad, 1);
  const bwdLimit = driveLimit(pose, steerRad, -1);
  const fLen = CAR.wb + CAR.fOver;
  const half = CAR.wid / 2;

  function sampleArc(limit, dir) {
    const cFL = [], cFR = [], cRL = [], cRR = [];
    const wFL = [], wFR = [], wRL = [], wRR = [];
    for (let i = 0; i <= N; i++) {
      const p = advance(pose, steerRad, dir * limit * i / N);
      const cs = Math.cos(p.h), sn = Math.sin(p.h);
      const w = (lx, ly) => ({ x: p.x + cs * lx - sn * ly, y: p.y + sn * lx + cs * ly });
      cFL.push(w(fLen,        half));
      cFR.push(w(fLen,       -half));
      cRL.push(w(-CAR.rOver,  half));
      cRR.push(w(-CAR.rOver, -half));
      wFL.push(w(CAR.wb,  half));
      wFR.push(w(CAR.wb, -half));
      wRL.push(w(0,        half));
      wRR.push(w(0,       -half));
    }
    return { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR };
  }

  for (const [limit, dir] of [[fwdLimit, 1], [bwdLimit, -1]]) {
    if (limit < 0.2) continue;
    const { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR } = sampleArc(limit, dir);
    const col  = dir > 0 ? 'rgba(69,196,255,0.28)' : 'rgba(255,159,67,0.28)';
    const wCol = dir > 0 ? 'rgba(69,196,255,0.50)' : 'rgba(255,159,67,0.50)';
    drawPath(cFL, col, false, 0.06);
    drawPath(cFR, col, false, 0.06);
    drawPath(cRL, col, false, 0.06);
    drawPath(cRR, col, false, 0.06);
    drawPath(wFL, wCol, true, 0.05);
    drawPath(wFR, wCol, true, 0.05);
    drawPath(wRL, wCol, true, 0.05);
    drawPath(wRR, wCol, true, 0.05);
  }
}

// Steering geometry overlay: the rear-axle axis (perpendicular to heading),
// the instantaneous turn centre sitting on it, and the radius lines from the
// rear axle and both front wheels to that centre. Each front wheel rolls
// perpendicular to its own radius line (classic Ackermann); the rear-axle →
// centre segment is the turn radius R = wheelbase / tan(steer).
function drawSteerGeometry(pose, steerRad) {
  if (Math.abs(steerRad) < rad(0.5)) return;   // ~straight: centre at infinity
  const R = CAR.wb / Math.tan(steerRad);
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const ux = -s, uy = c;                       // rear-axle axis direction (toward centre)
  const O = { x: pose.x + R * ux, y: pose.y + R * uy };
  const sgn = Math.sign(R);
  const at = t => ({ x: pose.x + t * ux, y: pose.y + t * uy });

  // rear-axle axis — thin, extends just past the centre and the opposite side
  drawPath([at(-sgn * 1.0), at(R + sgn * 1.0)], 'rgba(255,255,255,0.28)', false, 0.035);

  // radius lines from each front wheel to the turn centre
  const half = CAR.wid / 2 - 0.16;             // matches drawn wheel inset
  const fw = ly => ({ x: pose.x + CAR.wb * c - ly * s, y: pose.y + CAR.wb * s + ly * c });
  drawPath([fw(half),  O], 'rgba(255,255,255,0.28)', false, 0.03);
  drawPath([fw(-half), O], 'rgba(255,255,255,0.28)', false, 0.03);

  // the turn radius itself: rear-axle centre → turn centre
  drawPath([{ x: pose.x, y: pose.y }, O], 'rgba(120,220,255,0.8)', false, 0.045);

  // turn-centre marker
  ctx.beginPath();
  ctx.arc(O.x, O.y, 0.12, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(120,220,255,0.9)';
  ctx.fill();
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
  // On a cutscene there's no puzzle to render — the briefing overlay covers
  // the screen. Keep the RAF alive so play resumes when we leave it.
  if (!level) { requestAnimationFrame(draw); return; }
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
  const gPoly = goalPoly(g);
  drawPoly(gPoly);
  ctx.fillStyle = 'rgba(61,220,132,0.10)';
  ctx.fill();
  ctx.lineWidth = 0.1;
  ctx.strokeStyle = '#3ddc84';
  ctx.setLineDash([0.45, 0.3]);
  ctx.stroke();
  ctx.setLineDash([]);
  for (const hd of g.heads) {
    drawArrow(g.cx, g.cy, rad(hd), Math.min(g.w, g.h) * 0.45, 'rgba(61,220,132,0.7)');
  }

  // decorative traffic (non-collision, animated sedans outside parking zone)
  if (level.traffic) {
    const tSec = now / 1000;
    for (const tr of level.traffic) {
      const d = (tSec * tr.speed + tr.offset) % tr.loop;
      const tx = tr.x + Math.cos(tr.h) * d;
      const ty = tr.y + Math.sin(tr.h) * d;
      const rearX = tx - Math.cos(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      const rearY = ty - Math.sin(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      drawCarBody({ x: rearX, y: rearY, h: tr.h },
                  { fill: tr.color || '#4e5a6e', stroke: '#3a4255', detail: false, wheels: false },
                  SEDAN);
    }
  }

  // obstacles
  for (const o of level.obstacles) {
    if (o.kind === 'car') {
      const sp = o.carSpec || SEDAN;
      drawCarBody({ x: o.pose.cx - Math.cos(o.pose.h) * (sp.len / 2 - sp.rOver),
                    y: o.pose.cy - Math.sin(o.pose.h) * (sp.len / 2 - sp.rOver),
                    h: o.pose.h },
                  { fill: '#737d8c', stroke: '#525a66', detail: true, wheels: true }, sp);
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

  // arc guides (all 4 corners + all 4 wheels) — drawn first so they sit behind everything
  if (!anim) {
    drawArcGuides(editStartPose(), rad(editSteer));
    drawSteerGeometry(editStartPose(), rad(editSteer));
  }

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
    const best = loadBest();
    const prec = parkingClearance(end);
    const precCm = Math.round(prec * 100);
    // Thresholds calibrated to real geometry: a 2.2 m zone with a 1.8 m car
    // gives ~20 cm max clearance per side when perfectly centred.
    const precLabel = prec >= 0.15 ? 'Perfect' : prec >= 0.07 ? 'Good' : prec >= 0.02 ? 'Tight' : 'Squeezed';
    $('ovStats').innerHTML =
      `<div class="sc-row"><span class="sc-label">Time</span><span class="sc-val">${st.time.toFixed(1)} s</span></div>` +
      `<div class="sc-row"><span class="sc-label">Clearance</span><span class="sc-val">${precCm} cm <span class="sc-note">${precLabel}</span></span></div>` +
      (best && best.time < st.time - 0.05
        ? `<div class="sub" style="margin-top:6px">Best: ${best.time.toFixed(1)} s &nbsp;·&nbsp; ${starStr(best.stars)}</div>`
        : '');
    $('ovTip').textContent = stars === 3 ? 'Perfect run!' :
      `3★ ≤ ${(level.starThreshQuick || [999])[0]} s`;
    $('ovNext').style.display = nextPlayable(levelIdx, +1) >= 0 ? '' : 'none';
    pendingLb = solutionUsed ? null : { levelIdx, stars, st: { ...st } };
    $('ovSubmitRow').style.display = (lbEnabled() && !solutionUsed) ? '' : 'none';
    $('ovSubmit').disabled = false;
    $('ovSubmit').textContent = 'Submit to leaderboard';
    $('overlay').classList.remove('hidden');
  } else {
    toast('Not parked yet — keep planning');
  }
}

/* ===================== Level switching ===================== */

// Next/previous level index that isn't a cutscene, in the given direction.
// Cutscenes are skipped during normal navigation (only "Replay intro" enters
// one explicitly). Returns -1 when there's no playable level that way.
function nextPlayable(from, dir) {
  for (let i = from + dir; i >= 0 && i < LEVELS.length; i += dir)
    if (!isCutscene(LEVELS[i])) return i;
  return -1;
}

function setLevel(i) {
  levelIdx = (i + LEVELS.length) % LEVELS.length;
  // Don't persist progress while previewing a test level — the shifted
  // indices would corrupt the real game's saved position.
  if (!testLevelLoaded) localStorage.setItem('parking.level', String(levelIdx));
  const def = LEVELS[levelIdx];
  if (isCutscene(def)) { level = null; showCutscene(def); return; }
  $('intro').classList.add('hidden');  // leaving a cutscene
  setVehicle(def.vehicle || 'default');
  steerEl.min = -CAR.maxSteer;
  steerEl.max = CAR.maxSteer;
  level = buildLevel(def);
  moves = [];
  planSims = [];
  anim = null;
  editIdx = null;
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
}

/* ===================== Input ===================== */

const steerEl = $('steer'), distEl = $('dist');
// Steer slider is always ±35° (symmetric), so the neutral tick is fixed at 50%.
steerEl.style.setProperty('--zero', '50%');

function setEdit(steerDeg, dist) {
  editSteer = clamp(Math.abs(steerDeg) <= 2 ? 0 : steerDeg, -CAR.maxSteer, CAR.maxSteer);
  editDist = Math.abs(dist) < 0.15 ? 0 : dist; // clamped to drivable range in recomputeEdit
  steerEl.value = editSteer;
  $('steerVal').textContent = editSteer === 0 ? '0°'
    : `${Math.abs(editSteer)}° ${editSteer < 0 ? 'left' : 'right'}`;
  recomputeEdit();
  updateHUD();
}

steerEl.addEventListener('input', () => setEdit(parseFloat(steerEl.value), editDist));
distEl.addEventListener('input', () => setEdit(editSteer, parseFloat(distEl.value)));

function commitMove() {
  if (!editSim || editSim.pts.length < 2 || anim) return false;
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
  return true;
}

$('addBtn').addEventListener('click', commitMove);

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
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
});

$('goBtn').addEventListener('click', () => {
  if (anim) return;
  // Auto-commit a valid pending move so the player doesn't have to press Add first
  if (editSim && editSim.pts.length >= 2 && editIdx === null) {
    moves.push({ steer: rad(editSteer), dist: editDist });
    editIdx = null;
    setEdit(editSteer, 0);
    recomputePlan();
  }
  if (!moves.length) { toast('Add some moves first'); return; }
  startRun();
});

$('prevLv').addEventListener('click', () => { const t = nextPlayable(levelIdx, -1); if (t >= 0) setLevel(t); });
$('nextLv').addEventListener('click', () => { const t = nextPlayable(levelIdx, +1); if (t >= 0) setLevel(t); });

$('menuBtn').addEventListener('click', () => $('menuOverlay').classList.remove('hidden'));
$('menuClose').addEventListener('click', () => $('menuOverlay').classList.add('hidden'));
$('menuOverlay').addEventListener('click', e => {
  if (e.target === $('menuOverlay')) $('menuOverlay').classList.add('hidden');
});
$('menuHelp').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  $('helpOverlay').classList.remove('hidden');
});
$('menuSol').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  showSolution();
});
$('menuLb').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  if (lbEnabled()) openLeaderboard(levelIdx);
  else toast('Leaderboard not configured — see LB_URL / LB_KEY in game.js');
});
$('menuIntro').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  const ci = LEVELS.findIndex(isCutscene);
  if (ci >= 0) setLevel(ci);
  else toast('No cutscene in this game');
});
$('helpClose').addEventListener('click', () => $('helpOverlay').classList.add('hidden'));
$('lbClose').addEventListener('click', () => $('lbOverlay').classList.add('hidden'));

$('ovSubmit').addEventListener('click', async () => {
  if (!pendingLb) return;
  const player = localStorage.getItem('parking.player');
  if (!player) {
    $('nameInput').value = '';
    $('nameOverlay').classList.remove('hidden');
    setTimeout(() => $('nameInput').focus(), 50);
    return;
  }
  await doLbSubmit(player);
});

$('nameCancel').addEventListener('click', () => $('nameOverlay').classList.add('hidden'));
$('nameOk').addEventListener('click', async () => {
  const name = $('nameInput').value.trim().slice(0, 20);
  if (!name) return;
  localStorage.setItem('parking.player', name);
  $('nameOverlay').classList.add('hidden');
  if (pendingLb) await doLbSubmit(name);
});
$('nameInput').addEventListener('keydown', e => { if (e.key === 'Enter') $('nameOk').click(); });

$('ovImprove').addEventListener('click', () => $('overlay').classList.add('hidden'));
$('ovRetry').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  moves = [];
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
});
$('ovNext').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  const t = nextPlayable(levelIdx, +1);
  if (t >= 0) setLevel(t);
});

function selectMove(i) {
  if (anim) return;
  if (editIdx === i) { editIdx = null; setEdit(0, 0); return; }
  editIdx = i;
  const m = moves[i];
  setEdit(deg(m.steer), m.dist);
}

// ── Leaderboard functions ────────────────────────────────────────────────────
const lbEnabled = () => !!(LB_URL && LB_KEY);

function escHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

async function lbPost(levelIdx, player, stars, st) {
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard`, {
    method: 'POST',
    headers: {
      apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}`,
      'Content-Type': 'application/json', Prefer: 'return=minimal',
    },
    body: JSON.stringify({
      player, level: levelIdx, level_name: level.name,
      stars, moves: st.moves,
      dist: +st.dist.toFixed(2), time_s: +st.time.toFixed(1),
      mode: 'quick',
    }),
  });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
}

async function lbGet(levelIdx) {
  const p = new URLSearchParams({
    select: 'player,stars,moves,dist,time_s',
    level_name: `eq.${LEVELS[levelIdx].name}`, mode: 'eq.quick',
    order: 'stars.desc,time_s.asc', limit: '50',
  });
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard?${p}`, {
    headers: { apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}` },
  });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

async function openLeaderboard(idx) {
  $('lbTitle').textContent = `${LEVELS[idx].name} · Best Time`;
  $('lbTable').innerHTML = '<tr><td colspan="4" class="lb-empty">Loading…</td></tr>';
  $('lbOverlay').classList.remove('hidden');
  try {
    const rows = await lbGet(idx);
    // keep only each player's first (best) entry since rows are sorted optimally
    const seen = new Set();
    const top = rows.filter(r => { if (seen.has(r.player)) return false; seen.add(r.player); return true; }).slice(0, 10);
    if (!top.length) {
      $('lbTable').innerHTML = '<tr><td colspan="4" class="lb-empty">No entries yet — be first!</td></tr>';
      return;
    }
    $('lbTable').innerHTML = top.map((r, i) => {
      const cls = i === 0 ? 'lb-gold' : i === 1 ? 'lb-silver' : i === 2 ? 'lb-bronze' : '';
      const metric = `${r.time_s.toFixed(1)}s`;
      const stars = '★'.repeat(r.stars) + `<span class="lb-dim">★</span>`.repeat(3 - r.stars);
      return `<tr class="${cls}"><td class="lb-rank">${i + 1}</td><td class="lb-name">${escHtml(r.player)}</td><td class="lb-stars">${stars}</td><td class="lb-metric">${metric}</td></tr>`;
    }).join('');
  } catch (e) {
    $('lbTable').innerHTML = `<tr><td colspan="4" class="lb-empty" style="color:#ff7070">Error: ${escHtml(e.message)}</td></tr>`;
  }
}

async function doLbSubmit(player) {
  const { levelIdx: li, stars, st } = pendingLb;
  $('ovSubmit').disabled = true;
  $('ovSubmit').textContent = '⏳ Submitting…';
  try {
    await lbPost(li, player, stars, st);
    pendingLb = null;
    $('overlay').classList.add('hidden');
    await openLeaderboard(li);
  } catch (e) {
    toast(`Submit failed: ${e.message}`);
    $('ovSubmit').disabled = false;
    $('ovSubmit').textContent = 'Submit to leaderboard';
  }
}

function showSolution() {
  if (!level.solution) { toast('No solution on record for this level'); return; }
  if (anim) return;
  editIdx = null;
  solutionUsed = true;
  moves = level.solution.map(m => ({ steer: rad(m.steer), dist: m.dist }));
  setEdit(0, 0);
  recomputePlan();
  toast('Solution loaded — leaderboard disabled until Reset');
}


// Drag directly on the canvas: the ghost car chases the pointer. The arc
// from the current move's start pose through the pointer's world position
// determines both steering angle and signed distance, so dragging feels
// like placing the car where you want it to go.
// A tap (minimal movement) on a move badge selects it for tweaking.
function pointerToWorld(e) {
  const r = cv.getBoundingClientRect();
  return { x: (e.clientX - r.left - view.ox) / view.scale,
           y: (e.clientY - r.top - view.oy) / view.scale };
}

// Given a target point in world space, find the constant-steer arc from the
// edit start pose whose rear axle passes through it: a circle tangent to the
// pose heading. Returns {steer (deg), dist (m, signed)}.
function arcToPoint(pose, wp) {
  const dx = wp.x - pose.x, dy = wp.y - pose.y;
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const lx = dx * c + dy * s;    // forward component
  const ly = -dx * s + dy * c;   // lateral component (steer>0 side)
  if (Math.abs(ly) < 0.05) return { steer: 0, dist: lx };
  const R = (lx * lx + ly * ly) / (2 * ly);
  const th = Math.atan2(lx / R, (R - ly) / R);
  return { steer: deg(Math.atan(CAR.wb / R)), dist: R * th };
}

function editStartPose() {
  return editIdx !== null
    ? (editIdx === 0 ? level.start : planSims[editIdx - 1].end)
    : planEnd();
}

// Relative drag: the ghost is "grabbed" wherever it currently is and moves
// 1:1 with the finger in world space — no jump on touch, and the finger
// never has to cover the ghost. Double-tap commits the pending move.
let drag = null, lastTap = 0;
cv.addEventListener('pointerdown', e => {
  if (anim) return;
  cv.setPointerCapture(e.pointerId);
  const t = (editSim && Math.abs(editDist) > 0.01) ? editSim.end : editStartPose();
  drag = { x: e.clientX, y: e.clientY, tx: t.x, ty: t.y, moved: false };
});
cv.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.abs(dx) > 5 || Math.abs(dy) > 5) drag.moved = true;
  if (!drag.moved) return;
  const a = arcToPoint(editStartPose(),
    { x: drag.tx + dx / view.scale, y: drag.ty + dy / view.scale });
  setEdit(a.steer, a.dist);
});
cv.addEventListener('pointerup', e => {
  if (drag && !drag.moved) {
    const cx = e.clientX, cy = e.clientY;
    let hit = -1;
    for (let i = 0; i < planSims.length; i++) {
      const ep = planSims[i].end;
      const sp = toScreen({ x: ep.x + Math.cos(ep.h) * CAR.wb / 2,
                             y: ep.y + Math.sin(ep.h) * CAR.wb / 2 });
      if (Math.hypot(cx - sp.x, cy - sp.y) < 22) { hit = i; break; }
    }
    if (hit >= 0) {
      selectMove(hit);
    } else {
      const now = performance.now();
      if (now - lastTap < 350 && Math.abs(editDist) > 0.01) {
        if (commitMove()) toast('Move added');
        lastTap = 0;
      } else {
        lastTap = now;
        if (editIdx !== null) { editIdx = null; setEdit(0, 0); }
      }
    }
  }
  drag = null;
});
cv.addEventListener('pointercancel', () => { drag = null; });

document.addEventListener('gesturestart', e => e.preventDefault());

/* ===================== Cutscene / dashboard briefing ===================== */

// Default briefing text, used when a cutscene level omits its own `message`.
const DEFAULT_CUTSCENE_MSG = [
  '> MSG INCOMING', '', '  AGENT 7', '  VALET', '',
  '  PKG BY 0300.', '  NO SCRATCHES.', '', '> MISSION: GO.',
];
let cutsceneMessage = DEFAULT_CUTSCENE_MSG;

function showCutscene(def) {
  cutsceneMessage = (def.message && def.message.length) ? def.message : DEFAULT_CUTSCENE_MSG;
  playIntroDash();
}

let introAnimId = null;

function playIntroDash() {
  cancelAnimationFrame(introAnimId);
  $('intro').classList.remove('hidden');
  $('introGo').classList.add('hidden');

  const canvas = $('introCanvas');
  const c = canvas.getContext('2d');

  // Full device-pixel-ratio resolution — smooth rendering, no virtual grid.
  // Use window.innerWidth/Height (current visual viewport) and let CSS
  // width:100%;height:100% control the display size so it adapts if the
  // browser chrome shifts (URL bar show/hide on mobile).
  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const W = window.innerWidth;
  const H = window.innerHeight;
  canvas.width  = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  c.scale(dpr, dpr);

  const portrait = H > W * 1.1;

  // Layout bands
  const SKY_H = H * 0.37;
  const DASH_Y = SKY_H;
  const DASH_H = H * 0.46;

  // Centre CRT screen
  const SC_W = portrait ? W * 0.84 : Math.min(W * 0.52, H * 0.72);
  const SC_X = (W - SC_W) / 2;
  const SC_Y = DASH_Y + DASH_H * 0.09;
  const SC_H = DASH_H * 0.84;
  const SC_R = 8;

  const MSG = cutsceneMessage;
  const SC_PAD = Math.max(8, SC_W * 0.05);
  // Auto-fit font so the message fits the CRT screen on BOTH axes.
  const LINE_SP = 1.5;                                   // line-height multiplier
  const longest = MSG.reduce((m, l) => Math.max(m, l.length), 1);
  const fsByHeight = (SC_H - 2 * SC_PAD) / (MSG.length * LINE_SP);
  // Courier New advance width ≈ 0.6em per glyph.
  const fsByWidth  = (SC_W - 2 * SC_PAD) / (longest * 0.6);
  const FS = Math.max(7, Math.min(18, Math.floor(Math.min(fsByHeight, fsByWidth))));
  const LH = Math.ceil(FS * LINE_SP);

  // Side clusters only in landscape
  const showSide = !portrait;
  const SP_R  = showSide ? Math.min((SC_X - 12) * 0.82, DASH_H * 0.38) : 0;
  const SP_CX = SC_X / 2;
  const SP_CY = DASH_Y + DASH_H * 0.52;
  const RC_X  = Math.ceil(SC_X + SC_W + 8);
  const RC_W  = W - RC_X - 8;

  // Mirror
  const MIR_W = W * 0.28, MIR_H = Math.max(14, SKY_H * 0.18);
  const MIR_X = (W - MIR_W) / 2;

  // LEDs top-centre of dash
  const LED_XS = [-2,-1,0,1,2].map(i => W / 2 + i * W * 0.04);
  const LED_Y  = DASH_Y + 8;
  const LED_W  = Math.max(10, W * 0.024), LED_H = Math.max(5, H * 0.012);

  const rnd = n => { const x = Math.sin(n) * 1e4; return x - Math.floor(x); };

  // Rain
  const RAIN = Array.from({ length: Math.floor(W * 0.16) }, (_, i) => ({
    x: rnd(i * 7.3) * W, y0: rnd(i * 3.1) * SKY_H,
    spd: 55 + rnd(i * 5.7) * 90, len: 7 + rnd(i * 2.2) * 10,
  }));

  // Buildings with pre-generated windows
  const BLDGS = [
    [0.00,0.70,0.056],[0.04,0.53,0.038],[0.08,0.76,0.044],[0.12,0.65,0.033],
    [0.15,0.73,0.046],[0.19,0.56,0.036],[0.23,0.80,0.038],
    [0.63,0.73,0.046],[0.67,0.56,0.038],[0.71,0.74,0.046],[0.75,0.52,0.036],
    [0.79,0.70,0.053],[0.83,0.63,0.040],[0.88,0.77,0.050],[0.93,0.68,0.07],
  ].map(([xf, yf, wf]) => {
    const bx = xf * W, by = yf * SKY_H;
    const bw = Math.max(10, wf * W), bh = (1 - yf) * SKY_H;
    const cols = Math.max(1, Math.round(bw / 12)), rows = Math.max(1, Math.round(bh / 10));
    const wins = [];
    for (let ri = 0; ri < rows; ri++)
      for (let ci = 0; ci < cols; ci++)
        if (rnd((bx + ci) * 11 + (by + ri) * 7) < 0.32)
          wins.push({
            x: bx + (ci + 0.25) * (bw / cols), y: by + (ri + 0.3) * (bh / rows),
            w: bw / cols * 0.45, h: bh / rows * 0.5,
            col: rnd(bx + ci * 3.1 + ri) < 0.55 ? '#ffe880' : '#8899cc',
          });
    return { bx, by, bw, bh, wins };
  });

  let msgShown = false;
  const t0 = performance.now();

  function rrect(x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y); c.lineTo(x + w - r, y);
    c.arcTo(x + w, y, x + w, y + r, r); c.lineTo(x + w, y + h - r);
    c.arcTo(x + w, y + h, x + w - r, y + h, r); c.lineTo(x + r, y + h);
    c.arcTo(x, y + h, x, y + h - r, r); c.lineTo(x, y + r);
    c.arcTo(x, y, x + r, y, r); c.closePath();
  }

  function frame(now) {
    const ms = now - t0, t = ms / 1000;

    // ── Sky ───────────────────────────────────────────────────────────
    const skyGrd = c.createLinearGradient(0, 0, 0, SKY_H);
    skyGrd.addColorStop(0, '#05070d'); skyGrd.addColorStop(1, '#0b0f1c');
    c.fillStyle = skyGrd; c.fillRect(0, 0, W, SKY_H);

    for (const { bx, by, bw, bh, wins } of BLDGS) {
      c.fillStyle = '#090c18'; c.fillRect(bx, by, bw, bh);
      for (const w of wins) {
        c.fillStyle = w.col;
        c.globalAlpha = 0.5 + 0.12 * Math.sin(t * 0.4 + bx);
        c.fillRect(w.x, w.y, w.w, w.h);
      }
      c.globalAlpha = 1;
    }

    // Rearview mirror
    if (ms > 200) {
      c.fillStyle = '#1c2030';
      rrect(MIR_X, 4, MIR_W, MIR_H, 4); c.fill();
      c.fillStyle = '#0e1120';
      rrect(MIR_X + 2, 6, MIR_W - 4, MIR_H - 4, 3); c.fill();
      c.fillStyle = '#161924';
      c.fillRect(MIR_X + MIR_W * 0.08, 7, MIR_W * 0.16, MIR_H - 6);
      c.fillRect(MIR_X + MIR_W * 0.38, 7, MIR_W * 0.14, MIR_H - 6);
      c.fillRect(MIR_X + MIR_W * 0.68, 7, MIR_W * 0.16, MIR_H - 6);
    }

    // Rain
    if (ms > 400) {
      c.strokeStyle = 'rgba(100,150,220,0.45)'; c.lineWidth = 0.8;
      for (const dr of RAIN) {
        const y = (dr.y0 + dr.spd * t) % SKY_H;
        c.beginPath(); c.moveTo(dr.x, y); c.lineTo(dr.x - 1, y + dr.len); c.stroke();
      }
    }

    // ── Dashboard ─────────────────────────────────────────────────────
    const dashGrd = c.createLinearGradient(0, DASH_Y, 0, DASH_Y + DASH_H);
    dashGrd.addColorStop(0, '#181c27'); dashGrd.addColorStop(1, '#0e1118');
    c.fillStyle = dashGrd; c.fillRect(0, DASH_Y, W, H - DASH_Y);
    c.strokeStyle = '#252c3c'; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, DASH_Y); c.lineTo(W, DASH_Y); c.stroke();

    // LEDs
    {
      const blink = ms > 1600 && Math.floor(ms / 210) % 2 === 0;
      for (let i = 0; i < LED_XS.length; i++) {
        const lx = LED_XS[i] - LED_W / 2, on = i === 3 && blink;
        c.fillStyle = '#0d1018';
        rrect(lx - 1, LED_Y - 1, LED_W + 2, LED_H + 2, 2); c.fill();
        c.fillStyle = on ? '#4eff6a' : (i === 3 ? '#1a4a28' : '#1a2030');
        rrect(lx, LED_Y, LED_W, LED_H, 1); c.fill();
        if (on) {
          c.shadowColor = '#4eff6a'; c.shadowBlur = 10;
          rrect(lx, LED_Y, LED_W, LED_H, 1); c.fill();
          c.shadowBlur = 0;
        }
      }
    }

    // Speedometer (landscape only)
    if (showSide && SP_R > 8) {
      c.save();
      c.shadowColor = 'rgba(0,0,0,0.6)'; c.shadowBlur = 16;
      c.fillStyle = '#0b0e18';
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R, 0, Math.PI * 2); c.fill();
      c.shadowBlur = 0;
      const spGrd = c.createRadialGradient(SP_CX, SP_CY - SP_R * 0.3, 0, SP_CX, SP_CY, SP_R);
      spGrd.addColorStop(0, '#1c2232'); spGrd.addColorStop(1, '#0c0f1c');
      c.fillStyle = spGrd;
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R * 0.86, 0, Math.PI * 2); c.fill();
      for (let ti = 0; ti <= 10; ti++) {
        const a = Math.PI * 0.75 + ti * (Math.PI * 1.5 / 10), major = ti % 2 === 0;
        c.strokeStyle = major ? '#4a5a80' : '#28304e';
        c.lineWidth = major ? 1.5 : 0.8;
        const r1 = SP_R * 0.72, r2 = SP_R * (major ? 0.58 : 0.64);
        c.beginPath();
        c.moveTo(SP_CX + Math.cos(a) * r1, SP_CY + Math.sin(a) * r1);
        c.lineTo(SP_CX + Math.cos(a) * r2, SP_CY + Math.sin(a) * r2);
        c.stroke();
      }
      const na = Math.PI * 0.77;
      c.shadowColor = '#cc3333'; c.shadowBlur = 6;
      c.strokeStyle = '#dd3333'; c.lineWidth = 2;
      c.beginPath();
      c.moveTo(SP_CX + Math.cos(na + Math.PI) * SP_R * 0.14, SP_CY + Math.sin(na + Math.PI) * SP_R * 0.14);
      c.lineTo(SP_CX + Math.cos(na) * SP_R * 0.66, SP_CY + Math.sin(na) * SP_R * 0.66);
      c.stroke(); c.shadowBlur = 0;
      c.fillStyle = '#dd3333';
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R * 0.07, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#38486a'; c.font = `${Math.max(8, SP_R * 0.16)}px sans-serif`;
      c.textAlign = 'center'; c.textBaseline = 'alphabetic';
      c.fillText('km/h', SP_CX, SP_CY + SP_R * 0.42);
      c.restore();
    }

    // Fuel gauge (landscape only)
    if (showSide && RC_W > 24) {
      c.save();
      const rcH = DASH_H * 0.38, rcY = SC_Y;
      c.fillStyle = '#0c0f1c'; rrect(RC_X, rcY, RC_W, rcH, 6); c.fill();
      c.fillStyle = '#10131e'; rrect(RC_X + 2, rcY + 2, RC_W - 4, rcH - 4, 5); c.fill();
      c.fillStyle = '#2e3c54'; c.font = `bold ${Math.max(9, RC_W * 0.28)}px monospace`;
      c.textAlign = 'center'; c.textBaseline = 'top';
      c.fillText('FUEL', RC_X + RC_W / 2, rcY + 7);
      const fbY = rcY + rcH * 0.55, fbH = Math.max(3, H * 0.013);
      c.fillStyle = '#1c2436'; c.fillRect(RC_X + 5, fbY, RC_W - 10, fbH);
      c.fillStyle = '#b36a00'; c.fillRect(RC_X + 5, fbY, (RC_W - 10) * 0.14, fbH);
      c.textBaseline = 'alphabetic'; c.restore();
    }

    // ── CRT Screen ────────────────────────────────────────────────────
    c.shadowColor = 'rgba(0,0,0,0.75)'; c.shadowBlur = 24;
    c.fillStyle = '#0c1018'; rrect(SC_X - 5, SC_Y - 5, SC_W + 10, SC_H + 10, SC_R + 2); c.fill();
    c.shadowBlur = 0;
    const bezGrd = c.createLinearGradient(SC_X, SC_Y, SC_X, SC_Y + SC_H);
    bezGrd.addColorStop(0, '#1e2430'); bezGrd.addColorStop(1, '#131620');
    c.fillStyle = bezGrd; rrect(SC_X - 4, SC_Y - 4, SC_W + 8, SC_H + 8, SC_R + 1); c.fill();

    if (ms < 1400) {
      c.fillStyle = '#010208'; rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
    } else if (ms < 2100) {
      c.fillStyle = '#010208'; rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
      c.save(); rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.clip();
      const n = Math.floor((ms - 1400) / 700 * SC_W * SC_H * 0.05);
      for (let i = 0; i < n; i++) {
        c.fillStyle = Math.random() > 0.45 ? '#28b040' : '#102a18';
        c.fillRect(SC_X + Math.random() * SC_W, SC_Y + Math.random() * SC_H, 2, 2);
      }
      c.restore();
    } else if (ms < 2500) {
      const p = (ms - 2100) / 400;
      c.fillStyle = `rgb(${Math.floor(p*2)},${Math.floor(p*16)},${Math.floor(p*7)})`;
      rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
    } else {
      c.save(); rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.clip();
      const scrGrd = c.createRadialGradient(
        SC_X + SC_W * 0.5, SC_Y + SC_H * 0.35, 0,
        SC_X + SC_W * 0.5, SC_Y + SC_H * 0.5, SC_W * 0.7);
      scrGrd.addColorStop(0, '#021108'); scrGrd.addColorStop(1, '#010604');
      c.fillStyle = scrGrd; c.fillRect(SC_X, SC_Y, SC_W, SC_H);
      // Scanlines
      c.fillStyle = 'rgba(0,0,0,0.18)';
      for (let sy = SC_Y; sy < SC_Y + SC_H; sy += 3) c.fillRect(SC_X, sy, SC_W, 1);
      // Typed message
      c.fillStyle = '#3dfa65'; c.shadowColor = '#00ee55'; c.shadowBlur = 5;
      c.font = `${FS}px "Courier New", monospace`;
      c.textAlign = 'left'; c.textBaseline = 'top';
      let rem = Math.floor((ms - 2500) / 1000 * 18), done = true;
      for (let li = 0; li < MSG.length; li++) {
        const line = MSG[li];
        if (rem <= 0) { done = false; break; }
        const take = Math.max(1, line.length);
        if (rem >= take) {
          c.fillText(line, SC_X + SC_PAD, SC_Y + SC_PAD + li * LH); rem -= take;
        } else {
          c.fillText(line.slice(0, rem) + '█', SC_X + SC_PAD, SC_Y + SC_PAD + li * LH);
          rem = 0; done = false; break;
        }
      }
      c.shadowBlur = 0; c.restore();
      if (done && !msgShown) { msgShown = true; $('introGo').classList.remove('hidden'); }
      // Green glow rim
      c.shadowColor = 'rgba(0,200,60,0.4)'; c.shadowBlur = 20;
      c.strokeStyle = 'rgba(0,150,50,0.15)'; c.lineWidth = 2;
      rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.stroke();
      c.shadowBlur = 0;
    }

    // ── Steering wheel ────────────────────────────────────────────────
    {
      const wX = W / 2, wY = H + H * 0.22, wR = H * 0.44, hubR = wR * 0.13;
      c.strokeStyle = '#1e2334'; c.lineWidth = wR * 0.09;
      c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.12, Math.PI * 1.88); c.stroke();
      c.strokeStyle = '#2c3348'; c.lineWidth = wR * 0.04;
      c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.12, Math.PI * 1.88); c.stroke();
      c.strokeStyle = '#1a1e2c'; c.lineWidth = wR * 0.034;
      for (const a of [Math.PI * 1.22, Math.PI * 1.5, Math.PI * 1.78]) {
        c.beginPath();
        c.moveTo(wX + Math.cos(a) * hubR * 1.2, wY + Math.sin(a) * hubR * 1.2);
        c.lineTo(wX + Math.cos(a) * wR * 0.91, wY + Math.sin(a) * wR * 0.91);
        c.stroke();
      }
      const hGrd = c.createRadialGradient(wX - hubR * 0.3, wY - hubR * 0.3, 0, wX, wY, hubR);
      hGrd.addColorStop(0, '#2a3244'); hGrd.addColorStop(1, '#141824');
      c.fillStyle = hGrd;
      c.beginPath(); c.arc(wX, wY, hubR, 0, Math.PI * 2); c.fill();
    }

    if (!$('intro').classList.contains('hidden'))
      introAnimId = requestAnimationFrame(frame);
  }

  introAnimId = requestAnimationFrame(frame);
}

// A cutscene is a level — finishing it (Skip or Begin) advances to the next.
function endCutscene() {
  cancelAnimationFrame(introAnimId);
  introAnimId = null;
  $('introGo').classList.add('hidden');
  let t = nextPlayable(levelIdx, +1);
  if (t < 0) t = nextPlayable(levelIdx, -1);  // cutscene sits at the very end
  setLevel(t >= 0 ? t : levelIdx);
}
$('introSkip').addEventListener('click', endCutscene);
$('introGo').addEventListener('click', endCutscene);

/* ===================== Boot ===================== */

setLevel(levelIdx);
requestAnimationFrame(draw);
