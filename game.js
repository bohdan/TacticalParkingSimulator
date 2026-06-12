'use strict';

/* ===================== Car & math ===================== */

// All world units are meters. Pose = {x, y, h}; (x,y) is the REAR AXLE
// center, h is heading in radians (0 = +x, y axis points down on screen).
const CAR = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, maxSteer: 35 };
CAR.fOver = CAR.len - CAR.wb - CAR.rOver;

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
    name: "Lesson 1: Drive", tier: "Tutorial", mode: "moves", w: 18, h: 10,
    start: { x: 3, y: 5, h: 0 },
    goal: { cx: 13.5, cy: 5, w: 6.5, h: 3, heads: [0], tol: 15 },
    walls: [],
    cars: [],
    starThresh: [1, 2], starThreshQuick: [6, 9],
    tut: "Drag forward on the road — the ghost car follows your finger. Then press Run.",
    hint: "Just drive into the green zone.",
    solution: [{ steer: 0, dist: 9 }],
  },
  {
    name: "Lesson 2: Steer", tier: "Tutorial", mode: "moves", w: 14, h: 12,
    start: { x: 3, y: 9, h: 0 },
    goal: { cx: 6.9, cy: 3.8, w: 3.8, h: 5.8, heads: [-90], tol: 15 },
    walls: [],
    cars: [],
    starThresh: [1, 2], starThreshQuick: [5, 8],
    tut: "Steer with the top slider (or drag sideways) — the car drives in an arc.",
    hint: "One smooth arc gets you in.",
    solution: [{ steer: -35, dist: 6.06 }],
  },
  {
    name: "Lesson 3: Reverse", tier: "Tutorial", mode: "moves", w: 18, h: 10,
    start: { x: 13, y: 5, h: 0 },
    goal: { cx: 5.5, cy: 5, w: 7, h: 3.2, heads: [0], tol: 15 },
    walls: [],
    cars: [],
    starThresh: [1, 2], starThreshQuick: [5, 8],
    tut: "Drag backwards to reverse. Fewer moves = more stars (see ? for scoring).",
    hint: "The zone is behind you.",
    solution: [{ steer: 0, dist: -8 }],
  },
  {
    name: "First Steps", tier: "Easy", mode: "moves", w: 18, h: 12,
    start: { x: 2.5, y: 9.8, h: -Math.PI / 2 },
    goal: { cx: 14.2, cy: 2.8, w: 5.8, h: 2.6, heads: [0], tol: 12 },
    walls: [
      { x: 7, y: 5, w: 2.6, h: 7 },
    ],
    cars: [],
    starThresh: [2, 4], starThreshQuick: [11, 16],
    hint: "Get around the block into the green zone.",
    solution: [{ steer: 22, dist: 10.5 }, { steer: 0, dist: 3 }],
  },
  {
    name: "Open Bay", tier: "Easy", mode: "moves", w: 20, h: 13,
    start: { x: 16.5, y: 8.6, h: Math.PI },
    goal: { cx: 9.4, cy: 2.85, w: 4.5, h: 4.8, heads: [90, -90], tol: 10 },
    walls: [],
    cars: [
      { cx: 2.4, cy: 2.85, h: Math.PI / 2 },
      { cx: 4.7, cy: 2.85, h: Math.PI / 2 },
      { cx: 14.1, cy: 2.85, h: Math.PI / 2 },
      { cx: 16.4, cy: 2.85, h: Math.PI / 2 },
    ],
    starThresh: [3, 5], starThreshQuick: [18, 27],
    hint: "Plenty of room — back into the wide bay.",
    solution: [{ steer: 0, dist: 10.6 }, { steer: 35, dist: -6.3 }, { steer: 0, dist: -2.9 }],
  },
  {
    name: "First Parallel", tier: "Easy", mode: "moves", w: 24, h: 13,
    start: { x: 2.6, y: 7, h: 0 },
    goal: { cx: 10.7, cy: 9.4, w: 8, h: 2.2, heads: [0], tol: 12 },
    walls: [
      { x: 0, y: 10.5, w: 24, h: 2.5, kind: "curb" },
      { x: 0, y: 0, w: 24, h: 1.6, kind: "curb" },
    ],
    cars: [
      { cx: 4.6, cy: 9.5, h: 0 },
      { cx: 18.4, cy: 9.5, h: 0 },
      { cx: 9.5, cy: 2.55, h: Math.PI },
    ],
    starThresh: [3, 5], starThreshQuick: [17, 26],
    hint: "A roomy gap at the curb — classic reverse parallel park.",
    solution: [{ steer: 0, dist: 10.75 }, { steer: 35, dist: -3 }, { steer: -35, dist: -3 }],
  },
  {
    name: "Parallel Squeeze", tier: "Medium", mode: "moves", w: 22, h: 13,
    start: { x: 2.6, y: 7, h: 0 },
    goal: { cx: 10.15, cy: 9.4, w: 6.4, h: 2.2, heads: [0], tol: 12 },
    walls: [
      { x: 0, y: 10.5, w: 22, h: 2.5, kind: "curb" },
      { x: 0, y: 0, w: 22, h: 1.6, kind: "curb" },
    ],
    cars: [
      { cx: 4.6, cy: 9.5, h: 0 },
      { cx: 15.7, cy: 9.5, h: 0 },
      { cx: 9.5, cy: 2.55, h: Math.PI },
    ],
    starThresh: [3, 5], starThreshQuick: [17, 26],
    hint: "Reverse into the gap at the curb.",
    solution: [{ steer: 0, dist: 10.75 }, { steer: 35, dist: -3 }, { steer: -35, dist: -3 }],
  },
  {
    name: "Tight Bay", tier: "Medium", mode: "moves", w: 20, h: 13,
    start: { x: 16.5, y: 8.6, h: Math.PI },
    goal: { cx: 9.4, cy: 2.85, w: 2.7, h: 4.8, heads: [90, -90], tol: 10 },
    walls: [],
    cars: [
      { cx: 2.4, cy: 2.85, h: Math.PI / 2 },
      { cx: 4.7, cy: 2.85, h: Math.PI / 2 },
      { cx: 7, cy: 2.85, h: Math.PI / 2 },
      { cx: 11.8, cy: 2.85, h: Math.PI / 2 },
      { cx: 14.1, cy: 2.85, h: Math.PI / 2 },
      { cx: 16.4, cy: 2.85, h: Math.PI / 2 },
    ],
    starThresh: [3, 5], starThreshQuick: [18, 27],
    hint: "Back into the empty bay (either direction).",
    solution: [{ steer: 0, dist: 10.6 }, { steer: 35, dist: -6.3 }, { steer: 0, dist: -2.9 }],
  },
  {
    name: "Slalom", tier: "Medium", mode: "moves", w: 28, h: 12,
    start: { x: 2.8, y: 9.3, h: 0 },
    goal: { cx: 23.4, cy: 7.4, w: 7, h: 4.6, heads: [0], tol: 15 },
    walls: [
      { x: 7.5, y: 0, w: 2.4, h: 7 },
      { x: 16.5, y: 5.4, w: 2.4, h: 6.6 },
    ],
    cars: [],
    starThresh: [4, 6], starThreshQuick: [20, 31],
    hint: "Weave under, over, and home — all in forward gear.",
    solution: [{ steer: 0, dist: 4 }, { steer: -25, dist: 6 }, { steer: 25, dist: 10 }, { steer: -25, dist: 3 }],
  },
  {
    name: "Dead End", tier: "Medium", mode: "dist", w: 22, h: 13,
    start: { x: 3.2, y: 6.7, h: 0 },
    goal: { cx: 4.6, cy: 6.7, w: 8, h: 6.6, heads: [180], tol: 12 },
    walls: [
      { x: 0, y: 0, w: 22, h: 3.3 },
      { x: 0, y: 10.1, w: 22, h: 2.9 },
      { x: 18.4, y: 0, w: 3.6, h: 13 },
    ],
    cars: [],
    starThresh: [19, 27], starThreshQuick: [32, 49],
    hint: "Turn around to face the way you came.",
    solution: [{ steer: 12, dist: 3 }, { steer: -35, dist: 3 }, { steer: 35, dist: -3 }, { steer: -35, dist: 1.5 }, { steer: 0, dist: -1.5 }, { steer: -35, dist: 5 }],
  },
  {
    name: "Battle Park", tier: "Hard", mode: "moves", w: 24, h: 13,
    start: { x: 2.6, y: 7, h: 0 },
    goal: { cx: 9.29, cy: 9.22, w: 5, h: 2, heads: [0], tol: 10 },
    walls: [
      { x: 0, y: 10.5, w: 24, h: 2.5, kind: "curb" },
      { x: 0, y: 0, w: 24, h: 1.6, kind: "curb" },
    ],
    cars: [
      { cx: 4.8, cy: 9.4, h: 0 },
      { cx: 15.7, cy: 9.4, h: 0 },
      { cx: 9.5, cy: 2.55, h: Math.PI },
      { cx: 21, cy: 9.4, h: 0 },
    ],
    starThresh: [3, 5], starThreshQuick: [17, 26],
    hint: "Barely 9 cm from Car A — pure parallel precision.",
    solution: [{ steer: 0, dist: 10.75 }, { steer: 35, dist: -3 }, { steer: -35, dist: -3 }],
  },
  {
    name: "Distant Return", tier: "Hard", mode: "moves", w: 30, h: 13,
    start: { x: 5.4, y: 7, h: 0 },
    goal: { cx: 17.35, cy: 9.4, w: 5, h: 2.2, heads: [0], tol: 10 },
    walls: [
      { x: 0, y: 10.5, w: 30, h: 2.5, kind: "curb" },
      { x: 0, y: 0, w: 30, h: 1.6, kind: "curb" },
    ],
    cars: [
      { cx: 12.6, cy: 9.5, h: 0 },
      { cx: 23.75, cy: 9.5, h: 0 },
      { cx: 17.35, cy: 2.55, h: Math.PI },
    ],
    starThresh: [3, 5], starThreshQuick: [19, 30],
    hint: "Drive the length of the lot — the goal is barely wider than the car.",
    solution: [{ steer: 0, dist: 16 }, { steer: 35, dist: -3 }, { steer: -35, dist: -3 }],
  },
  {
    name: "Diagonal Slot", tier: "Hard", mode: "moves", w: 24, h: 12,
    start: { x: 3, y: 9, h: 0 },
    goal: { cx: 14, cy: 3.2, w: 5.2, h: 5.2, heads: [-45], tol: 8 },
    walls: [
      { x: 0, y: 0, w: 24, h: 0.5, kind: "curb" },
    ],
    cars: [
      { cx: 6.5, cy: 3.2, h: -Math.PI / 4 },
      { cx: 10.5, cy: 3.2, h: -Math.PI / 4 },
      { cx: 17.5, cy: 3.2, h: -Math.PI / 4 },
      { cx: 21, cy: 3.2, h: -Math.PI / 4 },
    ],
    starThresh: [3, 5], starThreshQuick: [11, 18],
    hint: "Angled bays: enter on the diagonal, dead straight.",
    solution: [{ steer: -15, dist: 6 }, { steer: 0, dist: 4 }, { steer: -35, dist: 1 }],
  },
  {
    name: "Narrow Dead End", tier: "Hard", mode: "dist", w: 22, h: 12,
    start: { x: 3.2, y: 6, h: 0 },
    goal: { cx: 4.6, cy: 6, w: 8, h: 5.8, heads: [180], tol: 12 },
    walls: [
      { x: 0, y: 0, w: 22, h: 3.1 },
      { x: 0, y: 8.9, w: 22, h: 3.1 },
      { x: 18.4, y: 0, w: 3.6, h: 12 },
    ],
    cars: [],
    starThresh: [16, 22], starThreshQuick: [50, 78],
    hint: "A 5.8 m corridor — the simple U-turn no longer fits.",
    solution: [{ steer: 0, dist: 5 }, { steer: -35, dist: 1 }, { steer: 35, dist: -2.5 }, { steer: -35, dist: 0.5 }, { steer: 35, dist: -1 }, { steer: -35, dist: 1 }, { steer: 35, dist: -1 }, { steer: -35, dist: 1 }, { steer: 35, dist: -0.5 }, { steer: -35, dist: 3 }],
  },
  {
    name: "Marathon Bay", tier: "Expert", mode: "moves", w: 36, h: 13,
    start: { x: 34, y: 8.6, h: Math.PI },
    goal: { cx: 9.4, cy: 2.85, w: 2.7, h: 4.8, heads: [90, -90], tol: 10 },
    walls: [],
    cars: [
      { cx: 2.4, cy: 2.85, h: Math.PI / 2 },
      { cx: 4.7, cy: 2.85, h: Math.PI / 2 },
      { cx: 7, cy: 2.85, h: Math.PI / 2 },
      { cx: 11.8, cy: 2.85, h: Math.PI / 2 },
      { cx: 14.1, cy: 2.85, h: Math.PI / 2 },
      { cx: 16.4, cy: 2.85, h: Math.PI / 2 },
      { cx: 24, cy: 11.6, h: 0 },
      { cx: 29, cy: 11.6, h: 0 },
    ],
    starThresh: [3, 5], starThreshQuick: [25, 39],
    hint: "Cross the entire 36 m lot, then thread the bay.",
    solution: [{ steer: 0, dist: 28.1 }, { steer: 35, dist: -6.3 }, { steer: 0, dist: -2.9 }],
  },
  {
    name: "The Garage", tier: "Expert", mode: "moves", w: 24, h: 13,
    start: { x: 3, y: 10.6, h: 0 },
    goal: { cx: 21, cy: 3.6, w: 5.4, h: 6.4, heads: [-90], tol: 10 },
    walls: [
      { x: 13, y: 0, w: 0.6, h: 4.9 },
      { x: 13, y: 8.1, w: 0.6, h: 4.9 },
    ],
    cars: [
      { cx: 16.2, cy: 3, h: Math.PI / 2 },
    ],
    starThresh: [4, 6], starThreshQuick: [21, 33],
    hint: "Thread the door, dodge the junk, face the back wall.",
    solution: [{ steer: -35, dist: 3 }, { steer: 15, dist: 10 }, { steer: -35, dist: 6 }, { steer: 25, dist: -0.5 }],
  },
  {
    name: "Tight Corner", tier: "Expert", mode: "dist", w: 22, h: 11,
    start: { x: 3.2, y: 5.5, h: 0 },
    goal: { cx: 4.6, cy: 5.5, w: 8, h: 5.6, heads: [180], tol: 12 },
    walls: [
      { x: 0, y: 0, w: 22, h: 2.7 },
      { x: 0, y: 8.3, w: 22, h: 2.7 },
      { x: 18.4, y: 0, w: 3.6, h: 11 },
    ],
    cars: [],
    starThresh: [18, 25], starThreshQuick: [59, 91],
    hint: "5.6 m to turn around — shuffle, inch by inch.",
    solution: [{ steer: -35, dist: 1 }, { steer: 20, dist: 5 }, { steer: -35, dist: -2.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 4 }],
  },
  {
    name: "The Gauntlet", tier: "Expert", mode: "dist", w: 22, h: 11,
    start: { x: 3.2, y: 5.5, h: 0 },
    goal: { cx: 4.6, cy: 5.5, w: 8, h: 5.5, heads: [180], tol: 12 },
    walls: [
      { x: 0, y: 0, w: 22, h: 2.75 },
      { x: 0, y: 8.25, w: 22, h: 2.75 },
      { x: 18.4, y: 0, w: 3.6, h: 11 },
    ],
    cars: [],
    starThresh: [20, 28], starThreshQuick: [69, 106],
    hint: "Only 5.5 m of corridor — a true shuffle marathon.",
    solution: [{ steer: -35, dist: 1.5 }, { steer: 20, dist: 6 }, { steer: -35, dist: -2 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 0.5 }, { steer: -35, dist: -0.5 }, { steer: 35, dist: 3 }],
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

let levelIdx = clamp(parseInt(localStorage.getItem('parking.level') || '0', 10) || 0, 0, LEVELS.length - 1);
let level = buildLevel(LEVELS[levelIdx]);

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

// Arc guides: shown when dist=0 but wheels are turned.
// Two solid curves = outer / inner edges of the car's swept path.
// Two dashed curves = front-left and front-right wheel tracks.
function drawArcGuides(pose, steerRad) {
  if (Math.abs(steerRad) < rad(1.5)) return;
  const N = 60;
  const fwdLimit = driveLimit(pose, steerRad, 1);
  const bwdLimit = driveLimit(pose, steerRad, -1);
  const fLen = CAR.wb + CAR.fOver;
  const half = CAR.wid / 2;

  function sampleArc(limit, dir) {
    const outer = [], inner = [], wfl = [], wfr = [];
    for (let i = 0; i <= N; i++) {
      const p = advance(pose, steerRad, dir * limit * i / N);
      const c = Math.cos(p.h), s = Math.sin(p.h);
      const w = (lx, ly) => ({ x: p.x + c * lx - s * ly, y: p.y + s * lx + c * ly });
      // outer corner = front-right for left turn, front-left for right turn
      if (steerRad >= 0) {
        outer.push(w(fLen,       -half));
        inner.push(w(-CAR.rOver,  half));
      } else {
        outer.push(w(fLen,        half));
        inner.push(w(-CAR.rOver, -half));
      }
      wfl.push(w(CAR.wb,  half));
      wfr.push(w(CAR.wb, -half));
    }
    return { outer, inner, wfl, wfr };
  }

  for (const [limit, dir] of [[fwdLimit, 1], [bwdLimit, -1]]) {
    if (limit < 0.2) continue;
    const { outer, inner, wfl, wfr } = sampleArc(limit, dir);
    const col  = dir > 0 ? 'rgba(69,196,255,0.38)' : 'rgba(255,159,67,0.38)';
    const wCol = dir > 0 ? 'rgba(69,196,255,0.60)' : 'rgba(255,159,67,0.60)';
    drawPath(outer, col, false, 0.06);
    drawPath(inner, col, false, 0.06);
    drawPath(wfl,  wCol,  true, 0.05);
    drawPath(wfr,  wCol,  true, 0.05);
  }
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

  // arc guides when distance is 0 but wheels are turned (no active animation)
  if (!anim && Math.abs(editDist) < 0.01) {
    drawArcGuides(editStartPose(), rad(editSteer));
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
    $('ovStats').innerHTML =
      `<div>${st.moves} moves &nbsp;·&nbsp; ${st.dist.toFixed(1)} m &nbsp;·&nbsp; ${st.time.toFixed(1)} s</div>` +
      (best && best.time < st.time - 0.05
        ? `<div class="sub" style="margin-top:4px">Best: ${best.time.toFixed(1)} s &nbsp;·&nbsp; ${starStr(best.stars)}</div>`
        : '');
    $('ovTip').textContent = stars === 3 ? 'Perfect!' :
      `3★ ≤ ${(level.starThreshQuick || [999])[0]} s`;
    $('ovNext').style.display = levelIdx < LEVELS.length - 1 ? '' : 'none';
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

function setLevel(i) {
  const prevIdx = levelIdx;
  levelIdx = (i + LEVELS.length) % LEVELS.length;
  localStorage.setItem('parking.level', String(levelIdx));
  level = buildLevel(LEVELS[levelIdx]);
  moves = [];
  planSims = [];
  anim = null;
  editIdx = null;
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
  // Show dashboard intro when leaving the last tutorial level for the first time
  const firstEasyIdx = LEVELS.findIndex(l => l.tier !== 'Tutorial');
  if (levelIdx === firstEasyIdx && prevIdx === firstEasyIdx - 1
      && !localStorage.getItem('parking.dashIntroSeen')) {
    setTimeout(playIntroDash, 80);
  }
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

$('prevLv').addEventListener('click', () => setLevel(levelIdx - 1));
$('nextLv').addEventListener('click', () => setLevel(levelIdx + 1));

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
  playIntroDash();
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
  setLevel(levelIdx + 1);
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

/* ===================== Intro briefing ===================== */

/* ===================== Dashboard intro animation ===================== */

let introAnimId = null;

function playIntroDash() {
  cancelAnimationFrame(introAnimId);
  $('intro').classList.remove('hidden');
  $('introGo').classList.add('hidden');

  const canvas = $('introCanvas');
  const c = canvas.getContext('2d');
  c.imageSmoothingEnabled = false;

  // Virtual resolution: one "pixel" = PX physical pixels, matching screen aspect ratio
  const cW = canvas.offsetWidth  || window.innerWidth;
  const cH = canvas.offsetHeight || window.innerHeight;
  const PX = Math.max(2, Math.round(Math.min(cW, cH) / 96));
  const VW = Math.floor(cW / PX);
  const VH = Math.floor(cH / PX);
  canvas.width  = VW;  canvas.height = VH;
  canvas.style.width  = cW + 'px';
  canvas.style.height = cH + 'px';

  // Layout bands (proportional to VH)
  const SKY_H = Math.floor(VH * 0.36);
  const DASH_Y = SKY_H;
  const DASH_H = Math.floor(VH * 0.46);

  // Centre CRT screen: width bounded by both orientation limits so it fits
  const SC_W = Math.min(Math.floor(VW * 0.70), Math.floor(VH * 0.78));
  const SC_X = Math.floor((VW - SC_W) / 2);
  const SC_Y = DASH_Y + Math.floor(DASH_H * 0.07);
  const SC_H = DASH_H - Math.floor(DASH_H * 0.12);
  // Font: tall enough to read, short enough to fit ~14-char lines in SC_W
  const FS = Math.max(4, Math.min(8, Math.floor(SC_H / 10)));

  // Side clusters (speedo + fuel) only when screen is wide enough relative to height
  const showSide = VW > VH * 0.62;
  const SP_CX = Math.floor(VW * 0.12);
  const SP_CY = Math.floor(DASH_Y + DASH_H * 0.52);
  const SP_R  = showSide ? Math.max(5, Math.min(SC_X - 3, Math.floor(DASH_H * 0.38))) : 0;
  const RC_X  = SC_X + SC_W + 3;
  const RC_W  = VW - RC_X - 1;

  // LEDs centred in dashboard
  const LED_Y  = DASH_Y + 2;
  const LED_XS = [0.37, 0.41, 0.45, 0.49, 0.53].map(f => Math.floor(VW * f));

  // Rearview mirror
  const MIR_X = Math.floor(VW * 0.34), MIR_W = Math.floor(VW * 0.32);

  // Message — max 14 chars per line so it fits on narrow portrait screens
  const MSG = [
    '> MSG INCOMING',
    '',
    '  AGENT 7',
    '  VALET',
    '',
    '  PKG BY 0300.',
    '  NO SCRATCHES.',
    '',
    '> MISSION: GO.',
  ];

  const rnd = n => { let x = Math.sin(n) * 1e4; return x - Math.floor(x); };

  // Rain proportional to sky area
  const RAIN = Array.from({ length: Math.max(8, Math.floor(VW * 0.22)) }, (_, i) => ({
    x:   Math.floor(rnd(i * 7.3) * VW),
    y0:  rnd(i * 3.1) * (SKY_H - 4),
    spd: (2.5 + rnd(i * 5.7) * 4) * SKY_H / 55,
    len: 2 + Math.floor(rnd(i * 2.2) * 3),
  }));

  // Buildings as proportions [xFrac, yFrac, wFrac]
  const BLDGS = [
    [0.00,0.70,0.056],[0.04,0.53,0.038],[0.08,0.76,0.044],[0.12,0.65,0.033],
    [0.15,0.73,0.046],[0.19,0.56,0.036],[0.23,0.80,0.038],
    [0.63,0.73,0.046],[0.67,0.56,0.038],[0.71,0.74,0.046],[0.75,0.52,0.036],
    [0.79,0.70,0.053],[0.83,0.63,0.040],[0.88,0.77,0.050],[0.93,0.68,0.07],
  ].map(([xf, yf, wf]) => [
    Math.floor(xf * VW), Math.floor(yf * SKY_H),
    Math.max(2, Math.floor(wf * VW)), Math.ceil((1 - yf) * SKY_H),
  ]);

  let msgShown = false;
  const t0 = performance.now();

  function fillCirclePx(cx, cy, r, col) {
    c.fillStyle = col;
    for (let dy = -r; dy <= r; dy++) {
      const dx = Math.round(Math.sqrt(Math.max(0, r * r - dy * dy)));
      c.fillRect(cx - dx, cy + dy, dx * 2 + 1, 1);
    }
  }

  function frame(now) {
    const ms = now - t0;

    // Background
    c.fillStyle = '#080b12'; c.fillRect(0, 0, VW, VH);

    // ── Sky ──────────────────────────────────────────────────────
    for (const [bx, by, bw, bh] of BLDGS) {
      c.fillStyle = '#0c0f19'; c.fillRect(bx, by, bw, bh);
      const sw = Math.max(1, Math.floor(bw / 3)), sh = Math.max(2, Math.floor(bh / 5));
      for (let wy = by + 1; wy < by + bh - 1; wy += sh)
        for (let wx = bx + 1; wx < bx + bw - 1; wx += sw)
          if (rnd(wx * 11 + wy * 7) < 0.30) {
            c.fillStyle = rnd(wx + wy * 3) < 0.55 ? '#ffe880' : '#8899cc';
            c.fillRect(wx, wy, Math.max(1, Math.floor(bw / 4)), Math.max(1, Math.floor(bh / 7)));
          }
    }

    // Rearview mirror
    const mirH = Math.max(3, Math.floor(SKY_H * 0.21));
    c.fillStyle = '#1a1d2a'; c.fillRect(MIR_X - 1, 1, MIR_W + 2, mirH + 2);
    c.fillStyle = '#0a0d18'; c.fillRect(MIR_X, 2, MIR_W, mirH);
    c.fillStyle = '#0e1122';
    c.fillRect(MIR_X + 2, 3, Math.floor(MIR_W * .18), mirH - 2);
    c.fillRect(MIR_X + Math.floor(MIR_W * .33), 4, Math.floor(MIR_W * .14), mirH - 3);
    c.fillRect(MIR_X + Math.floor(MIR_W * .62), 3, Math.floor(MIR_W * .18), mirH - 2);
    c.fillStyle = '#ffe880'; c.fillRect(MIR_X + 4, 4, 1, 1);
    c.fillStyle = '#8899cc'; c.fillRect(MIR_X + Math.floor(MIR_W * .17), 3, 1, 1);

    // Rain
    if (ms > 300) {
      const t = ms / 1000;
      c.fillStyle = 'rgba(90,130,200,.65)';
      for (const dr of RAIN)
        c.fillRect(dr.x, Math.floor((dr.y0 + dr.spd * t) % (SKY_H - 1)), 1, dr.len);
    }

    // ── Dashboard ────────────────────────────────────────────────
    c.fillStyle = '#141720'; c.fillRect(0, DASH_Y, VW, VH - DASH_Y);
    c.fillStyle = '#20242e'; c.fillRect(0, DASH_Y, VW, 1);

    // LEDs
    const blink = ms > 1600 && Math.floor(ms / 210) % 2 === 0;
    const lW = Math.max(2, Math.floor(VW * .025)), lH = Math.max(2, Math.floor(VH * .014));
    for (let i = 0; i < LED_XS.length; i++) {
      const lx = LED_XS[i], on = i === 3 && blink;
      c.fillStyle = '#0d1018'; c.fillRect(lx - 1, LED_Y, lW + 2, lH + 2);
      c.fillStyle = on ? '#4eff6a' : (i === 3 ? '#1a4a28' : '#22283a');
      c.fillRect(lx, LED_Y + 1, lW, lH);
      if (on) { c.globalAlpha = .28; c.fillStyle = '#4eff6a'; c.fillRect(lx - 2, LED_Y - 1, lW + 4, lH + 4); c.globalAlpha = 1; }
    }

    // Speedo (landscape only)
    if (showSide && SP_R > 0) {
      fillCirclePx(SP_CX, SP_CY, SP_R, '#0c0f18');
      fillCirclePx(SP_CX, SP_CY, SP_R - Math.max(1, Math.floor(SP_R * .11)), '#181c26');
      const r1 = SP_R - Math.floor(SP_R * .20), r2 = SP_R - Math.floor(SP_R * .34);
      for (let ti = 0; ti <= 8; ti++) {
        const a = Math.PI * .75 + ti * (Math.PI * 1.5 / 8);
        c.fillStyle = ti === 0 ? '#3a4560' : '#22283a';
        for (let tr = r2; tr <= r1; tr++)
          c.fillRect(Math.round(SP_CX + Math.cos(a) * tr), Math.round(SP_CY + Math.sin(a) * tr), 1, 1);
      }
      const na = Math.PI * .77;
      c.fillStyle = '#cc3333';
      for (let ni = 1; ni <= SP_R - Math.floor(SP_R * .22); ni++)
        c.fillRect(Math.round(SP_CX + Math.cos(na) * ni), Math.round(SP_CY + Math.sin(na) * ni), 1, 1);
      fillCirclePx(SP_CX, SP_CY, Math.max(1, Math.floor(SP_R * .10)), '#cc3333');
    }

    // Fuel strip (landscape only, right of screen)
    if (showSide && RC_W > 5) {
      const rcH = Math.floor(DASH_H * .36), rcY = SC_Y;
      c.fillStyle = '#0c0f18'; c.fillRect(RC_X, rcY, RC_W, rcH);
      c.fillStyle = '#181c26'; c.fillRect(RC_X + 1, rcY + 1, RC_W - 2, rcH - 2);
      const fbY = rcY + Math.floor(rcH * .48), fbH = Math.max(1, Math.floor(VH * .02));
      c.fillStyle = '#252c3a'; c.fillRect(RC_X + 2, fbY, RC_W - 4, fbH);
      c.fillStyle = '#c97d00'; c.fillRect(RC_X + 2, fbY, Math.max(1, Math.floor((RC_W - 4) * .14)), fbH);
      c.fillStyle = '#3a4460'; c.font = `${FS}px monospace`; c.textAlign = 'left';
      c.fillText('FUEL', RC_X + 2, rcY + Math.floor(rcH * .32));
    }

    // ── Centre CRT screen ─────────────────────────────────────────
    c.fillStyle = '#0a0c14'; c.fillRect(SC_X - 2, SC_Y - 2, SC_W + 4, SC_H + 4);
    c.fillStyle = '#060810'; c.fillRect(SC_X - 1, SC_Y - 1, SC_W + 2, SC_H + 2);

    if (ms < 1400) {
      c.fillStyle = '#010203'; c.fillRect(SC_X, SC_Y, SC_W, SC_H);
    } else if (ms < 2100) {
      c.fillStyle = '#010203'; c.fillRect(SC_X, SC_Y, SC_W, SC_H);
      const n = Math.floor((ms - 1400) / 700 * SC_W * SC_H * .14);
      for (let i = 0; i < n; i++) {
        c.fillStyle = Math.random() > .45 ? '#36cc5a' : '#183325';
        c.fillRect(SC_X + Math.floor(Math.random() * SC_W), SC_Y + Math.floor(Math.random() * SC_H), 1, 1);
      }
    } else if (ms < 2500) {
      const p = (ms - 2100) / 400;
      c.fillStyle = `rgb(${Math.floor(p * 3)},${Math.floor(p * 22)},${Math.floor(p * 11)})`;
      c.fillRect(SC_X, SC_Y, SC_W, SC_H);
    } else {
      c.fillStyle = '#010a04'; c.fillRect(SC_X, SC_Y, SC_W, SC_H);
      c.globalAlpha = .14; c.fillStyle = '#000';
      for (let sy2 = SC_Y; sy2 < SC_Y + SC_H; sy2 += 2) c.fillRect(SC_X, sy2, SC_W, 1);
      c.globalAlpha = 1;

      c.fillStyle = '#4eff6a';
      c.font = `${FS}px "Courier New", monospace`; c.textAlign = 'left';
      const lh = Math.ceil(FS * 1.6);
      let rem = Math.floor((ms - 2500) / 1000 * 18), done = true;
      for (let li = 0; li < MSG.length; li++) {
        const line = MSG[li];
        if (rem <= 0) { done = false; break; }
        const take = Math.max(1, line.length);
        if (rem >= take) {
          c.fillText(line, SC_X + 3, SC_Y + FS + li * lh); rem -= take;
        } else {
          c.fillText(line.slice(0, rem) + '█', SC_X + 3, SC_Y + FS + li * lh);
          rem = 0; done = false; break;
        }
      }
      if (done && !msgShown) { msgShown = true; $('introGo').classList.remove('hidden'); }
      c.globalAlpha = .06; c.fillStyle = '#4eff6a';
      c.fillRect(SC_X - 4, SC_Y - 4, SC_W + 8, SC_H + 8); c.globalAlpha = 1;
    }

    // ── Steering wheel (top arc at bottom of canvas) ──────────────
    const wX = Math.floor(VW / 2), wY = VH + Math.floor(VH * .28);
    const wR = Math.floor(VH * .44), hubR = Math.floor(wR * .24);
    c.strokeStyle = '#1a1e2e'; c.lineWidth = Math.max(2, Math.floor(wR * .09));
    c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.14, Math.PI * 1.86); c.stroke();
    c.strokeStyle = '#22273a'; c.lineWidth = Math.max(1, Math.floor(wR * .055));
    c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.14, Math.PI * 1.86); c.stroke();
    c.strokeStyle = '#1a1e2e'; c.lineWidth = Math.max(1, Math.floor(wR * .04));
    for (const a of [Math.PI * 1.22, Math.PI * 1.5, Math.PI * 1.78]) {
      c.beginPath();
      c.moveTo(wX + Math.cos(a) * hubR, wY + Math.sin(a) * hubR);
      c.lineTo(wX + Math.cos(a) * (wR - Math.floor(wR * .07)), wY + Math.sin(a) * (wR - Math.floor(wR * .07)));
      c.stroke();
    }
    fillCirclePx(wX, wY, hubR, '#181c2c');
    c.fillStyle = '#242838';
    c.fillRect(wX - Math.floor(hubR * .55), wY - Math.floor(hubR * .35), Math.floor(hubR * 1.1), Math.floor(hubR * .7));

    if (!$('intro').classList.contains('hidden'))
      introAnimId = requestAnimationFrame(frame);
  }

  introAnimId = requestAnimationFrame(frame);
}

function closeIntroDash() {
  cancelAnimationFrame(introAnimId);
  introAnimId = null;
  $('intro').classList.add('hidden');
  $('introGo').classList.add('hidden');
  localStorage.setItem('parking.dashIntroSeen', '1');
}
$('introSkip').addEventListener('click', closeIntroDash);
$('introGo').addEventListener('click', closeIntroDash);

/* ===================== Boot ===================== */

setLevel(levelIdx);
requestAnimationFrame(draw);
