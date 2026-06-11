'use strict';
// Headless solver — run with: node solve.js
// Duplicates the pure math from game.js (no DOM).

const CAR = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, maxSteer: 35 };
CAR.fOver = CAR.len - CAR.wb - CAR.rOver;
const SAMPLE_STEP = 0.06;
const rad = d => d * Math.PI / 180;
const deg = r => r * 180 / Math.PI;

function normAng(a) {
  a %= 2 * Math.PI;
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
function advance(p, steer, s) {
  if (Math.abs(steer) < 1e-4)
    return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
  const R = CAR.wb / Math.tan(steer);
  const cx = p.x - Math.sin(p.h) * R, cy = p.y + Math.cos(p.h) * R;
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
function polysCollide(A, B) {
  for (const [P, Q] of [[A, B], [B, A]]) {
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      const nx = b.y - a.y, ny = a.x - b.x;
      let minP = Infinity, maxP = -Infinity, minQ = Infinity, maxQ = -Infinity;
      for (const v of P) { const d = v.x*nx+v.y*ny; if (d<minP) minP=d; if (d>maxP) maxP=d; }
      for (const v of Q) { const d = v.x*nx+v.y*ny; if (d<minQ) minQ=d; if (d>maxQ) maxQ=d; }
      if (maxP < minQ || maxQ < minP) return false;
    }
  }
  return true;
}
function rectPoly(x, y, w, h) {
  return [{ x, y }, { x: x+w, y }, { x: x+w, y: y+h }, { x, y: y+h }];
}
function obbPoly(cx, cy, w, h, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const pt = (x, y) => ({ x: cx + c*x - s*y, y: cy + s*x + c*y });
  return [pt(-w/2,-h/2), pt(w/2,-h/2), pt(w/2,h/2), pt(-w/2,h/2)];
}
function simulateMove(start, steer, dist, obstacles) {
  const n = Math.max(2, Math.ceil(Math.abs(dist) / SAMPLE_STEP));
  const pts = [start];
  let hit = null;
  for (let i = 1; i <= n; i++) {
    const p = advance(start, steer, dist * i / n);
    const poly = carPoly(p);
    for (const o of obstacles) {
      if (polysCollide(poly, o)) { hit = { pose: p }; break; }
    }
    if (hit) break;
    pts.push(p);
  }
  return { pts, end: pts[pts.length - 1], hit };
}
function inGoal(pose, goal) {
  const okHead = goal.heads.some(hd => Math.abs(normAng(pose.h - rad(hd))) <= rad(goal.tol));
  if (!okHead) return false;
  const x0 = goal.cx - goal.w/2, x1 = goal.cx + goal.w/2;
  const y0 = goal.cy - goal.h/2, y1 = goal.cy + goal.h/2;
  return carPoly(pose).every(v => v.x >= x0 && v.x <= x1 && v.y >= y0 && v.y <= y1);
}

// Build obstacle list for Battle Park
const B = 0.45;
const W = 26, H = 13;
const obstacles = [
  rectPoly(-B, -B, W + 2*B, B),
  rectPoly(-B, H,  W + 2*B, B),
  rectPoly(-B, 0, B, H),
  rectPoly(W,  0, B, H),
  rectPoly(0, 10.5, W, 2.5),          // curb bottom
  rectPoly(0, 0,    W, 1.6),           // curb top
  obbPoly(5.25,  9.4, CAR.len, CAR.wid, 0),  // Car A (left of gap)
  obbPoly(15.05, 9.4, CAR.len, CAR.wid, 0),  // Car B (right of gap)
  obbPoly(9.5,   2.4, CAR.len, CAR.wid, Math.PI), // opposing car
];

const goal = { cx: 10.15, cy: 9.4, w: 5.2, h: 2.1, heads: [0], tol: 14 };
const start = { x: 2.6, y: 7.0, h: 0 };

// Beam search
const STEERS = [-35, -25, -15, 0, 15, 25, 35].map(rad);
const DISTS  = [-6, -5, -4, -3.5, -3, -2.5, -2, -1.5, -1, -0.5, 0.5, 1, 1.5, 2, 3, 4, 5, 6, 8, 10, 11, 12];
const BEAM   = 800;
const MAX_DEPTH = 6;

function heuristic(pose) {
  const dx = pose.x - goal.cx, dy = pose.y - goal.cy;
  const dh = Math.abs(normAng(pose.h - rad(0)));
  return dx*dx + dy*dy + (dh * 3) ** 2;
}

let beam = [{ pose: start, moves: [] }];
let best = null;

for (let depth = 0; depth < MAX_DEPTH; depth++) {
  const next = [];
  for (const state of beam) {
    for (const s of STEERS) {
      for (const d of DISTS) {
        const sim = simulateMove(state.pose, s, d, obstacles);
        if (sim.hit) continue;
        const moves = [...state.moves, { steer: Math.round(deg(s)), dist: d }];
        if (inGoal(sim.end, goal)) {
          if (!best || moves.length < best.length ||
              (moves.length === best.length && moves.reduce((t,m)=>t+Math.abs(m.dist),0) <
               best.reduce((t,m)=>t+Math.abs(m.dist),0))) {
            best = moves;
            console.log(`Depth ${depth+1}: ${JSON.stringify(best)}`);
          }
          continue; // don't extend winning states
        }
        next.push({ pose: sim.end, moves, h: heuristic(sim.end) });
      }
    }
  }
  if (best && best.length <= depth + 1) break;
  next.sort((a, b) => a.h - b.h);
  beam = next.slice(0, BEAM);
  const top = beam[0];
  console.log(`Depth ${depth+1}: beam=${beam.length} best h=${top?.h.toFixed(2)} pose=(${top?.pose.x.toFixed(2)},${top?.pose.y.toFixed(2)},${deg(top?.pose.h).toFixed(1)}°) moves=${top?.moves.length}`);
}

if (best) {
  console.log('\n=== BEST SOLUTION ===');
  console.log(JSON.stringify(best, null, 2));
  // Replay it to confirm
  let pose = start;
  for (const m of best) {
    const sim = simulateMove(pose, rad(m.steer), m.dist, obstacles);
    console.log(`  steer=${m.steer}° dist=${m.dist}m → (${sim.end.x.toFixed(2)}, ${sim.end.y.toFixed(2)}, ${deg(sim.end.h).toFixed(1)}°) hit=${!!sim.hit}`);
    pose = sim.end;
  }
  console.log(`  inGoal=${inGoal(pose, goal)}`);
} else {
  console.log('No solution found');
}
