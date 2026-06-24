'use strict';
// Shared physics engine — loaded by index.html (before game.js) and
// editor.html (before solver.js). All symbols are globals.

/* ─── Vehicle specs ────────────────────────────────────────────────────── */

const CAR = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, maxSteer: 35 };
CAR.fOver = CAR.len - CAR.wb - CAR.rOver;

const SEDAN = { len: 4.4, wid: 1.8, wb: 2.7, rOver: 0.85, fOver: 0.85, maxSteer: 35 };

const VEHICLES = {
  default: { len: 4.4,  wid: 1.8,  wb: 2.7,  rOver: 0.85, maxSteer: 35 },
  miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
  bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
  tractor: { len: 3.8,  wid: 1.95, wb: 2.15,  rOver: 0.45, maxSteer: 52 },
};

function setVehicle(name) {
  const v = VEHICLES[name] || VEHICLES.default;
  CAR.len = v.len; CAR.wid = v.wid; CAR.wb = v.wb;
  CAR.rOver = v.rOver; CAR.maxSteer = v.maxSteer;
  CAR.fOver = CAR.len - CAR.wb - CAR.rOver;
}

/* ─── Math helpers ─────────────────────────────────────────────────────── */

const SAMPLE_STEP = 0.06; // m, collision sampling along path

const rad   = d => d * Math.PI / 180;
const deg   = r => r * 180 / Math.PI;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

function normAng(a) {
  a %= 2 * Math.PI;
  if (a >  Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

/* ─── Kinematic bicycle model ──────────────────────────────────────────── */

// Constant steering angle → circular arc of the rear axle.
// steer in radians, s = signed arc length (negative = reverse).
function advance(p, steer, s) {
  if (Math.abs(steer) < 1e-4)
    return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
  const R  = CAR.wb / Math.tan(steer);
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

/* ─── Collision (SAT, convex polygons) ─────────────────────────────────── */

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

function ptSegDist(px, py, ax, ay, bx, by) {
  const dx = bx-ax, dy = by-ay, l2 = dx*dx+dy*dy;
  if (!l2) return Math.hypot(px-ax, py-ay);
  const t = Math.max(0, Math.min(1, ((px-ax)*dx+(py-ay)*dy)/l2));
  return Math.hypot(px-ax-t*dx, py-ay-t*dy);
}

/* ─── Geometry helpers ─────────────────────────────────────────────────── */

function rectPoly(x, y, w, h) {
  return [{ x, y }, { x: x+w, y }, { x: x+w, y: y+h }, { x, y: y+h }];
}

function obbPoly(cx, cy, w, h, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  const pt = (x, y) => ({ x: cx+c*x-s*y, y: cy+s*x+c*y });
  return [pt(-w/2,-h/2), pt(w/2,-h/2), pt(w/2,h/2), pt(-w/2,h/2)];
}

function goalPoly(g) {
  return g.ang
    ? obbPoly(g.cx, g.cy, g.w, g.h, g.ang)
    : [{ x: g.cx-g.w/2, y: g.cy-g.h/2 }, { x: g.cx+g.w/2, y: g.cy-g.h/2 },
       { x: g.cx+g.w/2, y: g.cy+g.h/2 }, { x: g.cx-g.w/2, y: g.cy+g.h/2 }];
}

function pointInPoly(pt, poly) {
  let inside = false;
  for (let i = 0, j = poly.length-1; i < poly.length; j = i++) {
    const a = poly[i], b = poly[j];
    if ((a.y > pt.y) !== (b.y > pt.y) &&
        pt.x < (b.x-a.x)*(pt.y-a.y)/(b.y-a.y)+a.x) inside = !inside;
  }
  return inside;
}

function centroid(poly) {
  let x = 0, y = 0;
  for (const v of poly) { x += v.x; y += v.y; }
  return { x: x/poly.length, y: y/poly.length };
}

function contactPoint(carP, obsP) {
  for (const v of carP) if (pointInPoly(v, obsP)) return v;
  for (const v of obsP) if (pointInPoly(v, carP)) return v;
  const c = centroid(obsP);
  let best = carP[0], bd = Infinity;
  for (const v of carP) {
    const d = (v.x-c.x)**2+(v.y-c.y)**2;
    if (d < bd) { bd = d; best = v; }
  }
  return best;
}

/* ─── Simulation ───────────────────────────────────────────────────────── */

// Convex hull (Andrew's monotone chain), CCW. Used to wrap two consecutive
// car rectangles into one swept polygon so collision is continuous, not just
// sampled at discrete poses.
function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a.x - b.x || a.y - b.y);
  const n = p.length;
  if (n < 3) return p;
  const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lo = [];
  for (const pt of p) {
    while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], pt) <= 0) lo.pop();
    lo.push(pt);
  }
  const hi = [];
  for (let i = n - 1; i >= 0; i--) {
    const pt = p[i];
    while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], pt) <= 0) hi.pop();
    hi.push(pt);
  }
  lo.pop(); hi.pop();
  return lo.concat(hi);
}

// Bounding circle of a polygon (centroid + farthest-vertex radius).
function polyBC(poly) {
  let cx = 0, cy = 0;
  for (const v of poly) { cx += v.x; cy += v.y; }
  cx /= poly.length; cy /= poly.length;
  let r = 0;
  for (const v of poly) { const d = Math.hypot(v.x - cx, v.y - cy); if (d > r) r = d; }
  return { x: cx, y: cy, r };
}

function simulateMove(start, steer, dist, obstacles, step) {
  const n = Math.max(2, Math.ceil(Math.abs(dist) / (step || SAMPLE_STEP)));
  const pts = [start];
  let hit = null;
  // Car bounding circle (constant size, moves with the car) for broad-phase.
  const rCar = 0.5 * Math.hypot(CAR.len, CAR.wid) + 0.02;
  const offc = (CAR.wb + CAR.fOver - CAR.rOver) / 2; // body centre ahead of rear axle
  const stepLen = Math.abs(dist) / n;                // arc length between samples
  // Swept collision: test the convex hull of each consecutive pose pair, so a
  // clip that passes between two samples is still caught. The hull matches the
  // true swept region to within the sub-mm corner-arc bulge at this step size.
  let prevPoly = carPoly(start);
  for (let i = 1; i <= n; i++) {
    const p = advance(start, steer, dist * i / n);
    const curPoly = carPoly(p);
    let swept = null;
    const ccx = p.x + Math.cos(p.h) * offc, ccy = p.y + Math.sin(p.h) * offc;
    for (let oi = 0; oi < obstacles.length; oi++) {
      const o = obstacles[oi];
      const bc = o.bc || (o.bc = polyBC(o.poly));
      // Broad-phase: hull reaches back ~stepLen toward the previous body centre.
      const dx = ccx - bc.x, dy = ccy - bc.y, rr = rCar + stepLen + bc.r;
      if (dx * dx + dy * dy > rr * rr) continue;     // broad-phase reject
      if (!swept) swept = convexHull(prevPoly.concat(curPoly));
      if (polysCollide(swept, o.poly)) {
        hit = { pose: p, point: contactPoint(curPoly, o.poly) };
        break;
      }
    }
    if (hit) break;
    pts.push(p);
    prevPoly = curPoly;
  }
  return { pts, end: pts[pts.length-1], hit };
}

/* ─── Level building ───────────────────────────────────────────────────── */

function buildLevel(def) {
  const obstacles = [];
  const B = 0.45;
  obstacles.push({ kind: 'border', poly: rectPoly(-B, -B, def.w+2*B, B) });
  obstacles.push({ kind: 'border', poly: rectPoly(-B, def.h, def.w+2*B, B) });
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
      ? { ...VEHICLES[c.type], fOver: VEHICLES[c.type].len-VEHICLES[c.type].wb-VEHICLES[c.type].rOver }
      : SEDAN;
    obstacles.push({ kind: 'car', pose: c, carSpec: sp, poly: obbPoly(c.cx, c.cy, sp.len, sp.wid, c.h) });
  }
  return Object.assign({ obstacles }, def);
}

function inGoal(pose, goal) {
  const okHead = goal.heads.some(
    hd => Math.abs(normAng(pose.h - rad(hd))) <= rad(goal.tol));
  if (!okHead) return false;
  return carPoly(pose).every(v => pointInPoly(v, goalPoly(goal)));
}
