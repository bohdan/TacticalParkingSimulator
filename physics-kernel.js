'use strict';
/*
 * physics-kernel.js — the PhysicsKernel interface, implemented.
 *
 * This is the refactor's Component 1 (PhysicsStatics + per-level PhysicsKernel + the
 * kernel-bound Solver hook) realised as code. It is ADDITIVE and self-contained: it does
 * NOT touch the legacy globals in physics.js, so both can load side by side while consumers
 * migrate. Everything is namespaced under the single global `Physics` (no `const advance`
 * clashes with physics.js).
 *
 * DEFAULT POSTURE: everything the layer emits is opaque. Pose / Point / Polygon /
 * VehicleSpec / Collision / Shape / Move / MoveResult are handles; the outside reads them
 * ONLY through the accessors below. The sole raw crossings are scalars (numbers, the
 * 'L'|'R'|'S' TurnDirection string) and the opaque wire STRING from moveSequenceToString.
 *
 * Works as a browser global and as a Node module (for tests).
 */
const Physics = (function () {

  /* ─── PhysicsStatics: constants ────────────────────────────────────────── */

  const SAMPLE_STEP = 0.06; // m, default collision sampling step along a path

  // Vehicle registry. `fOver` (front overhang) is derived: len - wb - rOver.
  const VEHICLE_DEFS = {
    default: { len: 4.4,  wid: 1.8,  wb: 2.7,   rOver: 0.85, maxSteer: 35 },
    miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
    bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
    tractor: { len: 3.8,  wid: 1.95, wb: 2.15,  rOver: 0.45, maxSteer: 52 },
  };

  // vehicleSpecFor(type) → opaque VehicleSpec (fOver filled). ⇐ game/editor/scene/kernel.
  function vehicleSpecFor(type) {
    const v = VEHICLE_DEFS[type] || VEHICLE_DEFS.default;
    return Object.freeze({
      len: v.len, wid: v.wid, wb: v.wb, rOver: v.rOver,
      fOver: v.len - v.wb - v.rOver, maxSteer: v.maxSteer,
    });
  }
  const VEHICLES = Object.freeze(Object.keys(VEHICLE_DEFS).reduce((o, k) => {
    o[k] = vehicleSpecFor(k); return o;
  }, {}));

  /* ─── PhysicsStatics: math helpers ─────────────────────────────────────── */

  const rad = d => d * Math.PI / 180;
  const deg = r => r * 180 / Math.PI;
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  function normalizeAngle(a) {
    a %= 2 * Math.PI;
    if (a >  Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /* ─── PhysicsStatics: geometry primitives (no vehicle dependence) ───────── */

  function rectanglePolygon(x, y, w, h) {
    return [{ x, y }, { x: x + w, y }, { x: x + w, y: y + h }, { x, y: y + h }];
  }
  function orientedBoxPolygon(cx, cy, w, h, ang) {
    const c = Math.cos(ang), s = Math.sin(ang);
    const pt = (x, y) => ({ x: cx + c * x - s * y, y: cy + s * x + c * y });
    return [pt(-w / 2, -h / 2), pt(w / 2, -h / 2), pt(w / 2, h / 2), pt(-w / 2, h / 2)];
  }
  function pointInPolygon(pt, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const a = poly[i], b = poly[j];
      if ((a.y > pt.y) !== (b.y > pt.y) &&
          pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }
  function polygonsCollide(A, B) {                 // SAT, convex polygons
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
  function pointToSegmentDistance(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    if (!l2) return Math.hypot(px - ax, py - ay);
    const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  }
  function convexHull(pts) {                       // Andrew's monotone chain, CCW
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
  function polygonBoundingCircle(poly) {
    let cx = 0, cy = 0;
    for (const v of poly) { cx += v.x; cy += v.y; }
    cx /= poly.length; cy /= poly.length;
    let r = 0;
    for (const v of poly) { const d = Math.hypot(v.x - cx, v.y - cy); if (d > r) r = d; }
    return { x: cx, y: cy, r };
  }

  // internal — NO external caller; not exported.
  function centroid(poly) {
    let x = 0, y = 0;
    for (const v of poly) { x += v.x; y += v.y; }
    return { x: x / poly.length, y: y / poly.length };
  }
  function contactPoint(carP, obsP) {
    for (const v of carP) if (pointInPolygon(v, obsP)) return v;
    for (const v of obsP) if (pointInPolygon(v, carP)) return v;
    const c = centroid(obsP);
    let best = carP[0], bd = Infinity;
    for (const v of carP) {
      const d = (v.x - c.x) ** 2 + (v.y - c.y) ** 2;
      if (d < bd) { bd = d; best = v; }
    }
    return best;
  }

  /* ─── Accessors for the opaque value types (the ONLY external read path) ─ */

  const poseX = p => p.x, poseY = p => p.y, poseHeading = p => p.h;
  const pointX = pt => pt.x, pointY = pt => pt.y;
  const polygonCount = poly => poly.length;
  const polygonVertex = (poly, i) => poly[i];
  const specLength = s => s.len, specWidth = s => s.wid, specWheelbase = s => s.wb;
  const specRearOverhang = s => s.rOver, specFrontOverhang = s => s.fOver;
  const specMaxSteer = s => s.maxSteer;
  const collisionPose = c => c.pose, collisionPoint = c => c.point;

  /* ─── Opaque obstacle Shape (pure geometry, vehicle-independent) ────────── */

  function makeShape(poly) { return { poly, bc: polygonBoundingCircle(poly) }; }
  const Shape = Object.freeze({
    rectangle:   (x, y, w, h)        => makeShape(rectanglePolygon(x, y, w, h)),
    orientedBox: (cx, cy, w, h, ang) => makeShape(orientedBoxPolygon(cx, cy, w, h, ang)),
    polygon:     (points)            => makeShape(points),
  });
  const shapePolygon  = shape => shape.poly;
  const shapesCollide = (a, b) => polygonsCollide(a.poly, b.poly);

  /* ─── Opaque Move (control intent; vehicle-independent) ─────────────────── */

  // Move(steeringDegrees, signedDistanceMeters). One allocation per user move — never hot.
  function Move(steeringDegrees, signedDistanceMeters) {
    return Object.freeze({ _steer: rad(steeringDegrees), _dist: signedDistanceMeters });
  }
  const moveTurnDirection = m => Math.abs(m._steer) < 1e-4 ? 'S' : (m._steer > 0 ? 'L' : 'R');
  const moveDirection     = m => m._dist >= 0 ? 1 : -1;
  const moveDistance      = m => m._dist;
  const moveSteeringDegrees = m => deg(m._steer);

  // Serialization — format owned by physics; callers treat the string as opaque.
  const round = (v, q) => Math.round(v * q) / q;
  const moveToString = m => `${round(deg(m._steer), 10)}:${round(m._dist, 100)}`;
  function parseMove(s) {
    const [d, dist] = s.split(':').map(Number);
    return Move(d, dist);
  }
  const moveSequenceToString = moves => moves.map(moveToString).join(';');
  const parseMoveSequence = s => (s ? s.split(';').filter(Boolean).map(parseMove) : []);

  /* ─── Opaque MoveResult accessors ──────────────────────────────────────── */

  const resultPath      = r => r.pts;
  const resultEndPose   = r => r.end;
  const resultCollision = r => r.hit;

  /* ─── Per-level kernel config ──────────────────────────────────────────── */

  // physicsConfigForLevel(def) → PhysicsConfig chosen by the level's vehicle type.
  function physicsConfigForLevel(def) {
    return Object.freeze({
      vehicle: vehicleSpecFor(def && def.vehicle),
      sampleStep: SAMPLE_STEP,
    });
  }

  /* ─── PhysicsKernel(config): the per-level instance (Component 1b) ──────── */

  function PhysicsKernel(config) {
    const spec = config.vehicle;
    const sampleStep = config.sampleStep || SAMPLE_STEP;
    // Spec-derived constants, computed ONCE (captured by the closures below).
    const wheelbase = spec.wb;
    const carRadius = 0.5 * Math.hypot(spec.len, spec.wid) + 0.02;   // broad-phase radius
    const centerOffset = (spec.wb + spec.fOver - spec.rOver) / 2;     // body centre ahead of rear axle

    // ── kinematics (HOT) ──────────────────────────────────────────────────
    function advancePose(p, steer, s) {
      if (Math.abs(steer) < 1e-4)
        return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
      const R = wheelbase / Math.tan(steer);
      const cx = p.x - Math.sin(p.h) * R, cy = p.y + Math.cos(p.h) * R;
      const h2 = p.h + s / R;
      return { x: cx + Math.sin(h2) * R, y: cy - Math.cos(h2) * R, h: h2 };
    }
    function turnRadius(steer) {
      return Math.abs(steer) < 1e-4 ? Infinity : wheelbase / Math.tan(steer);
    }
    function arcCenter(p, steer) {
      if (Math.abs(steer) < 1e-4) return null;       // straight line: no finite centre
      const R = wheelbase / Math.tan(steer);
      return { x: p.x - Math.sin(p.h) * R, y: p.y + Math.cos(p.h) * R };
    }
    function carPolygon(p, inf = 0) {
      const c = Math.cos(p.h), s = Math.sin(p.h);
      const x0 = -spec.rOver - inf, x1 = spec.wb + spec.fOver + inf;
      const y0 = -spec.wid / 2 - inf, y1 = spec.wid / 2 + inf;
      const pt = (x, y) => ({ x: p.x + c * x - s * y, y: p.y + s * x + c * y });
      return [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
    }
    function carShape(p) { return makeShape(carPolygon(p)); }

    // ── collision (HOT) ───────────────────────────────────────────────────
    // poseCollides: single-pose broad-phase + SAT. carRadius/centerOffset captured.
    function poseCollides(x, y, h, shapes) {
      const ccx = x + Math.cos(h) * centerOffset, ccy = y + Math.sin(h) * centerOffset;
      let poly = null;
      for (let i = 0; i < shapes.length; i++) {
        const o = shapes[i], bc = o.bc;
        const dx = ccx - bc.x, dy = ccy - bc.y, rr = carRadius + bc.r;
        if (dx * dx + dy * dy > rr * rr) continue;
        if (!poly) poly = carPolygon({ x, y, h });
        if (polygonsCollide(poly, o.poly)) return true;
      }
      return false;
    }
    // simulateMove: swept, continuous collision (convex hull of consecutive poses).
    function simulateMove(start, steer, dist, shapes, step) {
      const n = Math.max(2, Math.ceil(Math.abs(dist) / (step || sampleStep)));
      const pts = [start];
      let hit = null;
      const stepLen = Math.abs(dist) / n;
      let prevPoly = carPolygon(start);
      for (let i = 1; i <= n; i++) {
        const p = advancePose(start, steer, dist * i / n);
        const curPoly = carPolygon(p);
        let swept = null;
        const ccx = p.x + Math.cos(p.h) * centerOffset, ccy = p.y + Math.sin(p.h) * centerOffset;
        for (let oi = 0; oi < shapes.length; oi++) {
          const o = shapes[oi], bc = o.bc;
          const dx = ccx - bc.x, dy = ccy - bc.y, rr = carRadius + stepLen + bc.r;
          if (dx * dx + dy * dy > rr * rr) continue;
          if (!swept) swept = convexHull(prevPoly.concat(curPoly));
          if (polygonsCollide(swept, o.poly)) {
            hit = { pose: p, point: contactPoint(curPoly, o.poly) };
            break;
          }
        }
        if (hit) break;
        pts.push(p);
        prevPoly = curPoly;
      }
      return { pts, end: pts[pts.length - 1], hit };
    }

    // ── opaque gameplay surface ───────────────────────────────────────────
    const applyMove = (pose, move, shapes, step) =>
      simulateMove(pose, move._steer, move._dist, shapes, step);
    const moveTurnRadius = move => turnRadius(move._steer);

    // ── createSolver(): kernel-bound Solver (Component 1c) ────────────────
    // The search STRATEGY (A*, dock, anytime) is the large deferred port; this is the
    // interface, bound to THIS kernel and standing on the low-level surface above.
    function createSolver() { return makeSolver(kernel); }

    const kernel = {
      config, spec,
      // low-level kinematic surface (intra-component: the bundled Solver)
      wheelbase, carRadius, centerOffset,
      advancePose, turnRadius, arcCenter, carPolygon, carShape,
      poseCollides, simulateMove,
      // opaque gameplay surface
      applyMove, moveTurnRadius, createSolver,
    };
    return kernel;
  }

  /* ─── Solver (Component 1c) — bound to a kernel ─────────────────────────── */

  // Interface stub: the heavy weighted-A* / dock-interval search is ported in a later step.
  // Shape is fixed here so the contract is callable and testable.
  function makeSolver(kernel) {
    return {
      // solve(level, opts, progressCb?) → Promise<Move[] | null>
      solve(/* level, opts, progressCb */) {
        throw new Error('Solver.solve: search not yet ported (interface stub).');
      },
      // bruteForce(...) → Promise<void> (the Web-Worker entry)
      bruteForce(/* geom, params, emit, shouldStop, yieldHook, best, progressHook */) {
        throw new Error('Solver.bruteForce: search not yet ported (interface stub).');
      },
    };
  }

  /* ─── Public surface ───────────────────────────────────────────────────── */

  return {
    // constants / registry
    SAMPLE_STEP, VEHICLES, vehicleSpecFor,
    // math
    rad, deg, clamp, normalizeAngle,
    // geometry primitives
    rectanglePolygon, orientedBoxPolygon, pointInPolygon, convexHull,
    polygonBoundingCircle, polygonsCollide, pointToSegmentDistance,
    // opaque-type accessors
    poseX, poseY, poseHeading, pointX, pointY, polygonCount, polygonVertex,
    specLength, specWidth, specWheelbase, specRearOverhang, specFrontOverhang, specMaxSteer,
    collisionPose, collisionPoint,
    // Shape
    Shape, shapePolygon, shapesCollide,
    // Move
    Move, moveTurnDirection, moveDirection, moveDistance, moveSteeringDegrees,
    moveToString, parseMove, moveSequenceToString, parseMoveSequence,
    // MoveResult
    resultPath, resultEndPose, resultCollision,
    // kernel
    physicsConfigForLevel, PhysicsKernel,
  };
})();

if (typeof module !== 'undefined' && module.exports) module.exports = Physics;
