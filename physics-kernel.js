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
 * VALUE TYPES: Pose {x,y,h}, Point {x,y}, Polygon (array of points), VehicleSpec
 * {len,wid,wb,rOver,fOver,maxSteer}, Collision {pose,point}, MoveResult {pts,end,hit} and
 * Shape {poly,bc} are plain readable structs — read their fields directly. Move is the one
 * encapsulated type: its `_steer` (radians) / `_dist` are private (underscore-prefixed) and
 * read only via the move* helpers (degrees in/out, 'L'|'R'|'S' TurnDirection, wire string).
 *
 * Generic 2D geometry (SAT, hull, point-in-polygon, segment math) lives in geometry2d.js
 * (`Geom2D`); this file owns only the vehicle/kinematics/collision-orchestration layer.
 *
 * Works as a browser global and as a Node module (for tests).
 */
import { Geom2D } from './geometry2d.js';

export const Physics = (function (G) {

  /* ─── PhysicsStatics: constants ────────────────────────────────────────── */

  const SAMPLE_STEP = 0.06; // m, default collision sampling step along a path

  // Vehicle registry. `fOver` (front overhang) is derived: len - wb - rOver.
  const VEHICLE_DEFS = {
    default: { len: 4.4,  wid: 1.8,  wb: 2.7,   rOver: 0.85, maxSteer: 35 },
    miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
    bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
    tractor: { len: 3.8,  wid: 1.95, wb: 2.15,  rOver: 0.45, maxSteer: 52 },
  };

  // vehicleSpecFor(type) → VehicleSpec (fOver filled). ⇐ game/editor/scene/kernel.
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

  /* ─── Geometry comes from Geom2D (geometry2d.js) ───────────────────────── */
  // Pulled into locals so the hot loops below call them as flat references.
  const { rectanglePolygon, orientedBoxPolygon, pointInPolygon, polygonsCollide,
          convexHull, polygonBoundingCircle, contactPoint, pointToSegmentDistance } = G;

  /* ─── Obstacle Shape (pure geometry, vehicle-independent) ───────────────── */
  // Shape = { poly, bc }: poly is the polygon, bc its bounding circle (collision cache).

  function makeShape(poly) { return { poly, bc: polygonBoundingCircle(poly) }; }
  const Shape = Object.freeze({
    rectangle:   (x, y, w, h)        => makeShape(rectanglePolygon(x, y, w, h)),
    orientedBox: (cx, cy, w, h, ang) => makeShape(orientedBoxPolygon(cx, cy, w, h, ang)),
    polygon:     (points)            => makeShape(points),
  });
  const shapesCollide = (a, b) => polygonsCollide(a.poly, b.poly);

  /* ─── Move (control intent; vehicle-independent; encapsulated) ──────────── */

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

    // ── goal geometry / fit ───────────────────────────────────────────────
    // goalPolygon: the goal zone as an opaque Polygon (axis-aligned or oriented box).
    function goalPolygon(goal) {
      return goal.ang
        ? orientedBoxPolygon(goal.cx, goal.cy, goal.w, goal.h, goal.ang)
        : rectanglePolygon(goal.cx - goal.w / 2, goal.cy - goal.h / 2, goal.w, goal.h);
    }
    // inGoal: does this vehicle's footprint sit fully inside the goal at an allowed heading?
    function inGoal(pose, goal) {
      const okHead = goal.heads.some(
        hd => Math.abs(normalizeAngle(pose.h - rad(hd))) <= rad(goal.tol));
      if (!okHead) return false;
      const zone = goalPolygon(goal);
      return carPolygon(pose).every(v => pointInPolygon(v, zone));
    }

    // parkingClearance: smallest distance from any car corner to the goal zone edge.
    function parkingClearance(pose, goal) {
      const cp = carPolygon(pose), zone = goalPolygon(goal);
      let minGap = Infinity;
      for (const v of cp)
        for (let j = 0; j < zone.length; j++) {
          const a = zone[j], b = zone[(j + 1) % zone.length];
          minGap = Math.min(minGap, pointToSegmentDistance(v.x, v.y, a.x, a.y, b.x, b.y));
        }
      return isFinite(minGap) ? minGap : 0;
    }
    // distToGoalBoundary: signed distance of the rear-axle point to the goal edge
    // (positive outside / approaching, negative inside).
    function distToGoalBoundary(pose, goal) {
      const zone = goalPolygon(goal);
      let d = Infinity;
      for (let j = 0; j < zone.length; j++) {
        const a = zone[j], b = zone[(j + 1) % zone.length];
        d = Math.min(d, pointToSegmentDistance(pose.x, pose.y, a.x, a.y, b.x, b.y));
      }
      return pointInPolygon({ x: pose.x, y: pose.y }, zone) ? -d : d;
    }
    // distCarToGoal: signed distance of the car outline to the goal zone (positive = a
    // corner is outside, magnitude = nearest edge gap; negative = fully inside).
    function distCarToGoal(pose, goal) {
      const cp = carPolygon(pose), zone = goalPolygon(goal);
      let minD = Infinity, anyOutside = false;
      for (const v of cp) {
        if (!pointInPolygon(v, zone)) anyOutside = true;
        for (let j = 0; j < zone.length; j++) {
          const a = zone[j], b = zone[(j + 1) % zone.length];
          minD = Math.min(minD, pointToSegmentDistance(v.x, v.y, a.x, a.y, b.x, b.y));
        }
      }
      return anyOutside ? minD : -minD;
    }

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

    // ── gameplay surface ──────────────────────────────────────────────────
    // The kernel owns each operation and returns the result directly (a bool from
    // inGoal, a Polygon from goalPolygon) — callers read fields, no accessors.
    const applyMove = (pose, move, shapes, step) =>
      simulateMove(pose, move._steer, move._dist, shapes, step);
    const moveTurnRadius = move => turnRadius(move._steer);

    // ── createSolver(): kernel-bound Solver (Component 1c) ────────────────
    // The search STRATEGY (A*, dock, anytime) is the large deferred port; this is the
    // interface, bound to THIS kernel and standing on the low-level surface above.
    function createSolver() { return makeSolver(kernel); }

    const kernel = {
      config, spec,
      // private intra-component internals (for the bundled Solver / low-level use)
      _wheelbase: wheelbase, _carRadius: carRadius, _centerOffset: centerOffset,
      advancePose, turnRadius, arcCenter, carPolygon, carShape,
      poseCollides, simulateMove,
      // gameplay surface
      goalPolygon, inGoal, parkingClearance, distToGoalBoundary, distCarToGoal,
      applyMove, moveTurnRadius, createSolver,
    };
    return kernel;
  }

  /* ─── Solver (Component 1c) — bound to a kernel ─────────────────────────── */

  // The weighted-A* / dock-interval search lives in solver.js. To avoid an import cycle
  // (solver.js imports this module), solver.js registers its factory via _useSolver() on
  // load; createSolver() then resolves through it. A kernel built for collision-only use
  // never calls createSolver and so needs no solver.
  let _solverFactory = null;
  const _useSolver = factory => { _solverFactory = factory; };
  function makeSolver(kernel) {
    if (!_solverFactory)
      throw new Error('Solver not loaded — import ./solver.js before calling createSolver().');
    return _solverFactory(kernel);   // → { solve, bruteForce, validateMoves }
  }

  /* ─── Public surface ───────────────────────────────────────────────────── */

  return {
    // constants / registry
    SAMPLE_STEP, VEHICLES, vehicleSpecFor,
    // math
    rad, deg, clamp, normalizeAngle,
    // (generic 2D geometry lives in Geom2D / geometry2d.js — not re-exported here)
    // (Pose/Point/Polygon/VehicleSpec/Collision/MoveResult are plain structs — read fields)
    // Shape
    Shape, shapesCollide,
    // Move (encapsulated — these are its read/serialize surface)
    Move, moveTurnDirection, moveDirection, moveDistance, moveSteeringDegrees,
    moveToString, parseMove, moveSequenceToString, parseMoveSequence,
    // kernel
    physicsConfigForLevel, PhysicsKernel,
    // solver registration hook (solver.js calls this on import)
    _useSolver,
  };
})(Geom2D);
