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
 * encapsulated type: its `_sd` (steer, degrees, snapped to STEER_Q) / `_n` (signed integer
 * step count at DIST_Q m/step) are private (underscore-prefixed) and read only via the
 * move* helpers (degrees in/out, 'L'|'R'|'S' TurnDirection, wire string).
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
  const STEER_Q = 0.2;     // deg, player steering input grid
  const DIST_Q  = 0.05;    // m,   player distance input grid

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

  // Move(steeringDegrees, signedDistanceMeters).
  // Internally: _sd = steer snapped to STEER_Q (deg), _n = signed step count at DIST_Q.
  // This representation enables O(1) integer-key lookup into the kernel's precomputed
  // steer table without any float-to-int conversion at the call site.
  function Move(steeringDegrees, signedDistanceMeters) {
    const _sd = Math.round(steeringDegrees / STEER_Q) * STEER_Q;
    const _n  = Math.round(signedDistanceMeters / DIST_Q);
    return Object.freeze({ _sd, _n });
  }
  const moveTurnDirection   = m => Math.abs(m._sd) < 0.1 ? 'S' : (m._sd > 0 ? 'L' : 'R');
  const moveDirection       = m => m._n >= 0 ? 1 : -1;
  const moveDistance        = m => Math.round(m._n * DIST_Q * 100) / 100;
  const moveSteeringDegrees = m => m._sd;

  // Serialization — format owned by physics; callers treat the string as opaque.
  const moveToString = m => `${m._sd}:${moveDistance(m)}`;
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

    // ── Precomputed steer geometry table ──────────────────────────────────
    // Integer key for steer in degrees: avoids float-equality Map mismatches.
    // STEER_Q=0.2 → key=Math.round(sd*10), e.g. 0.2→2, 35.0→350, -0.2→-2.
    const _siKey = sd => Math.round(sd * 10);

    const nSteerSteps = Math.round(spec.maxSteer / STEER_Q);
    const NS = 2 * nSteerSteps + 1;
    const steers = new Array(NS);
    for (let i = 0; i < NS; i++) steers[i] = (i - nSteerSteps) * STEER_Q;
    const steerMap = new Map(steers.map((sd, si) => [_siKey(sd), si]));

    // Car corners in local frame (rear axle at origin, heading = +x direction).
    const locCorners = [
      [-spec.rOver,          -spec.wid / 2],
      [spec.wb + spec.fOver, -spec.wid / 2],
      [spec.wb + spec.fOver,  spec.wid / 2],
      [-spec.rOver,           spec.wid / 2],
    ];
    const locEdges = locCorners.map((c, i) => [c, locCorners[(i + 1) % 4]]);

    // Per-steer geometry: R, corner radii from C, true inner/outer radii.
    // Turn center in local frame for steer s: C_local = (0, R) where R = wb/tan(s).
    const steerTable = new Array(NS);
    for (let si = 0; si < NS; si++) {
      const s = steers[si] * Math.PI / 180;
      const R = Math.abs(s) < 1e-4 ? Infinity : wheelbase / Math.tan(s);
      const cornerRadii = locCorners.map(([cx, cy]) =>
        isFinite(R) ? Math.hypot(cx, cy - R) : Infinity);
      // True rMin: min distance from C=(0,R) to the car boundary (edge-based, not just corners).
      // Using only corners would undercount: C can project perpendicularly onto an edge.
      let trueRMin = isFinite(R) ? Infinity : 0;
      if (isFinite(R)) {
        for (const [[ax, ay], [bx, by]] of locEdges)
          trueRMin = Math.min(trueRMin, pointToSegmentDistance(0, R, ax, ay, bx, by));
      }
      steerTable[si] = Object.freeze({
        R, absR: Math.abs(R),
        cornerRadii,
        rMin: trueRMin,
        rMax: Math.max(...cornerRadii),
      });
    }

    // Local-frame delta table for fast arc advancement without per-step trig.
    // Index: si * MAX_ND + n  (n = 1 .. MAX_ND-1 forward steps).
    // dDx[si,n] = R*sin(n*DQ/R), dDy[si,n] = R*(1−cos(n*DQ/R)), dDh[si,n] = n*DQ/R.
    // Reverse: negate dDx and dDh; dDy unchanged (same lateral offset magnitude).
    const MAX_ND = 250; // covers 12.5 m at 0.05 m/step
    const dDx = new Float64Array(NS * MAX_ND);
    const dDy = new Float64Array(NS * MAX_ND);
    const dDh = new Float64Array(NS * MAX_ND);
    for (let si = 0; si < NS; si++) {
      const R = steerTable[si].R, base = si * MAX_ND;
      if (!isFinite(R)) {
        for (let n = 1; n < MAX_ND; n++) dDx[base + n] = n * DIST_Q; // dDy, dDh stay 0
      } else {
        for (let n = 1; n < MAX_ND; n++) {
          const a = n * DIST_Q / R;
          dDx[base + n] = R * Math.sin(a);
          dDy[base + n] = R * (1 - Math.cos(a));
          dDh[base + n] = a;
        }
      }
    }

    const _precomp = Object.freeze({ steers, steerTable, steerMap, _siKey, dDx, dDy, dDh, MAX_ND });

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

    // ── Analytic swept-arc collision ──────────────────────────────────────
    // sweepCollides: exact swept-envelope test for one constant-steer sub-step.
    // Returns the first shape that the car hits (or null). Replaces the convex-hull
    // approximation: for arc moves the car sweeps an annular sector, not a convex hull.
    //
    // Phases:
    //   0  bounding-circle broad-phase (annular sector outer/inner for arcs)
    //   1  endpoint SAT at start and end poses (catches initial/final overlap)
    //   straight only: SAT on convex hull of start+end polygon (exact for straight)
    //   arc 2a  car corner i traces a circle of radius cornR[i]; find where it crosses
    //            each obstacle edge and check if the crossing bearing is in the sweep arc
    //   arc 2b  obstacle vertex j at distance rOv from C; in the co-rotating frame it
    //            moves backward; find where its orbit (radius rOv) crosses each car edge
    //            from the start polygon and check if the crossing is in [−Delta] arc

    const TWO_PI = 2 * Math.PI;
    function inSweepArc(phi, phi0, Delta) {
      if (Math.abs(Delta) < 1e-9) return false;
      if (Delta > 0)
        return ((phi - phi0) % TWO_PI + TWO_PI) % TWO_PI <= Delta + 1e-9;
      else
        return ((phi0 - phi) % TWO_PI + TWO_PI) % TWO_PI <= -Delta + 1e-9;
    }

    function sweepCollides(start, steer, dist, shapes) {
      if (!shapes.length || Math.abs(dist) < 1e-9) return null;
      const startPoly = carPolygon(start);
      const end = advancePose(start, steer, dist);
      const endPoly = carPolygon(end);
      const isStr = Math.abs(steer) < 1e-4;
      const R = isStr ? Infinity : wheelbase / Math.tan(steer);
      const Delta = isStr ? 0 : dist / R;   // signed arc angle swept (CCW > 0)

      let Cx, Cy, cornBear, cornR, rMin, rMax;
      if (!isStr) {
        Cx = start.x - Math.sin(start.h) * R;
        Cy = start.y + Math.cos(start.h) * R;
        // Look up precomputed steer geometry (works for any steer on the STEER_Q grid).
        const si = steerMap.get(_siKey(deg(steer)));
        const sg = si != null ? steerTable[si] : null;
        cornBear = startPoly.map(v => Math.atan2(v.y - Cy, v.x - Cx));
        cornR = sg ? sg.cornerRadii : startPoly.map(v => Math.hypot(v.x - Cx, v.y - Cy));
        rMin  = sg ? sg.rMin        : cornR.reduce((a, b) => Math.min(a, b), Infinity);
        rMax  = sg ? sg.rMax        : cornR.reduce((a, b) => Math.max(a, b), 0);
      }

      for (let oi = 0; oi < shapes.length; oi++) {
        const shape = shapes[oi], obs = shape.poly, bc = shape.bc;

        // Phase 0: broad-phase
        if (isStr) {
          // Straight: obstacle must be within carRadius + |dist| of starting car centre.
          const ccx = start.x + Math.cos(start.h) * centerOffset;
          const ccy = start.y + Math.sin(start.h) * centerOffset;
          const dx = ccx - bc.x, dy = ccy - bc.y;
          const reach = carRadius + Math.abs(dist);
          if (dx * dx + dy * dy > (reach + bc.r) * (reach + bc.r)) continue;
        } else {
          // Arc: annular-sector outer/inner radius filter around turn centre C.
          const dx = Cx - bc.x, dy = Cy - bc.y;
          const distCC = Math.hypot(dx, dy);
          if (distCC - bc.r > rMax + 0.01) continue;   // all points outside outer arc
          if (distCC + bc.r < rMin - 0.01) continue;   // all points inside inner arc (not reachable)
        }

        // Phase 1: endpoint SAT
        if (polygonsCollide(startPoly, obs)) return shape;
        if (polygonsCollide(endPoly,   obs)) return shape;

        if (isStr) {
          // Straight: exact swept region = convex hull of start+end polygon.
          if (polygonsCollide(convexHull(startPoly.concat(endPoly)), obs)) return shape;
          continue;
        }

        // Phase 2a: each car corner traces a circle of radius cornR[ci] around C.
        // Detect crossings of that circle with each obstacle edge.
        for (let ci = 0; ci < 4; ci++) {
          const rk = cornR[ci], phi0 = cornBear[ci];
          for (let ei = 0; ei < obs.length; ei++) {
            const A = obs[ei], B = obs[(ei + 1) % obs.length];
            const ax = A.x - Cx, ay = A.y - Cy;
            const bx = B.x - A.x, by = B.y - A.y;
            const qa = bx * bx + by * by; if (qa < 1e-12) continue;
            const qb = 2 * (ax * bx + ay * by);
            const qc = ax * ax + ay * ay - rk * rk;
            const disc = qb * qb - 4 * qa * qc; if (disc < 0) continue;
            const sq = Math.sqrt(disc);
            for (const t of [(-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)]) {
              if (t < -1e-9 || t > 1 + 1e-9) continue;
              const phi = Math.atan2(A.y + t * by - Cy, A.x + t * bx - Cx);
              if (inSweepArc(phi, phi0, Delta)) return shape;
            }
          }
        }

        // Phase 2b: obstacle vertex at distance rOv from C, orbit radius rOv.
        // In the co-rotating frame the vertex moves backward (−Delta); find where its
        // orbit crosses a car edge from startPoly, then check if the bearing is in [−Delta].
        for (let vi = 0; vi < obs.length; vi++) {
          const Ov = obs[vi];
          const rOv = Math.hypot(Ov.x - Cx, Ov.y - Cy);
          if (rOv < rMin - 0.01 || rOv > rMax + 0.01) continue;
          const phi_Ov = Math.atan2(Ov.y - Cy, Ov.x - Cx);
          for (let ei = 0; ei < 4; ei++) {
            const A = startPoly[ei], B = startPoly[(ei + 1) % 4];
            const ax = A.x - Cx, ay = A.y - Cy;
            const bx = B.x - A.x, by = B.y - A.y;
            const qa = bx * bx + by * by; if (qa < 1e-12) continue;
            const qb = 2 * (ax * bx + ay * by);
            const qc = ax * ax + ay * ay - rOv * rOv;
            const disc = qb * qb - 4 * qa * qc; if (disc < 0) continue;
            const sq = Math.sqrt(disc);
            for (const t of [(-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)]) {
              if (t < -1e-9 || t > 1 + 1e-9) continue;
              const phi_int = Math.atan2(A.y + t * by - Cy, A.x + t * bx - Cx);
              if (inSweepArc(phi_int, phi_Ov, -Delta)) return shape;
            }
          }
        }
      }
      return null;
    }

    // simulateMove: swept, continuous collision using sweepCollides per sub-step.
    function simulateMove(start, steer, dist, shapes, step) {
      const n = Math.max(2, Math.ceil(Math.abs(dist) / (step || sampleStep)));
      const pts = [start];
      let hit = null;
      const subDist = dist / n;
      let prevPose = start;
      for (let i = 1; i <= n; i++) {
        const curPose = advancePose(start, steer, dist * i / n);
        const hitShape = sweepCollides(prevPose, steer, subDist, shapes);
        if (hitShape) {
          hit = { pose: curPose, point: contactPoint(carPolygon(curPose), hitShape.poly) };
          break;
        }
        pts.push(curPose);
        prevPose = curPose;
      }
      return { pts, end: pts[pts.length - 1], hit };
    }

    // ── gameplay surface ──────────────────────────────────────────────────
    const applyMove = (pose, move, shapes, step) =>
      simulateMove(pose, move._sd * Math.PI / 180, move._n * DIST_Q, shapes, step);
    const moveTurnRadius = move => turnRadius(move._sd * Math.PI / 180);

    // ── createSolver(): kernel-bound Solver (Component 1c) ────────────────
    function createSolver() { return makeSolver(kernel); }

    const kernel = {
      config, spec,
      // private intra-component internals (for the bundled Solver / low-level use)
      _wheelbase: wheelbase, _carRadius: carRadius, _centerOffset: centerOffset,
      _precomp,
      advancePose, turnRadius, arcCenter, carPolygon, carShape,
      poseCollides, simulateMove, sweepCollides,
      // gameplay surface
      goalPolygon, inGoal, parkingClearance, distToGoalBoundary, distCarToGoal,
      applyMove, moveTurnRadius, createSolver,
    };
    return kernel;
  }

  /* ─── Solver (Component 1c) — bound to a kernel ─────────────────────────── */

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
    SAMPLE_STEP, STEER_Q, DIST_Q, VEHICLES, vehicleSpecFor,
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
