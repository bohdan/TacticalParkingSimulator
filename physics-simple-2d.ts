/*
 * physics-simple-2d.ts — bicycle-model 2D kernel (Component 1b implementation).
 *
 * Implements KernelFactory for a rigid-body single-track (bicycle) vehicle:
 *   advance    = Ackermann / constant-steer arc integration
 *   collision  = analytic swept-arc test against convex polygon obstacles
 *   goal       = axis-aligned or oriented rectangular zone + heading tolerance
 *
 * Self-registers with the Physics singleton on import (analogous to solver.ts
 * registering via Physics._useSolver). Callers that need PhysicsKernel must
 * import this module before calling Physics.PhysicsKernel():
 *
 *   import './physics-simple-2d.js';  // registers factory
 *   const kernel = Physics.PhysicsKernel(config);
 *
 * Depends on geometry2d.ts (Geom2D). Type-only imports from physics-kernel.ts
 * are erased at compile time (no runtime circular dependency).
 */
import { Physics } from './physics-kernel.js';
import type {
  Pose, Goal, SimResult, MoveHandle, KernelPrecomp, KernelInstance, PhysicsKernelConfig,
} from './physics-kernel.js';
import { Geom2D } from './geometry2d.js';
import type { Point, Shape } from './geometry2d.js';

const _rad = (d: number): number => d * Math.PI / 180;
const _deg = (r: number): number => r * 180 / Math.PI;
function _normalizeAngle(a: number): number {
  a %= 2 * Math.PI;
  if (a > Math.PI) a -= 2 * Math.PI;
  if (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function createSimple2DKernel(
  config: PhysicsKernelConfig,
  makeSolver: (k: KernelInstance) => any,
): KernelInstance {
  const { vehicle: spec, sampleStep, steerQ: STEER_Q, distQ: DIST_Q } = config;
  const wheelbase = spec.wb;
  const carRadius = 0.5 * Math.hypot(spec.len, spec.wid) + 0.02;
  const centerOffset = (spec.wb + spec.fOver - spec.rOver) / 2;

  // Collision footprint is inset by this margin so a *clean touch* (zero-gap
  // contact — e.g. parking flush against a bumper or wall) is not a collision;
  // only a real overlap deeper than this counts. Applied uniformly to the SAT
  // poly checks and the analytic swept-arc corner radii, so every collision path
  // shares one tolerance. Far below the DIST_Q input grid, so it never affects
  // a move the player can actually express. Motion (turn radius, advance) is
  // unaffected — only the collision corners shrink.
  const COLLISION_EPS = 1e-4;   // metres (0.1 mm)

  const makeShape = (poly: Point[]): Shape => ({ poly, bc: Geom2D.polygonBoundingCircle(poly) });
  const { rectanglePolygon, orientedBoxPolygon, pointInPolygon, polygonsCollide,
    convexHull, pointToSegmentDistance, contactPoint } = Geom2D;

  // ── kinematics (HOT) ──────────────────────────────────────────────────
  function advancePose(p: Pose, steer: number, s: number): Pose {
    if (Math.abs(steer) < 1e-4)
      return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
    const R = wheelbase / Math.tan(steer);
    const cx = p.x - Math.sin(p.h) * R, cy = p.y + Math.cos(p.h) * R;
    const h2 = p.h + s / R;
    return { x: cx + Math.sin(h2) * R, y: cy - Math.cos(h2) * R, h: h2 };
  }
  function turnRadius(steer: number): number {
    return Math.abs(steer) < 1e-4 ? Infinity : wheelbase / Math.tan(steer);
  }
  function arcCenter(p: Pose, steer: number): Point | null {
    if (Math.abs(steer) < 1e-4) return null;
    const R = wheelbase / Math.tan(steer);
    return { x: p.x - Math.sin(p.h) * R, y: p.y + Math.cos(p.h) * R };
  }
  function carPolygon(p: Pose, inf = 0): Point[] {
    const c = Math.cos(p.h), s = Math.sin(p.h);
    const x0 = -spec.rOver - inf, x1 = spec.wb + spec.fOver + inf;
    const y0 = -spec.wid / 2 - inf, y1 = spec.wid / 2 + inf;
    const pt = (x: number, y: number) => ({ x: p.x + c * x - s * y, y: p.y + s * x + c * y });
    return [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
  }
  function carShape(p: Pose): Shape { return makeShape(carPolygon(p)); }

  // ── Precomputed steer geometry table ──────────────────────────────────
  // Integer key for steer in degrees: avoids float-equality Map mismatches.
  // STEER_Q=0.2 → key=Math.round(sd*10), e.g. 0.2→2, 35.0→350, -0.2→-2.
  const _siKey = (sd: number) => Math.round(sd * 10);

  const nSteerSteps = Math.round(spec.maxSteer / STEER_Q);
  const NS = 2 * nSteerSteps + 1;
  const steers: number[] = new Array(NS);
  for (let i = 0; i < NS; i++) steers[i] = (i - nSteerSteps) * STEER_Q;
  const steerMap = new Map(steers.map((sd, si) => [_siKey(sd), si]));

  // Car corners in local frame (rear axle at origin, heading = +x direction).
  // Inset by COLLISION_EPS so the precomputed corner radii / inner-outer radii
  // match the inset SAT footprint used in sweepCollides (clean touches allowed).
  const locCorners: [number, number][] = [
    [-spec.rOver + COLLISION_EPS,          -spec.wid / 2 + COLLISION_EPS],
    [spec.wb + spec.fOver - COLLISION_EPS, -spec.wid / 2 + COLLISION_EPS],
    [spec.wb + spec.fOver - COLLISION_EPS,  spec.wid / 2 - COLLISION_EPS],
    [-spec.rOver + COLLISION_EPS,           spec.wid / 2 - COLLISION_EPS],
  ];
  const locEdges = locCorners.map((c, i) => [c, locCorners[(i + 1) % 4]]);

  // Per-steer geometry: R, corner radii from C, true inner/outer radii.
  const steerTable: Array<{ R: number; absR: number; cornerRadii: number[]; rMin: number; rMax: number }> = new Array(NS);
  for (let si = 0; si < NS; si++) {
    const s = steers[si] * Math.PI / 180;
    const R = Math.abs(s) < 1e-4 ? Infinity : wheelbase / Math.tan(s);
    const cornerRadii = locCorners.map(([cx, cy]) =>
      isFinite(R) ? Math.hypot(cx, cy - R) : Infinity);
    let trueRMin = isFinite(R) ? Infinity : 0;
    if (isFinite(R)) {
      for (const [[ax, ay], [bx, by]] of locEdges as [[number,number],[number,number]][])
        trueRMin = Math.min(trueRMin, pointToSegmentDistance(0, R, ax, ay, bx, by));
    }
    steerTable[si] = Object.freeze({
      R, absR: Math.abs(R), cornerRadii, rMin: trueRMin, rMax: Math.max(...cornerRadii),
    });
  }

  // Local-frame delta table for fast arc advancement without per-step trig.
  const MAX_ND = 250;
  const dDx = new Float64Array(NS * MAX_ND);
  const dDy = new Float64Array(NS * MAX_ND);
  const dDh = new Float64Array(NS * MAX_ND);
  for (let si = 0; si < NS; si++) {
    const R = steerTable[si].R, base = si * MAX_ND;
    if (!isFinite(R)) {
      for (let n = 1; n < MAX_ND; n++) dDx[base + n] = n * DIST_Q;
    } else {
      for (let n = 1; n < MAX_ND; n++) {
        const a = n * DIST_Q / R;
        dDx[base + n] = R * Math.sin(a);
        dDy[base + n] = R * (1 - Math.cos(a));
        dDh[base + n] = a;
      }
    }
  }

  const _precomp: Readonly<KernelPrecomp> = Object.freeze({
    steers, steerTable, steerMap, _siKey, dDx, dDy, dDh, MAX_ND,
  });

  // ── Analytic swept-arc collision ──────────────────────────────────────
  const TWO_PI = 2 * Math.PI;
  function inSweepArc(phi: number, phi0: number, Delta: number): boolean {
    if (Math.abs(Delta) < 1e-9) return false;
    if (Delta > 0)
      return ((phi - phi0) % TWO_PI + TWO_PI) % TWO_PI <= Delta + 1e-9;
    else
      return ((phi0 - phi) % TWO_PI + TWO_PI) % TWO_PI <= -Delta + 1e-9;
  }

  function sweepCollides(start: Pose, steer: number, dist: number, shapes: Shape[]): Shape | null {
    if (!shapes.length || Math.abs(dist) < 1e-9) return null;
    const startPoly = carPolygon(start, -COLLISION_EPS);
    const end = advancePose(start, steer, dist);
    const endPoly = carPolygon(end, -COLLISION_EPS);
    const isStr = Math.abs(steer) < 1e-4;
    const R = isStr ? Infinity : wheelbase / Math.tan(steer);
    const Delta = isStr ? 0 : dist / R;

    let Cx: number, Cy: number, cornBear: number[], cornR: number[], rMin: number, rMax: number;
    if (!isStr) {
      Cx = start.x - Math.sin(start.h) * R;
      Cy = start.y + Math.cos(start.h) * R;
      const si = steerMap.get(_siKey(_deg(steer)));
      const sg = si != null ? steerTable[si] : null;
      cornBear = startPoly.map((v: Point) => Math.atan2(v.y - Cy, v.x - Cx));
      cornR = sg ? sg.cornerRadii : startPoly.map((v: Point) => Math.hypot(v.x - Cx, v.y - Cy));
      rMin  = sg ? sg.rMin        : cornR.reduce((a: number, b: number) => Math.min(a, b), Infinity);
      rMax  = sg ? sg.rMax        : cornR.reduce((a: number, b: number) => Math.max(a, b), 0);
    }

    for (let oi = 0; oi < shapes.length; oi++) {
      const shape = shapes[oi], obs = shape.poly, bc = shape.bc;

      if (isStr) {
        const ccx = start.x + Math.cos(start.h) * centerOffset;
        const ccy = start.y + Math.sin(start.h) * centerOffset;
        const dx = ccx - bc.x, dy = ccy - bc.y;
        const reach = carRadius + Math.abs(dist);
        if (dx * dx + dy * dy > (reach + bc.r) * (reach + bc.r)) continue;
      } else {
        const dx = Cx! - bc.x, dy = Cy! - bc.y;
        const distCC = Math.hypot(dx, dy);
        if (distCC - bc.r > rMax! + 0.01) continue;
        if (distCC + bc.r < rMin! - 0.01) continue;
      }

      if (polygonsCollide(startPoly, obs)) return shape;
      if (polygonsCollide(endPoly,   obs)) return shape;

      if (isStr) {
        if (polygonsCollide(convexHull(startPoly.concat(endPoly)), obs)) return shape;
        continue;
      }

      // Phase 2a: car corner arcs × obstacle edges
      for (let ci = 0; ci < 4; ci++) {
        const rk = cornR![ci], phi0 = cornBear![ci];
        for (let ei = 0; ei < obs.length; ei++) {
          const A = obs[ei], B = obs[(ei + 1) % obs.length];
          const ax = A.x - Cx!, ay = A.y - Cy!;
          const bx = B.x - A.x, by = B.y - A.y;
          const qa = bx * bx + by * by; if (qa < 1e-12) continue;
          const qb = 2 * (ax * bx + ay * by);
          const qc = ax * ax + ay * ay - rk * rk;
          const disc = qb * qb - 4 * qa * qc; if (disc < 0) continue;
          const sq = Math.sqrt(disc);
          for (const t of [(-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)]) {
            if (t < -1e-9 || t > 1 + 1e-9) continue;
            const phi = Math.atan2(A.y + t * by - Cy!, A.x + t * bx - Cx!);
            if (inSweepArc(phi, phi0, Delta)) return shape;
          }
        }
      }

      // Phase 2b: obstacle corners × car edges in co-rotating frame
      for (let vi = 0; vi < obs.length; vi++) {
        const Ov = obs[vi];
        const rOv = Math.hypot(Ov.x - Cx!, Ov.y - Cy!);
        if (rOv < rMin! - 0.01 || rOv > rMax! + 0.01) continue;
        const phi_Ov = Math.atan2(Ov.y - Cy!, Ov.x - Cx!);
        for (let ei = 0; ei < 4; ei++) {
          const A = startPoly[ei], B = startPoly[(ei + 1) % 4];
          const ax = A.x - Cx!, ay = A.y - Cy!;
          const bx = B.x - A.x, by = B.y - A.y;
          const qa = bx * bx + by * by; if (qa < 1e-12) continue;
          const qb = 2 * (ax * bx + ay * by);
          const qc = ax * ax + ay * ay - rOv * rOv;
          const disc = qb * qb - 4 * qa * qc; if (disc < 0) continue;
          const sq = Math.sqrt(disc);
          for (const t of [(-qb - sq) / (2 * qa), (-qb + sq) / (2 * qa)]) {
            if (t < -1e-9 || t > 1 + 1e-9) continue;
            const phi_int = Math.atan2(A.y + t * by - Cy!, A.x + t * bx - Cx!);
            if (inSweepArc(phi_int, phi_Ov, -Delta)) return shape;
          }
        }
      }
    }
    return null;
  }

  function goalPolygon(goal: Goal): Point[] {
    return goal.ang
      ? orientedBoxPolygon(goal.cx, goal.cy, goal.w, goal.h, goal.ang)
      : rectanglePolygon(goal.cx - goal.w / 2, goal.cy - goal.h / 2, goal.w, goal.h);
  }
  function inGoal(pose: Pose, goal: Goal): boolean {
    const okHead = goal.heads.some(
      (hd: number) => Math.abs(_normalizeAngle(pose.h - _rad(hd))) <= _rad(goal.tol));
    if (!okHead) return false;
    const zone = goalPolygon(goal);
    return carPolygon(pose).every((v: Point) => pointInPolygon(v, zone));
  }

  function parkingClearance(pose: Pose, goal: Goal): number {
    const cp = carPolygon(pose), zone = goalPolygon(goal);
    let minGap = Infinity;
    for (const v of cp)
      for (let j = 0; j < zone.length; j++) {
        const a = zone[j], b = zone[(j + 1) % zone.length];
        minGap = Math.min(minGap, pointToSegmentDistance(v.x, v.y, a.x, a.y, b.x, b.y));
      }
    return isFinite(minGap) ? minGap : 0;
  }
  function distToGoalBoundary(pose: Pose, goal: Goal): number {
    const zone = goalPolygon(goal);
    let d = Infinity;
    for (let j = 0; j < zone.length; j++) {
      const a = zone[j], b = zone[(j + 1) % zone.length];
      d = Math.min(d, pointToSegmentDistance(pose.x, pose.y, a.x, a.y, b.x, b.y));
    }
    return pointInPolygon({ x: pose.x, y: pose.y }, zone) ? -d : d;
  }
  function distCarToGoal(pose: Pose, goal: Goal): number {
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

  function poseCollides(x: number, y: number, h: number, shapes: Shape[]): boolean {
    const ccx = x + Math.cos(h) * centerOffset, ccy = y + Math.sin(h) * centerOffset;
    let poly: Point[] | null = null;
    for (let i = 0; i < shapes.length; i++) {
      const o = shapes[i], bc = o.bc;
      const dx = ccx - bc.x, dy = ccy - bc.y, rr = carRadius + bc.r;
      if (dx * dx + dy * dy > rr * rr) continue;
      if (!poly) poly = carPolygon({ x, y, h }, -COLLISION_EPS);
      if (polygonsCollide(poly, o.poly)) return true;
    }
    return false;
  }
  function simulateMove(start: Pose, steer: number, dist: number, shapes: Shape[], step?: number): SimResult {
    const n = Math.max(2, Math.ceil(Math.abs(dist) / (step || sampleStep)));
    const pts = [start];
    let hit: SimResult['hit'] = null;
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

  const applyMove = (pose: Pose, move: MoveHandle, shapes: Shape[], step?: number): SimResult =>
    simulateMove(pose, move._sd * Math.PI / 180, move._n * DIST_Q, shapes, step);
  const moveTurnRadius = (move: MoveHandle): number => turnRadius(move._sd * Math.PI / 180);

  function createSolver(): any { return makeSolver(kernel); }

  const kernel: KernelInstance = {
    config, spec,
    _wheelbase: wheelbase, _carRadius: carRadius, _centerOffset: centerOffset,
    _precomp,
    advancePose, turnRadius, arcCenter, carPolygon, carShape,
    poseCollides, simulateMove, sweepCollides,
    goalPolygon, inGoal, parkingClearance, distToGoalBoundary, distCarToGoal,
    applyMove, moveTurnRadius, createSolver,
  };
  return kernel;
}

// Register this factory as the default kernel implementation.
// Mirrors how solver.ts calls Physics._useSolver(makeSolver).
Physics._useKernel(createSimple2DKernel);
