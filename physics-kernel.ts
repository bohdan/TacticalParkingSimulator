'use strict';
/*
 * physics-kernel.ts — the PhysicsKernel interface, implemented.
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
 * move* helpers (degrees in/out, 'L'|'R'|'S' TurnDirection, wire string). This integer
 * representation enables O(1) lookup into the kernel's precomputed steer table.
 *
 * Generic 2D geometry (SAT, hull, point-in-polygon, segment math) lives in geometry2d.ts
 * (`Geom2D`); this file owns only the vehicle/kinematics/collision-orchestration layer.
 *
 * Works as a browser global and as a Node module (for tests).
 */
import { Geom2D, Point, BoundingCircle, Shape } from './geometry2d.js';

export interface Pose { x: number; y: number; h: number; }
export interface VehicleSpec { len: number; wid: number; wb: number; rOver: number; fOver: number; maxSteer: number; }
export interface PhysicsKernelConfig { vehicle: VehicleSpec; sampleStep?: number; }
export interface Goal { cx: number; cy: number; w: number; h: number; heads: number[]; tol: number; ang?: number; }
export interface SimResult { pts: Pose[]; end: Pose; hit: { pose: Pose; point: Point } | null; }
export type MoveHandle = Readonly<{ _sd: number; _n: number }>;
export interface KernelInstance {
  config: PhysicsKernelConfig;
  spec: VehicleSpec;
  _wheelbase: number;
  _carRadius: number;
  _centerOffset: number;
  _precomp: any;
  advancePose(p: Pose, steer: number, s: number): Pose;
  turnRadius(steer: number): number;
  arcCenter(p: Pose, steer: number): Point | null;
  carPolygon(p: Pose, inf?: number): Point[];
  carShape(p: Pose): Shape;
  poseCollides(x: number, y: number, h: number, shapes: Shape[]): boolean;
  simulateMove(start: Pose, steer: number, dist: number, shapes: Shape[], step?: number): SimResult;
  sweepCollides(start: Pose, steer: number, dist: number, shapes: Shape[]): Shape | null;
  goalPolygon(goal: Goal): Point[];
  inGoal(pose: Pose, goal: Goal): boolean;
  parkingClearance(pose: Pose, goal: Goal): number;
  distToGoalBoundary(pose: Pose, goal: Goal): number;
  distCarToGoal(pose: Pose, goal: Goal): number;
  applyMove(pose: Pose, move: MoveHandle, shapes: Shape[], step?: number): SimResult;
  moveTurnRadius(move: MoveHandle): number;
  createSolver(): any;
}


export class PhysicsCore {
  private readonly G: typeof Geom2D;
  readonly SAMPLE_STEP = 0.06;
  readonly STEER_Q = 0.2;   // deg, player steering input grid
  readonly DIST_Q  = 0.05;  // m,   player distance input grid

  readonly VEHICLE_DEFS = {
    default: { len: 4.4,  wid: 1.8,  wb: 2.7,   rOver: 0.85, maxSteer: 35 },
    miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
    bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
    tractor: { len: 3.8,  wid: 1.95, wb: 2.15,  rOver: 0.45, maxSteer: 52 },
  };

  readonly VEHICLES: Readonly<Record<string, VehicleSpec>>;
  readonly Shape: {
    rectangle(x: number, y: number, w: number, h: number): Shape;
    orientedBox(cx: number, cy: number, w: number, h: number, ang: number): Shape;
    polygon(points: Point[]): Shape;
  };
  Move: (steeringDegrees: number, signedDistanceMeters: number) => Readonly<{ _sd: number; _n: number }>;

  private _solverFactory: ((kernel: KernelInstance) => any) | null = null;

  constructor(G: typeof Geom2D = Geom2D) {
    this.G = G;
    this.VEHICLES = Object.freeze(Object.keys(this.VEHICLE_DEFS).reduce((o: Record<string, any>, k: string) => {
      o[k] = this.vehicleSpecFor(k);
      return o;
    }, {}));

    this.Shape = Object.freeze({
      rectangle: (x: number, y: number, w: number, h: number) => this.makeShape(this.G.rectanglePolygon(x, y, w, h)),
      orientedBox: (cx: number, cy: number, w: number, h: number, ang: number) =>
        this.makeShape(this.G.orientedBoxPolygon(cx, cy, w, h, ang)),
      polygon: (points: Array<{ x: number; y: number }>) => this.makeShape(points),
    });

    this.Move = (steeringDegrees: number, signedDistanceMeters: number) => {
      const _sd = Math.round(steeringDegrees / this.STEER_Q) * this.STEER_Q;
      const _n  = Math.round(signedDistanceMeters / this.DIST_Q);
      return Object.freeze({ _sd, _n });
    };
  }

  // vehicleSpecFor(type) → VehicleSpec (fOver filled). ⇐ game/editor/scene/kernel.
  vehicleSpecFor(type: string): VehicleSpec {
    const v = this.VEHICLE_DEFS[type] || this.VEHICLE_DEFS.default;
    return Object.freeze({
      len: v.len, wid: v.wid, wb: v.wb, rOver: v.rOver,
      fOver: v.len - v.wb - v.rOver, maxSteer: v.maxSteer,
    });
  }

  // ─── PhysicsStatics: math helpers ───────────────────────────────────────
  rad(d: number) { return d * Math.PI / 180; }
  deg(r: number) { return r * 180 / Math.PI; }
  clamp(v: number, a: number, b: number) { return Math.min(b, Math.max(a, v)); }
  normalizeAngle(a: number) {
    a %= 2 * Math.PI;
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  // ─── Geometry comes from Geom2D (geometry2d.js) ─────────────────────────
  private makeShape(poly: Point[]): Shape {
    return { poly, bc: this.G.polygonBoundingCircle(poly) };
  }

  shapesCollide(a: Shape, b: Shape): boolean {
    return this.G.polygonsCollide(a.poly, b.poly);
  }

  // ─── Move (control intent; vehicle-independent; encapsulated) ────────────
  moveTurnDirection(m: { _sd: number }) { return Math.abs(m._sd) < 0.1 ? 'S' : (m._sd > 0 ? 'L' : 'R'); }
  moveDirection(m: { _n: number }) { return m._n >= 0 ? 1 : -1; }
  moveDistance(m: { _n: number }) { return Math.round(m._n * this.DIST_Q * 100) / 100; }
  moveSteeringDegrees(m: { _sd: number }) { return m._sd; }

  // Serialization — format owned by physics; callers treat the string as opaque.
  moveToString(m: { _sd: number; _n: number }) {
    return `${m._sd}:${this.moveDistance(m)}`;
  }
  parseMove(s: string) {
    const [d, dist] = s.split(':').map(Number);
    return this.Move(d, dist);
  }
  moveSequenceToString(moves: Array<{ _sd: number; _n: number }>) {
    return moves.map((m) => this.moveToString(m)).join(';');
  }
  parseMoveSequence(s: string) {
    return s ? s.split(';').filter(Boolean).map((value) => this.parseMove(value)) : [];
  }

  // ─── Per-level kernel config ────────────────────────────────────────────
  physicsConfigForLevel(def: { vehicle?: string } | null | undefined): PhysicsKernelConfig {
    return Object.freeze({
      vehicle: this.vehicleSpecFor(def && def.vehicle),
      sampleStep: this.SAMPLE_STEP,
    });
  }

  // ─── PhysicsKernel(config): the per-level instance (Component 1b) ────────
  PhysicsKernel(config: PhysicsKernelConfig): KernelInstance {
    const _makeSolver = (k: KernelInstance) => this.makeSolver(k);
    const spec = config.vehicle;
    const sampleStep = config.sampleStep || this.SAMPLE_STEP;
    const wheelbase = spec.wb;
    const carRadius = 0.5 * Math.hypot(spec.len, spec.wid) + 0.02;
    const centerOffset = (spec.wb + spec.fOver - spec.rOver) / 2;
    const STEER_Q = this.STEER_Q, DIST_Q = this.DIST_Q;

    const G = this.G;
    const rad = this.rad.bind(this);
    const deg = this.deg.bind(this);
    const normalizeAngle = this.normalizeAngle.bind(this);
    const makeShape = (poly: Point[]): Shape => ({ poly, bc: G.polygonBoundingCircle(poly) });
    const { rectanglePolygon, orientedBoxPolygon, pointInPolygon, polygonsCollide,
      convexHull, polygonBoundingCircle, pointToSegmentDistance } = G;
    const contactPoint = G.contactPoint.bind(G);

    // ── kinematics (HOT) ──────────────────────────────────────────────────
    function advancePose(p: Pose, steer: number, s: number): Pose {
      if (Math.abs(steer) < 1e-4)
        return { x: p.x + Math.cos(p.h) * s, y: p.y + Math.sin(p.h) * s, h: p.h };
      const R = wheelbase / Math.tan(steer);
      const cx = p.x - Math.sin(p.h) * R, cy = p.y + Math.cos(p.h) * R;
      const h2 = p.h + s / R;
      return { x: cx + Math.sin(h2) * R, y: cy - Math.cos(h2) * R, h: h2 };
    }
    function turnRadius(steer: number) {
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
    const locCorners: [number, number][] = [
      [-spec.rOver,          -spec.wid / 2],
      [spec.wb + spec.fOver, -spec.wid / 2],
      [spec.wb + spec.fOver,  spec.wid / 2],
      [-spec.rOver,           spec.wid / 2],
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

    const _precomp = Object.freeze({ steers, steerTable, steerMap, _siKey, dDx, dDy, dDh, MAX_ND });

    // ── Analytic swept-arc collision ──────────────────────────────────────
    const TWO_PI = 2 * Math.PI;
    function inSweepArc(phi: number, phi0: number, Delta: number) {
      if (Math.abs(Delta) < 1e-9) return false;
      if (Delta > 0)
        return ((phi - phi0) % TWO_PI + TWO_PI) % TWO_PI <= Delta + 1e-9;
      else
        return ((phi0 - phi) % TWO_PI + TWO_PI) % TWO_PI <= -Delta + 1e-9;
    }

    function sweepCollides(start: Pose, steer: number, dist: number, shapes: Shape[]): Shape | null {
      if (!shapes.length || Math.abs(dist) < 1e-9) return null;
      const startPoly = carPolygon(start);
      const end = advancePose(start, steer, dist);
      const endPoly = carPolygon(end);
      const isStr = Math.abs(steer) < 1e-4;
      const R = isStr ? Infinity : wheelbase / Math.tan(steer);
      const Delta = isStr ? 0 : dist / R;

      let Cx: number, Cy: number, cornBear: number[], cornR: number[], rMin: number, rMax: number;
      if (!isStr) {
        Cx = start.x - Math.sin(start.h) * R;
        Cy = start.y + Math.cos(start.h) * R;
        const si = steerMap.get(_siKey(deg(steer)));
        const sg = si != null ? steerTable[si] : null;
        cornBear = startPoly.map((v: any) => Math.atan2(v.y - Cy, v.x - Cx));
        cornR = sg ? sg.cornerRadii : startPoly.map((v: any) => Math.hypot(v.x - Cx, v.y - Cy));
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
        (hd: number) => Math.abs(normalizeAngle(pose.h - rad(hd))) <= rad(goal.tol));
      if (!okHead) return false;
      const zone = goalPolygon(goal);
      return carPolygon(pose).every((v: any) => pointInPolygon(v, zone));
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
        if (!poly) poly = carPolygon({ x, y, h });
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

    function createSolver() { return _makeSolver(kernel); }

    const kernel = {
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

  _useSolver(factory: (kernel: KernelInstance) => any) { this._solverFactory = factory; }
  private makeSolver(kernel: KernelInstance): any {
    if (!this._solverFactory)
      throw new Error('Solver not loaded — import ./solver.js before calling createSolver().');
    return this._solverFactory(kernel);
  }
}

export const Physics = new PhysicsCore();
