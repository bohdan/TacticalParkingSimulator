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
 * encapsulated type: its `_steer` (radians) / `_dist` are private (underscore-prefixed) and
 * read only via the move* helpers (degrees in/out, 'L'|'R'|'S' TurnDirection, wire string).
 *
 * Generic 2D geometry (SAT, hull, point-in-polygon, segment math) lives in geometry2d.ts
 * (`Geom2D`); this file owns only the vehicle/kinematics/collision-orchestration layer.
 *
 * Works as a browser global and as a Node module (for tests).
 */
import { Geom2D } from './geometry2d.js';

export class PhysicsCore {
  private readonly G: typeof Geom2D;
  readonly SAMPLE_STEP = 0.06;

  readonly VEHICLE_DEFS = {
    default: { len: 4.4,  wid: 1.8,  wb: 2.7,   rOver: 0.85, maxSteer: 35 },
    miata:   { len: 3.97, wid: 1.72, wb: 2.265, rOver: 0.73, maxSteer: 40 },
    bus:     { len: 12.0, wid: 2.55, wb: 6.5,   rOver: 2.5,  maxSteer: 45 },
    tractor: { len: 3.8,  wid: 1.95, wb: 2.15,  rOver: 0.45, maxSteer: 52 },
  };

  readonly VEHICLES: Readonly<Record<string, any>>;
  Shape: any;
  Move: (steeringDegrees: number, signedDistanceMeters: number) => Readonly<{ _steer: number; _dist: number }>;

  private _solverFactory: ((kernel: any) => any) | null = null;

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

    this.Move = (steeringDegrees: number, signedDistanceMeters: number) =>
      Object.freeze({ _steer: this.rad(steeringDegrees), _dist: signedDistanceMeters });
  }

  // vehicleSpecFor(type) → VehicleSpec (fOver filled). ⇐ game/editor/scene/kernel.
  vehicleSpecFor(type: string) {
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
  private makeShape(poly: Array<{ x: number; y: number }>) {
    return { poly, bc: this.G.polygonBoundingCircle(poly) };
  }

  shapesCollide(a: { poly: any }, b: { poly: any }) {
    return this.G.polygonsCollide(a.poly, b.poly);
  }

  // ─── Move (control intent; vehicle-independent; encapsulated) ────────────
  moveTurnDirection(m: { _steer: number }) { return Math.abs(m._steer) < 1e-4 ? 'S' : (m._steer > 0 ? 'L' : 'R'); }
  moveDirection(m: { _dist: number }) { return m._dist >= 0 ? 1 : -1; }
  moveDistance(m: { _dist: number }) { return m._dist; }
  moveSteeringDegrees(m: { _steer: number }) { return this.deg(m._steer); }

  // Serialization — format owned by physics; callers treat the string as opaque.
  private round(v: number, q: number) { return Math.round(v * q) / q; }
  moveToString(m: { _steer: number; _dist: number }) {
    return `${this.round(this.deg(m._steer), 10)}:${this.round(m._dist, 100)}`;
  }
  parseMove(s: string) {
    const [d, dist] = s.split(':').map(Number);
    return this.Move(d, dist);
  }
  moveSequenceToString(moves: Array<{ _steer: number; _dist: number }>) {
    return moves.map((m) => this.moveToString(m)).join(';');
  }
  parseMoveSequence(s: string) {
    return s ? s.split(';').filter(Boolean).map((value) => this.parseMove(value)) : [];
  }

  // ─── Per-level kernel config ────────────────────────────────────────────
  physicsConfigForLevel(def: { vehicle?: string } | null | undefined) {
    return Object.freeze({
      vehicle: this.vehicleSpecFor(def && def.vehicle),
      sampleStep: this.SAMPLE_STEP,
    });
  }

  // ─── PhysicsKernel(config): the per-level instance (Component 1b) ────────
  PhysicsKernel(config: { vehicle: any; sampleStep?: number }) {
    const spec = config.vehicle;
    const sampleStep = config.sampleStep || this.SAMPLE_STEP;
    const wheelbase = spec.wb;
    const carRadius = 0.5 * Math.hypot(spec.len, spec.wid) + 0.02;
    const centerOffset = (spec.wb + spec.fOver - spec.rOver) / 2;

    const G = this.G;
    const rad = this.rad.bind(this);
    const normalizeAngle = this.normalizeAngle.bind(this);
    const makeShape = (poly: Array<{ x: number; y: number }>) => ({ poly, bc: G.polygonBoundingCircle(poly) });
    const { rectanglePolygon, orientedBoxPolygon, pointInPolygon, polygonsCollide,
      convexHull, polygonBoundingCircle, pointToSegmentDistance } = G;
    const contactPoint = G.contactPoint.bind(G);

    // ── kinematics (HOT) ──────────────────────────────────────────────────
    function advancePose(p: any, steer: number, s: number) {
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
    function arcCenter(p: any, steer: number) {
      if (Math.abs(steer) < 1e-4) return null;
      const R = wheelbase / Math.tan(steer);
      return { x: p.x - Math.sin(p.h) * R, y: p.y + Math.cos(p.h) * R };
    }
    function carPolygon(p: any, inf = 0) {
      const c = Math.cos(p.h), s = Math.sin(p.h);
      const x0 = -spec.rOver - inf, x1 = spec.wb + spec.fOver + inf;
      const y0 = -spec.wid / 2 - inf, y1 = spec.wid / 2 + inf;
      const pt = (x: number, y: number) => ({ x: p.x + c * x - s * y, y: p.y + s * x + c * y });
      return [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)];
    }
    function carShape(p: any) { return makeShape(carPolygon(p)); }

    function goalPolygon(goal: any) {
      return goal.ang
        ? orientedBoxPolygon(goal.cx, goal.cy, goal.w, goal.h, goal.ang)
        : rectanglePolygon(goal.cx - goal.w / 2, goal.cy - goal.h / 2, goal.w, goal.h);
    }
    function inGoal(pose: any, goal: any) {
      const okHead = goal.heads.some(
        (hd: number) => Math.abs(normalizeAngle(pose.h - rad(hd))) <= rad(goal.tol));
      if (!okHead) return false;
      const zone = goalPolygon(goal);
      return carPolygon(pose).every((v: any) => pointInPolygon(v, zone));
    }

    function parkingClearance(pose: any, goal: any) {
      const cp = carPolygon(pose), zone = goalPolygon(goal);
      let minGap = Infinity;
      for (const v of cp)
        for (let j = 0; j < zone.length; j++) {
          const a = zone[j], b = zone[(j + 1) % zone.length];
          minGap = Math.min(minGap, pointToSegmentDistance(v.x, v.y, a.x, a.y, b.x, b.y));
        }
      return isFinite(minGap) ? minGap : 0;
    }
    function distToGoalBoundary(pose: any, goal: any) {
      const zone = goalPolygon(goal);
      let d = Infinity;
      for (let j = 0; j < zone.length; j++) {
        const a = zone[j], b = zone[(j + 1) % zone.length];
        d = Math.min(d, pointToSegmentDistance(pose.x, pose.y, a.x, a.y, b.x, b.y));
      }
      return pointInPolygon({ x: pose.x, y: pose.y }, zone) ? -d : d;
    }
    function distCarToGoal(pose: any, goal: any) {
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

    function poseCollides(x: number, y: number, h: number, shapes: Array<any>) {
      const ccx = x + Math.cos(h) * centerOffset, ccy = y + Math.sin(h) * centerOffset;
      let poly: any = null;
      for (let i = 0; i < shapes.length; i++) {
        const o = shapes[i], bc = o.bc;
        const dx = ccx - bc.x, dy = ccy - bc.y, rr = carRadius + bc.r;
        if (dx * dx + dy * dy > rr * rr) continue;
        if (!poly) poly = carPolygon({ x, y, h });
        if (polygonsCollide(poly, o.poly)) return true;
      }
      return false;
    }
    function simulateMove(start: any, steer: number, dist: number, shapes: Array<any>, step: number) {
      const n = Math.max(2, Math.ceil(Math.abs(dist) / (step || sampleStep)));
      const pts = [start];
      let hit: any = null;
      const stepLen = Math.abs(dist) / n;
      let prevPoly = carPolygon(start);
      for (let i = 1; i <= n; i++) {
        const p = advancePose(start, steer, dist * i / n);
        const curPoly = carPolygon(p);
        let swept: any = null;
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

    const applyMove = (pose: any, move: any, shapes: Array<any>, step: number) =>
      simulateMove(pose, move._steer, move._dist, shapes, step);
    const moveTurnRadius = (move: any) => turnRadius(move._steer);

    function createSolver() { return makeSolver(kernel); }

    const kernel = {
      config, spec,
      _wheelbase: wheelbase, _carRadius: carRadius, _centerOffset: centerOffset,
      advancePose, turnRadius, arcCenter, carPolygon, carShape,
      poseCollides, simulateMove,
      goalPolygon, inGoal, parkingClearance, distToGoalBoundary, distCarToGoal,
      applyMove, moveTurnRadius, createSolver,
    };
    return kernel;
  }

  _useSolver(factory: (kernel: any) => any) { this._solverFactory = factory; }
  private makeSolver(kernel: any) {
    if (!this._solverFactory)
      throw new Error('Solver not loaded — import ./solver.js before calling createSolver().');
    return this._solverFactory(kernel);
  }
}

export const Physics = new PhysicsCore();
