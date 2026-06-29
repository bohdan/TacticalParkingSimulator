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
import { Geom2D } from './geometry2d.js';
import type { Point, Shape } from './geometry2d.js';

export interface Pose { x: number; y: number; h: number; }
export interface VehicleSpec { len: number; wid: number; wb: number; rOver: number; fOver: number; maxSteer: number; }
export interface PhysicsKernelConfig {
  vehicle: VehicleSpec;
  sampleStep: number;
  steerQ: number;
  distQ: number;
}
export interface Goal { cx: number; cy: number; w: number; h: number; heads: number[]; tol: number; ang?: number; }
export interface SimResult { pts: Pose[]; end: Pose; hit: { pose: Pose; point: Point } | null; }
export type MoveHandle = Readonly<{ _sd: number; _n: number }>;
export interface KernelPrecomp {
  steers: number[];
  steerTable: ReadonlyArray<{ R: number; absR: number; cornerRadii: number[]; rMin: number; rMax: number }>;
  steerMap: Map<number, number>;
  _siKey: (sd: number) => number;
  dDx: Float64Array;
  dDy: Float64Array;
  dDh: Float64Array;
  MAX_ND: number;
}
export type KernelFactory = (config: PhysicsKernelConfig, makeSolver: (k: KernelInstance) => any) => KernelInstance;

export interface KernelInstance {
  config: PhysicsKernelConfig;
  spec: VehicleSpec;
  _wheelbase: number;
  _carRadius: number;
  _centerOffset: number;
  _precomp: Readonly<KernelPrecomp>;
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
  readonly Shape = Object.freeze({
    rectangle: (x: number, y: number, w: number, h: number): Shape =>
      PhysicsCore.makeShape(Geom2D.rectanglePolygon(x, y, w, h)),
    orientedBox: (cx: number, cy: number, w: number, h: number, ang: number): Shape =>
      PhysicsCore.makeShape(Geom2D.orientedBoxPolygon(cx, cy, w, h, ang)),
    polygon: (points: Point[]): Shape =>
      PhysicsCore.makeShape(points),
  });
  Move: (steeringDegrees: number, signedDistanceMeters: number) => Readonly<{ _sd: number; _n: number }>;

  private _kernelFactory: KernelFactory | null = null;
  private _solverFactory: ((kernel: KernelInstance) => any) | null = null;

  constructor() {
    this.VEHICLES = Object.freeze(
      Object.fromEntries(Object.keys(this.VEHICLE_DEFS).map((k): [string, VehicleSpec] => [k, this.vehicleSpecFor(k)]))
    );

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
  rad(d: number): number { return d * Math.PI / 180; }
  deg(r: number): number { return r * 180 / Math.PI; }
  clamp(v: number, a: number, b: number): number { return Math.min(b, Math.max(a, v)); }
  normalizeAngle(a: number): number {
    a %= 2 * Math.PI;
    if (a > Math.PI) a -= 2 * Math.PI;
    if (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  // ─── Geometry comes from Geom2D (geometry2d.ts) ─────────────────────────
  private static makeShape(poly: Point[]): Shape {
    return { poly, bc: Geom2D.polygonBoundingCircle(poly) };
  }

  shapesCollide(a: Shape, b: Shape): boolean {
    return Geom2D.polygonsCollide(a.poly, b.poly);
  }

  // ─── Move (control intent; vehicle-independent; encapsulated) ────────────
  moveTurnDirection(m: { _sd: number }): 'S' | 'L' | 'R' { return Math.abs(m._sd) < 0.1 ? 'S' : (m._sd > 0 ? 'L' : 'R'); }
  moveDirection(m: { _n: number }): 1 | -1 { return m._n >= 0 ? 1 : -1; }
  moveDistance(m: { _n: number }): number { return Math.round(m._n * this.DIST_Q * 100) / 100; }
  moveSteeringDegrees(m: { _sd: number }): number { return m._sd; }

  // Serialization — format owned by physics; callers treat the string as opaque.
  moveToString(m: { _sd: number; _n: number }): string {
    return `${m._sd}:${this.moveDistance(m)}`;
  }
  parseMove(s: string): MoveHandle {
    const [d, dist] = s.split(':').map(Number);
    return this.Move(d, dist);
  }
  moveSequenceToString(moves: Array<{ _sd: number; _n: number }>): string {
    return moves.map((m) => this.moveToString(m)).join(';');
  }
  parseMoveSequence(s: string): MoveHandle[] {
    return s ? s.split(';').filter(Boolean).map((value) => this.parseMove(value)) : [];
  }

  // ─── Per-level kernel config ────────────────────────────────────────────
  physicsConfigForLevel(def: { vehicle?: string } | null | undefined): PhysicsKernelConfig {
    return Object.freeze({
      vehicle: this.vehicleSpecFor(def && def.vehicle),
      sampleStep: this.SAMPLE_STEP,
      steerQ: this.STEER_Q,
      distQ: this.DIST_Q,
    });
  }

  // ─── PhysicsKernel(config): delegates to the registered KernelFactory ────
  // Call Physics._useKernel(factory) (e.g. via physics-simple-2d.ts) before use.
  PhysicsKernel(config: PhysicsKernelConfig): KernelInstance {
    if (!this._kernelFactory)
      throw new Error('No kernel factory registered — import ./physics-simple-2d.js before calling PhysicsKernel().');
    return this._kernelFactory(config, (k) => this.makeSolver(k));
  }

  _useKernel(factory: KernelFactory): void { this._kernelFactory = factory; }
  _useSolver(factory: (kernel: KernelInstance) => any): void { this._solverFactory = factory; }
  private makeSolver(kernel: KernelInstance): any {
    if (!this._solverFactory)
      throw new Error('Solver not loaded — import ./solver.ts before calling createSolver().');
    return this._solverFactory(kernel);
  }
}

export const Physics = new PhysicsCore();
