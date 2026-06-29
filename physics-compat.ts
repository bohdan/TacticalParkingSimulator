/*
 * physics-compat.js — the legacy physics.js global surface, reimplemented as thin
 * delegation to the refactored components: PhysicsKernel (physics-kernel.js), Geom2D
 * (geometry2d.js) and Scene (scene.js).
 *
 * It lets the large consumers (game.js, editor.js) keep using the flat, named API they
 * were written against — they `import` these names — while the DUPLICATE algorithms that
 * used to live in physics.js are deleted: the single source of truth is now the
 * components. Behaviour is preserved by construction: every function below forwards to
 * the component the parity tests pin to the old physics.js numerically.
 *
 * ES module: imports the components and re-exports the legacy surface.
 */
import { Physics as _Phys } from './physics-kernel.js';
import type { Pose, VehicleSpec, Goal, SimResult, MoveHandle } from './physics-kernel.js';
import { Geom2D as _Geom } from './geometry2d.js';
import type { Point, Shape, BoundingCircle } from './geometry2d.js';
import { buildLevel as _sceneBuildLevel } from './scene.js';
import type { SceneObstacle, BuiltLevel } from './scene.js';

/* ─── Vehicle registry / current vehicle ──────────────────────────────────── */
// CAR is the MUTABLE current-vehicle dims object: game/editor read CAR.wb/len/… and
// hold references to it, so setVehicle syncs it IN PLACE. SEDAN/VEHICLES mirror the
// old constants. (VEHICLES is a shallow, unfrozen copy so older mutating callers
// don't throw against the frozen component registry.)
const SEDAN = Object.assign({}, _Phys.vehicleSpecFor('default'));
const VEHICLES = Object.keys(_Phys.VEHICLES).reduce(
  (o, k) => (o[k] = Object.assign({}, _Phys.VEHICLES[k]), o), {});
const SAMPLE_STEP = _Phys.SAMPLE_STEP;
const CAR = Object.assign({}, _Phys.vehicleSpecFor('default'));

let _kernel = _Phys.PhysicsKernel(_Phys.physicsConfigForLevel({ vehicle: 'default' }));
function setVehicle(name: string): VehicleSpec {
  _kernel = _Phys.PhysicsKernel(_Phys.physicsConfigForLevel({ vehicle: name || 'default' }));
  Object.assign(CAR, _kernel.spec);   // mutate in place so existing CAR references stay valid
  return CAR;
}

/* ─── Math helpers ─────────────────────────────────────────────────────────── */
const rad = _Phys.rad, deg = _Phys.deg, clamp = _Phys.clamp, normAng = _Phys.normalizeAngle;

/* ─── Kinematics / collision / goal (delegate to the CURRENT kernel) ───────── */
const advance      = (p: Pose, steer: number, s: number): Pose => _kernel.advancePose(p, steer, s);
const carPoly      = (p: Pose, inf = 0): Point[] => _kernel.carPolygon(p, inf);
const simulateMove = (start: Pose, steer: number, dist: number, obstacles: Shape[], step?: number): SimResult => _kernel.simulateMove(start, steer, dist, obstacles, step);
const inGoal       = (pose: Pose, goal: Goal): boolean => _kernel.inGoal(pose, goal);
const goalPoly     = (g: Goal): Point[] => _kernel.goalPolygon(g);
const parkingClearance   = (pose: Pose, goal: Goal): number => _kernel.parkingClearance(pose, goal);
const distToGoalBoundary = (pose: Pose, goal: Goal): number => _kernel.distToGoalBoundary(pose, goal);
const distCarToGoal      = (pose: Pose, goal: Goal): number => _kernel.distCarToGoal(pose, goal);

/* ─── Generic 2D geometry (delegate to Geom2D) ─────────────────────────────── */
const polysCollide = _Geom.polygonsCollide;
const pointInPoly  = _Geom.pointInPolygon;
const ptSegDist    = _Geom.pointToSegmentDistance;
const rectPoly     = _Geom.rectanglePolygon;
const obbPoly      = _Geom.orientedBoxPolygon;
const centroid     = _Geom.centroid;
const contactPoint = _Geom.contactPoint;
const convexHull   = _Geom.convexHull;
const polyBC       = _Geom.polygonBoundingCircle;

/* ─── Level building (Scene + flatten Shapes to the legacy obstacle shape) ─── */
// Old buildLevel obstacles exposed `.poly` (and lazily `.bc`); Scene exposes `.shape`.
// Flatten so game/editor's `o.poly` reads and simulateMove(o.poly/o.bc) keep working.
function buildLevel(def: Parameters<typeof _sceneBuildLevel>[0]): ReturnType<typeof _sceneBuildLevel> & { obstacles: (SceneObstacle & { poly: Point[]; bc: import('./geometry2d.js').BoundingCircle })[] } {
  const lvl = _sceneBuildLevel(def);
  const obstacles = lvl.obstacles.map(o =>
    Object.assign({}, o, { poly: o.shape.poly, bc: o.shape.bc }));
  return Object.assign({}, lvl, { obstacles });
}

export {
  CAR, SEDAN, VEHICLES, SAMPLE_STEP, setVehicle,
  rad, deg, clamp, normAng,
  advance, carPoly, simulateMove, inGoal, goalPoly,
  parkingClearance, distToGoalBoundary, distCarToGoal,
  polysCollide, pointInPoly, ptSegDist, rectPoly, obbPoly,
  centroid, contactPoint, convexHull, polyBC, buildLevel,
};
