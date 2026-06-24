'use strict';
/*
 * physics-compat.js — the legacy physics.js global surface, reimplemented as thin
 * delegation to the refactored components: PhysicsKernel (physics-kernel.js), Geom2D
 * (geometry2d.js) and Scene (scene.js).
 *
 * It lets the large, unchanged consumers (game.js, editor.html) keep using the flat,
 * mutable-global API they were written against, while the DUPLICATE algorithms that
 * used to live in physics.js are deleted — the single source of truth is now the
 * components. No game/editor logic changes; behaviour is preserved by construction:
 * every function below forwards to the component the smoke/parity tests pin to the old
 * physics.js numerically.
 *
 * Load as a classic browser script AFTER geometry2d/physics-kernel/scene (read as
 * globals). Also require()-able in Node (parity tests).
 */
const _Phys  = (typeof Physics !== 'undefined') ? Physics : require('./physics-kernel.js');
const _Geom  = (typeof Geom2D  !== 'undefined') ? Geom2D  : require('./geometry2d.js');
const _Scene = (typeof Scene   !== 'undefined') ? Scene   : require('./scene.js');

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
function setVehicle(name) {
  _kernel = _Phys.PhysicsKernel(_Phys.physicsConfigForLevel({ vehicle: name || 'default' }));
  Object.assign(CAR, _kernel.spec);   // mutate in place so existing CAR references stay valid
  return CAR;
}

/* ─── Math helpers ─────────────────────────────────────────────────────────── */
const rad = _Phys.rad, deg = _Phys.deg, clamp = _Phys.clamp, normAng = _Phys.normalizeAngle;

/* ─── Kinematics / collision / goal (delegate to the CURRENT kernel) ───────── */
const advance      = (p, steer, s) => _kernel.advancePose(p, steer, s);
const carPoly      = (p, inf = 0)  => _kernel.carPolygon(p, inf);
const simulateMove = (start, steer, dist, obstacles, step) => _kernel.simulateMove(start, steer, dist, obstacles, step);
const inGoal       = (pose, goal)  => _kernel.inGoal(pose, goal);
const goalPoly     = (g)           => _kernel.goalPolygon(g);

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
function buildLevel(def) {
  const lvl = _Scene.buildLevel(def);
  const obstacles = lvl.obstacles.map(o =>
    Object.assign({}, o, { poly: o.shape.poly, bc: o.shape.bc }));
  return Object.assign({}, lvl, { obstacles });
}

if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    CAR, SEDAN, VEHICLES, SAMPLE_STEP, setVehicle,
    rad, deg, clamp, normAng,
    advance, carPoly, simulateMove, inGoal, goalPoly,
    polysCollide, pointInPoly, ptSegDist, rectPoly, obbPoly,
    centroid, contactPoint, convexHull, polyBC, buildLevel,
  };
}
