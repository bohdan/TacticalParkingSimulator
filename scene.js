'use strict';
/*
 * scene.js — the Scene component (Component 2).
 *
 * Turns a level definition into obstacles that pair render/semantic metadata with an opaque
 * collision Shape, and owns goal logic. The physics kernel never sees `kind`/`carSpec`/`pose`
 * — only `shape`. Obstacle shapes are pure geometry, so buildLevel needs NO kernel; goal-fit
 * needs the player footprint, so inGoal takes the kernel.
 *
 * Depends on the `Physics` namespace (physics-kernel.js). Browser global + Node module.
 */
const Scene = (function (P) {

  const BORDER = 0.45; // field is fenced by a border this thick

  // buildLevel(def) → Level { w, h, start, goal, vehicle?, obstacles: SceneObstacle[] }
  // SceneObstacle = { kind, pose?, rect?, carSpec?, shape }
  function buildLevel(def) {
    const obstacles = [];
    const B = BORDER;
    // Field borders.
    obstacles.push({ kind: 'border', shape: P.Shape.rectangle(-B, -B, def.w + 2 * B, B) });
    obstacles.push({ kind: 'border', shape: P.Shape.rectangle(-B, def.h, def.w + 2 * B, B) });
    obstacles.push({ kind: 'border', shape: P.Shape.rectangle(-B, 0, B, def.h) });
    obstacles.push({ kind: 'border', shape: P.Shape.rectangle(def.w, 0, B, def.h) });
    // Walls / curbs (axis-aligned or angled).
    for (const r of (def.walls || [])) {
      const shape = r.ang != null
        ? P.Shape.orientedBox(r.cx, r.cy, r.w, r.h, r.ang)
        : P.Shape.rectangle(r.x, r.y, r.w, r.h);
      obstacles.push({ kind: r.kind || 'wall', rect: r, shape });
    }
    // Parked cars (each uses ITS OWN vehicle dims — pure geometry, no kernel).
    for (const c of (def.cars || [])) {
      const carSpec = P.vehicleSpecFor(c.type);
      const shape = P.Shape.orientedBox(c.cx, c.cy,
        P.specLength(carSpec), P.specWidth(carSpec), c.h);
      obstacles.push({ kind: 'car', pose: c, carSpec, shape });
    }
    return Object.assign({}, def, { obstacles });
  }

  // goalPolygon(goal) → Polygon (axis-aligned or oriented box). ⇐ render + clearance HUD.
  function goalPolygon(g) {
    if (g.ang) return P.orientedBoxPolygon(g.cx, g.cy, g.w, g.h, g.ang);
    return P.rectanglePolygon(g.cx - g.w / 2, g.cy - g.h / 2, g.w, g.h);
  }

  // inGoal(pose, goal, kernel) → boolean. Uses kernel.carPolygon so the footprint matches
  // the level's vehicle; heading must fall within tolerance of an allowed goal heading.
  function inGoal(pose, goal, kernel) {
    const okHead = goal.heads.some(
      hd => Math.abs(P.normalizeAngle(P.poseHeading(pose) - P.rad(hd))) <= P.rad(goal.tol));
    if (!okHead) return false;
    const zone = goalPolygon(goal);
    return kernel.carPolygon(pose).every(v => P.pointInPolygon(v, zone));
  }

  return { buildLevel, goalPolygon, inGoal };
})(typeof Physics !== 'undefined' ? Physics : require('./physics-kernel.js'));

if (typeof module !== 'undefined' && module.exports) module.exports = Scene;
