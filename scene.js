'use strict';
/*
 * scene.js — the Scene component (Component 2).
 *
 * Turns a level definition into obstacles that pair render/semantic metadata with an opaque
 * collision Shape, and threads the raw `goal` through unchanged. The physics kernel never
 * sees `kind`/`carSpec`/`pose` — only `shape`. Goal geometry and goal-fit are NOT here: the
 * kernel owns `goalPolygon`/`inGoal`, since the fit depends on the vehicle footprint.
 *
 * Depends on `Physics` (physics-kernel.js). Browser + Node.
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
      const shape = P.Shape.orientedBox(c.cx, c.cy, carSpec.len, carSpec.wid, c.h);
      obstacles.push({ kind: 'car', pose: c, carSpec, shape });
    }
    return Object.assign({}, def, { obstacles });
  }

  return { buildLevel };
})(
  typeof Physics !== 'undefined' ? Physics : require('./physics-kernel.js'),
);

if (typeof module !== 'undefined' && module.exports) module.exports = Scene;
