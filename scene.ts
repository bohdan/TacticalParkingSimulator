/*
 * scene.ts — the Scene component (Component 2).
 *
 * Turns a level definition into obstacles that pair render/semantic metadata with an opaque
 * collision Shape, and threads the raw `goal` through unchanged. The physics kernel never
 * sees `kind`/`carSpec`/`pose` — only `shape`. Goal geometry and goal-fit are NOT here: the
 * kernel owns `goalPolygon`/`inGoal`, since the fit depends on the vehicle footprint.
 *
 * Depends on `Physics` (physics-kernel.ts). Browser + Node.
 */
import { Physics } from './physics-kernel.js';
import type { VehicleSpec } from './physics-kernel.js';
import type { Shape } from './geometry2d.js';
import type { PlayableLevelDef, WallDef, CarDef } from './levels.js';

export interface SceneObstacle {
  kind: string;
  shape: Shape;
  rect?: WallDef;
  pose?: CarDef;
  carSpec?: VehicleSpec;
}

export interface BuiltLevel extends PlayableLevelDef {
  obstacles: SceneObstacle[];
}

const BORDER = 0.45;

export function buildLevel(def: PlayableLevelDef): BuiltLevel {
  const obstacles: SceneObstacle[] = [];
  const B = BORDER;

  obstacles.push({ kind: 'border', shape: Physics.Shape.rectangle(-B, -B, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', shape: Physics.Shape.rectangle(-B, def.h, def.w + 2 * B, B) });
  obstacles.push({ kind: 'border', shape: Physics.Shape.rectangle(-B, 0, B, def.h) });
  obstacles.push({ kind: 'border', shape: Physics.Shape.rectangle(def.w, 0, B, def.h) });

  for (const r of (def.walls || [])) {
    const shape = r.ang != null
      ? Physics.Shape.orientedBox(r.cx, r.cy, r.w, r.h, r.ang)
      : Physics.Shape.rectangle(r.x, r.y, r.w, r.h);
    obstacles.push({ kind: r.kind || 'wall', rect: r, shape });
  }

  for (const c of (def.cars || [])) {
    const carSpec = Physics.vehicleSpecFor(c.type);
    const shape = Physics.Shape.orientedBox(c.cx, c.cy, carSpec.len, carSpec.wid, c.h);
    obstacles.push({ kind: 'car', pose: c, carSpec, shape });
  }

  return { ...def, obstacles };
}
