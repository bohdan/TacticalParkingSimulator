'use strict';
// Beam-search parking solver.
// Requires physics.js to be loaded first (uses CAR, VEHICLES, setVehicle,
// buildLevel, advance, simulateMove, inGoal, normAng, rad, deg as globals).
//
// solveParkingLevel(def, opts, progressCb) → Promise<{steer°, dist}[] | null>
//   def        — level definition object (same format as LEVELS entries)
//   opts       — { beam=1000, maxDepth=7 }
//   progressCb — called with { type:'depth'|'solution', depth, moves?, beamSize? }

async function solveParkingLevel(def, opts = {}, progressCb = null) {
  const { beam: BEAM = 1000, maxDepth: MAX_DEPTH = 7 } = opts;

  // Save and switch vehicle so CAR matches the level.
  const savedCar = Object.assign({}, CAR);
  setVehicle(def.vehicle || 'default');

  const lvl = buildLevel(def);
  const { obstacles, goal, start } = lvl;

  // Candidate steer angles — finer grid near 0 for precision parking
  const STEERS_DEG = [-35, -28, -20, -12, -6, -2, 0, 2, 6, 12, 20, 28, 35]
    .filter(s => Math.abs(s) <= CAR.maxSteer);
  const STEERS = STEERS_DEG.map(rad);

  // Candidate distances (m) — short steps for tight spots, long for open ones
  const DISTS = [-10, -7, -5, -3.5, -2, -1.2, -0.6,
                   0.6, 1.2, 2, 3.5, 5, 7, 10, 13, 16];

  function heuristic(pose) {
    const dx = pose.x - goal.cx, dy = pose.y - goal.cy;
    const bestHead = goal.heads.reduce((best, hd) => {
      const err = Math.abs(normAng(pose.h - rad(hd)));
      return err < best ? err : best;
    }, Infinity);
    // Euclidean distance² + heavily weighted heading error²
    return dx*dx + dy*dy + (bestHead * 5) ** 2;
  }

  let beamStates = [{ pose: start, moves: [] }];
  let best = null;

  for (let depth = 0; depth < MAX_DEPTH; depth++) {
    const next = [];
    let lastYield = performance.now();

    for (const state of beamStates) {
      for (const s of STEERS) {
        for (const d of DISTS) {
          const sim = simulateMove(state.pose, s, d, obstacles);
          if (sim.hit) continue;
          const moves = [...state.moves,
            { steer: Math.round(deg(s) * 10) / 10, dist: +d.toFixed(2) }];
          if (inGoal(sim.end, goal)) {
            const totalDist = moves.reduce((t, m) => t + Math.abs(m.dist), 0);
            const prevDist  = best ? best.reduce((t, m) => t + Math.abs(m.dist), 0) : Infinity;
            if (!best || moves.length < best.length ||
                (moves.length === best.length && totalDist < prevDist)) {
              best = moves;
              progressCb?.({ type: 'solution', depth: depth+1, moves });
            }
            continue; // don't extend winning states further
          }
          // Prune branches already as long as the best found solution
          if (best && moves.length >= best.length) continue;
          next.push({ pose: sim.end, moves, h: heuristic(sim.end) });
        }
      }
      // Yield to UI every 16 ms so the browser stays responsive
      if (performance.now() - lastYield > 16) {
        await new Promise(r => setTimeout(r, 0));
        lastYield = performance.now();
      }
    }

    progressCb?.({ type: 'depth', depth: depth+1, beamSize: next.length });

    if (best && best.length <= depth+1) break; // can't do better
    next.sort((a, b) => a.h - b.h);
    beamStates = next.slice(0, BEAM);
    if (!beamStates.length) break;
  }

  // Restore vehicle state
  Object.assign(CAR, savedCar);
  CAR.fOver = CAR.len - CAR.wb - CAR.rOver;

  return best; // null if no solution found within MAX_DEPTH
}
