'use strict';
// Web Worker for the parallel brute-force solver. The main thread (solver.js,
// bruteForceParallel) spawns one of these per CPU slice, hands it a contiguous
// range of turn-1 steering angles, and streams back validated candidate plans.
//
// The component stack is pulled in via importScripts so the worker shares the exact
// same kinematics / collision / inGoal geometry and the brute-force kernel itself —
// no logic is duplicated here. solver.js binds to the per-vehicle PhysicsKernel built
// below; its module.exports line is a no-op in a worker (module is undefined), so the
// import just defines makeSolver/solveParkingLevel and the Solver global.
importScripts('geometry2d.js', 'physics-kernel.js', 'scene.js', 'solver.js');

let stopped = false;
const best = { v: Infinity, dist: Infinity };

onmessage = async (e) => {
  const m = e.data;
  if (m.type === 'stop') { stopped = true; return; }
  if (m.type === 'best') {
    if (m.v < best.v) { best.v = m.v; best.dist = m.dist ?? Infinity; }
    else if (m.v === best.v && m.dist < best.dist) { best.dist = m.dist; }
    return;
  }
  if (m.type !== 'run') return;

  stopped = false;
  // Build the per-vehicle kernel and bind the solver to it.
  const kernel = Physics.PhysicsKernel(Physics.physicsConfigForLevel({ vehicle: m.vehicle || 'default' }));
  const solver = kernel.createSolver();           // binds the kernel inside solver.js
  const lvl = Scene.buildLevel(m.def);
  const geom = { obstacles: lvl.obstacles.map(o => o.shape), goal: lvl.goal, start: lvl.start };

  // Validate each candidate locally (the kernel has simulateMove + inGoal), then post only
  // the survivors — keeps the message volume tiny and offloads validation from main.
  const emit = (moves) => {
    const v = solver.validateMoves(geom.start, moves, geom.obstacles, geom.goal);
    if (!v) return;
    const d = v.reduce((a, mv) => a + Math.abs(mv.dist), 0);
    if (v.length < best.v) { best.v = v.length; best.dist = d; }
    else if (v.length === best.v && d < best.dist) { best.dist = d; }
    postMessage({ type: 'sol', moves: v });
  };

  try {
    await solver.bruteForce(geom, m.prm, emit, () => stopped, null, best,
      (iter) => postMessage({ type: 'progress', iter }));
  } catch (err) {
    // swallow — a dead worker just contributes nothing; main still finishes
  }
  postMessage({ type: 'done' });
};
