'use strict';
// Web Worker for the parallel brute-force solver. The main thread (solver.js,
// bruteForceParallel) spawns one of these per CPU slice, hands it a contiguous
// range of turn-1 steering angles, and streams back validated candidate plans.
//
// physics.js + solver.js are pulled in via importScripts so the worker shares the
// exact same advance()/collision/inGoal geometry and the bruteForceKernel itself —
// no logic is duplicated here. solver.js's module.exports line is a no-op in a
// worker (module is undefined), so importing it just defines the functions.
importScripts('physics.js', 'solver.js');

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
  setVehicle(m.vehicle || 'default');
  const lvl = buildLevel(m.def);
  const geom = { obstacles: lvl.obstacles, goal: lvl.goal, start: lvl.start };

  // Validate each candidate locally (workers have simulateMove too), then post only
  // the survivors — keeps the message volume tiny and offloads validation from main.
  const emit = (moves) => {
    const v = validateMoves(geom.start, moves, geom.obstacles, geom.goal);
    if (!v) return;
    const d = v.reduce((a, m) => a + Math.abs(m.dist), 0);
    if (v.length < best.v) { best.v = v.length; best.dist = d; }
    else if (v.length === best.v && d < best.dist) { best.dist = d; }
    postMessage({ type: 'sol', moves: v });
  };

  try {
    await bruteForceKernel(geom, m.prm, emit, () => stopped, null, best,
      (iter) => postMessage({ type: 'progress', iter }));
  } catch (err) {
    // swallow — a dead worker just contributes nothing; main still finishes
  }
  postMessage({ type: 'done' });
};
