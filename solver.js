'use strict';
// Weighted-A* parking solver over the kinematic bicycle model (hybrid-A* style:
// continuous motion, but a discretized pose grid for de-duplication so deep,
// many-move maneuvers stay tractable).
//
// Requires physics.js globals: CAR, VEHICLES, setVehicle, buildLevel, advance,
// simulateMove, inGoal, normAng, rad, deg.
//
// solveParkingLevel(def, opts, cb) -> Promise<{steer (deg), dist}[] | null>
//   opts: { weight, maxExpand, posCell, angCellDeg, yield }
//   Candidate steer/dist are clean values and are simulated with the exact
//   numbers that get stored, so a returned solution replays to the same pose.

function _heapPush(h, s) {
  h.push(s);
  let i = h.length - 1;
  while (i > 0) { const p = (i - 1) >> 1; if (h[p].f <= h[i].f) break; const t = h[p]; h[p] = h[i]; h[i] = t; i = p; }
}
function _heapPop(h) {
  const top = h[0], last = h.pop();
  if (h.length) {
    h[0] = last; let i = 0; const n = h.length;
    for (;;) {
      let l = 2 * i + 1, r = 2 * i + 2, m = i;
      if (l < n && h[l].f < h[m].f) m = l;
      if (r < n && h[r].f < h[m].f) m = r;
      if (m === i) break;
      const t = h[m]; h[m] = h[i]; h[i] = t; i = m;
    }
  }
  return top;
}

async function solveParkingLevel(def, opts = {}, progressCb = null) {
  const {
    weight = 2.0,        // heuristic weight: higher = greedier/faster, longer plans
    maxExpand = 150000,  // safety cap on state expansions
    timeMs = Infinity,   // wall-clock budget per attempt
    posCell = 0.2,       // m, visited-grid resolution
    angCellDeg = 9,      // deg, visited-grid heading resolution
    yield: doYield = true,
  } = opts;

  const savedCar = Object.assign({}, CAR);
  setVehicle(def.vehicle || 'default');
  const lvl = buildLevel(def);
  const { obstacles, goal, start } = lvl;
  const ms = CAR.maxSteer;

  // Clean candidate values → stored == simulated == replayed (no drift).
  const fracs = [-1, -0.62, -0.36, -0.16, 0, 0.16, 0.36, 0.62, 1];
  const STEERS_DEG = [...new Set(fracs.map(f => Math.round(f * ms)))];
  const DISTS = [-9, -6, -4, -2.5, -1.4, -0.7, -0.35,
                  0.35, 0.7, 1.4, 2.5, 4, 6, 9, 13];

  const angCell = rad(angCellDeg);
  const key = p =>
    Math.round(p.x / posCell) + ',' +
    Math.round(p.y / posCell) + ',' +
    Math.round(normAng(p.h) / angCell);

  const goalHeads = goal.heads.map(rad);
  function heur(p) {
    const d = Math.hypot(p.x - goal.cx, p.y - goal.cy);
    let he = Infinity;
    for (const g of goalHeads) { const e = Math.abs(normAng(p.h - g)); if (e < he) he = e; }
    // rough estimate of moves remaining: travel/typical-arc + heading/max-turn
    return d / 9 + he / rad(ms);
  }

  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const open = [];
  const visited = new Map();
  _heapPush(open, { pose: start, moves: [], g: 0, f: weight * heur(start) });
  visited.set(key(start), 0);

  let best = inGoal(start, goal) ? [] : null;
  let expand = 0, lastYield = now();
  const startTime = now();

  while (open.length && !best && expand < maxExpand) {
    if ((expand & 1023) === 0 && now() - startTime > timeMs) break;
    const cur = _heapPop(open);
    expand++;
    let done = false;
    for (const sd of STEERS_DEG) {
      const s = rad(sd);
      for (const d of DISTS) {
        const sim = simulateMove(cur.pose, s, d, obstacles);
        if (sim.hit) continue;
        const end = sim.end;
        const moves = cur.moves.concat([{ steer: sd, dist: d }]);
        if (inGoal(end, goal)) {
          best = moves;
          progressCb && progressCb({ type: 'solution', depth: moves.length, moves });
          done = true; break;
        }
        const g2 = cur.g + 1;
        const k = key(end);
        const pv = visited.get(k);
        if (pv !== undefined && pv <= g2) continue;
        visited.set(k, g2);
        _heapPush(open, { pose: end, moves, g: g2, f: g2 + weight * heur(end) });
      }
      if (done) break;
    }
    if (doYield && now() - lastYield > 16) {
      await new Promise(r => setTimeout(r, 0));
      lastYield = now();
      progressCb && progressCb({ type: 'depth', depth: expand, beamSize: open.length });
    }
  }

  Object.assign(CAR, savedCar);
  CAR.fOver = CAR.len - CAR.wb - CAR.rOver;
  return best;
}

if (typeof module !== 'undefined' && module.exports) module.exports = { solveParkingLevel };
