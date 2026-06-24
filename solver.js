'use strict';
// Min-turn parking solver (anytime).
//
// Objective (lexicographic):
//   1. fewest TURNS  — a turn = one constant-(steer, direction) arc, i.e. exactly
//      one {steer, dist} move. Changing steering OR forward/back starts a new turn.
//   2. shortest total driving distance among plans with that turn count.
//
// Search = weighted-A* over a discretised pose lattice (hybrid-A* style) where each
// EDGE is a whole turn: from a node we roll one constant-steer arc out cell-by-cell
// (and stop-by-stop), dropping a successor each time the swept pose enters a new
// lattice cell. So the planner freely chooses where to end a turn — a long straight
// or sweeping arc still costs exactly 1 turn — and tight "centimetre-gap" nudges are
// reachable because stops land every `step` metres, not on a coarse distance grid.
//
// The closed set is re-openable (a cell may be re-reached with fewer turns, or the
// same turns and a shorter distance), which keeps the search complete under greedy
// (weighted) guidance.
//
// Anytime: never stops at the first goal — it keeps the best (shortest-distance)
// plan at each turn count from K* (fewest turns seen) up to K*+extraTurns, streaming
// every improvement through progressCb so the editor can offer the author several
// options and a Stop button.
//
// Every move is on the player's input grid (steer 0.2 deg, dist 0.05 m) and is
// propagated with those exact numbers; each emitted plan is re-simulated at the
// game's fine collision step, so what the solver returns replays identically.
//
// Requires physics.js globals: CAR, SAMPLE_STEP, setVehicle, buildLevel, advance,
// carPoly, polysCollide, polyBC, simulateMove, inGoal, normAng, rad.
//
// solveParkingLevel(def, opts, cb) -> Promise<{steer (deg), dist (m)}[] | null>

const STEER_Q = 0.2;   // deg, player steering input grid
const DIST_Q  = 0.05;  // m,   player distance input grid

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

// Ground-truth replay at the game's fine collision step — shared by the inline
// solver and the brute-force workers (which can't see solveParkingLevel's closure).
function validateMoves(start, moves, obstacles, goal) {
  let p = start;
  for (const m of moves) {
    const sim = simulateMove(p, rad(m.steer), m.dist, obstacles, SAMPLE_STEP);
    if (sim.hit) return null;
    p = sim.end;
  }
  return inGoal(p, goal) ? moves : null;
}

// ── Brute-force kernel (shared: main-thread fallback OR Web Worker) ───────────
// Exhaustive 3-turn search over the assigned slice of the FULL steer grid. It
// enumerates the first two turns (every candidate distance on the 0.05 m grid, each
// arc stopped on collision), then for each distinct turn-2 end pose docks the third
// turn EXACTLY: for every steer it scans the whole grid arc-length interval whose
// resulting heading falls inside the goal heading tolerance (see dock()), testing
// in-goal at each. No analytic window and no pose dedup, so no valid solution is
// dropped. Tractability comes only from geometric broad-phases that are provably
// non-lossy (each keeps a strict superset of the poses any valid solution passes
// through), e.g. the turn-2 broad-phase: a fixed steer traces a circle of radius
// R = wb/tan(steer); if the whole circle stays farther than dockRange from the goal
// centre (|dist(centre,goal) − R| > dockRange) no distance on that arc can ever dock,
// so the steer is skipped without rolling it out.
// Calls emit(moves[]) for every goal-reaching candidate; the caller validates+dedupes.
// `best.v` is the fewest VALID turns seen so far (updated by the caller after
// validation); once it is ≤ 2 the dock (a 3rd turn) is pruned as it can't be shorter.
async function bruteForceKernel(geom, prm, emit, shouldStop, yieldHook, best, progressHook = null) {
  const { obstacles, goal, start } = geom;
  const STEERS = prm.STEERS;
  const DIST_Q = prm.DIST_Q, A1 = prm.arc1, A2 = prm.arc2, A3 = prm.arc3;
  const wb = CAR.wb;
  const round2 = v => Math.round(v * 100) / 100;
  const rCar = 0.5 * Math.hypot(CAR.len, CAR.wid) + 0.02;
  const offc = (CAR.wb + CAR.fOver - CAR.rOver) / 2;
  const goalHalfDiag = 0.5 * Math.hypot(goal.w, goal.h);
  const tolR = rad(goal.tol), gHeads = goal.heads.map(rad);
  const dockRange = A3 + goalHalfDiag + 0.5;     // turn-2 must end within one dock arc of goal
  const gate1 = A2 + dockRange + 0.5;            // turn-1 must leave a reachable turn-2
  const N1 = Math.floor(A1 / DIST_Q), N2 = Math.floor(A2 / DIST_Q);
  best = best || { v: Infinity };

  // Per-steer turn-3 radii, precomputed once (R = wb/tan(steer); ±Infinity ≈ straight).
  const R3 = new Float64Array(STEERS.length), absR3 = new Float64Array(STEERS.length);
  for (let i = 0; i < STEERS.length; i++) {
    const s = rad(STEERS[i]);
    const r = Math.abs(s) < 1e-4 ? Infinity : wb / Math.tan(s);
    R3[i] = r; absR3[i] = Math.abs(r);
  }

  function hits(p) {
    const ccx = p.x + Math.cos(p.h) * offc, ccy = p.y + Math.sin(p.h) * offc;
    let poly = null;
    for (let i = 0; i < obstacles.length; i++) {
      const o = obstacles[i]; const bc = o.bc || (o.bc = polyBC(o.poly));
      const dx = ccx - bc.x, dy = ccy - bc.y, rr = rCar + bc.r;
      if (dx * dx + dy * dy > rr * rr) continue;
      if (!poly) poly = carPoly(p);
      if (polysCollide(poly, o.poly)) return true;
    }
    return false;
  }
  const hDist = p => Math.hypot(p.x - goal.cx, p.y - goal.cy);
  // Closest approach of a fixed-steer arc (a circle, or a line when straight) to
  // a target point, plus the signed arc length that reaches that closest point.
  function circleApproach(p, steer, Tx, Ty) {
    if (Math.abs(steer) < 1e-4) {
      const ux = Math.cos(p.h), uy = Math.sin(p.h);
      const t = (Tx - p.x) * ux + (Ty - p.y) * uy;
      const fx = p.x + ux * t, fy = p.y + uy * t;
      return { cd: Math.hypot(fx - Tx, fy - Ty), sStar: t };
    }
    const R = wb / Math.tan(steer);
    const cxC = p.x - Math.sin(p.h) * R, cyC = p.y + Math.cos(p.h) * R;
    const dC = Math.hypot(Tx - cxC, Ty - cyC);
    const phi = Math.atan2(Ty - cyC, Tx - cxC), hStar = phi + Math.PI / 2 * Math.sign(R);
    return { cd: Math.abs(dC - Math.abs(R)), sStar: R * normAng(hStar - p.h) };
  }
  // Exact final turn. The heading after a turn-3 arc is linear in its length,
  // For each turn-3 steer the in-goal arc lengths are bounded EXACTLY by two
  // necessary conditions, each a closed-form arc-length interval:
  //   • heading — p3.h = p2.h + d3/R is linear in d3, so the lengths whose heading is
  //     within the goal tolerance form [c − |R|·tolR, c + |R|·tolR] (c reaches the
  //     exact goal heading). In-goal requires heading within tolerance.
  //   • position — in-goal ⇒ the whole car footprint is inside the goal box, so the
  //     rear axle (a point of that footprint) is within goalHalfDiag of the goal
  //     centre. On the turn-3 circle that is the arc [sStar − |R|·α, sStar + |R|·α]
  //     around the closest-approach length sStar, with α from the law of cosines.
  // The valid d3 lie in the INTERSECTION of those intervals (and [−A3, A3]); scanning
  // every 0.05 m grid length in it and testing inGoal misses nothing. Both bounds are
  // necessary conditions, so the intersection is a superset of the true solution set —
  // exact, with no window guess and no p2 dedup (every distinct turn-2 end pose is
  // docked). The geometric broad-phases gating which (p1, p2) reach here are likewise
  // non-lossy (each keeps a strict superset of poses any valid solution passes through).
  const NA3 = Math.floor(A3 / DIST_Q);
  const rho = goalHalfDiag;          // rear axle ∈ car footprint ⊂ goal box ⇒ within rho of centre
  const scanInterval = (p2, s3, lo, hi, sd3, m1, m2) => {
    let nLo = Math.ceil(lo / DIST_Q), nHi = Math.floor(hi / DIST_Q);
    if (nLo < -NA3) nLo = -NA3;
    if (nHi >  NA3) nHi =  NA3;
    for (let n = nLo; n <= nHi; n++) {
      if (n === 0) continue;
      const d = round2(n * DIST_Q);
      const p3 = advance(p2, s3, d); if (hits(p3)) continue;
      if (inGoal(p3, goal)) emit([m1, m2, { steer: sd3, dist: d }]);
    }
  };
  const rho2 = rho * rho;
  function dock(p2, m1, m2) {
    if (shouldStop && shouldStop()) return;   // abort quickly when deadline fires
    const ph = p2.h, sinH = Math.sin(ph), cosH = Math.cos(ph);
    const gx = goal.cx, gy = goal.cy;
    for (let si = 0; si < STEERS.length; si++) {
      const R = R3[si], aR = absR3[si];
      if (R === Infinity) {                      // straight: heading constant, path is a line
        let okHead = false;
        for (const gh of gHeads) if (Math.abs(normAng(ph - gh)) <= tolR) { okHead = true; break; }
        if (!okHead) continue;
        const t = (gx - p2.x) * cosH + (gy - p2.y) * sinH;        // length to closest approach
        const fx = p2.x + cosH * t, fy = p2.y + sinH * t;
        const cd2 = (fx - gx) * (fx - gx) + (fy - gy) * (fy - gy);
        if (cd2 > rho2) continue;                                  // line never enters the goal box
        const half = Math.sqrt(rho2 - cd2);                        // chord half-length within rho
        scanInterval(p2, 0, t - half, t + half, 0, m1, m2);
        continue;
      }
      const Cx = p2.x - sinH * R, Cy = p2.y + cosH * R;            // turn-3 circle centre
      const dgx = gx - Cx, dgy = gy - Cy, dC2 = dgx * dgx + dgy * dgy;
      // Reject (no sqrt) any circle that never brings the rear axle within rho of goal:
      //   |dC − |R|| > rho  ⟺  dC > |R|+rho  OR  dC < |R|−rho
      const outer = aR + rho;
      if (dC2 > outer * outer) continue;
      const inner = aR - rho;
      if (inner > 0 && dC2 < inner * inner) continue;
      const dC = Math.sqrt(dC2);
      let cosA = (R * R + dC2 - rho2) / (2 * aR * dC);
      if (cosA >  1) cosA =  1;
      if (cosA < -1) cosA = -1;
      const posHalf = aR * Math.acos(cosA);                        // position-interval half-width
      const phi = Math.atan2(dgy, dgx), hStar = phi + (R > 0 ? Math.PI / 2 : -Math.PI / 2);
      const sStar = R * normAng(hStar - ph);                       // length to closest approach
      const halfW = aR * tolR;                                     // heading-interval half-width
      const s3 = rad(STEERS[si]), sd3 = STEERS[si];
      for (const gh of gHeads) {
        const c = R * normAng(gh - ph);                            // length onto the exact goal heading
        const lo = Math.max(c - halfW, sStar - posHalf);
        const hi = Math.min(c + halfW, sStar + posHalf);
        if (lo <= hi) scanInterval(p2, s3, lo, hi, sd3, m1, m2);
      }
    }
  }

  // Optional turn-2 end-pose merge. dedupPos = 0 (default) → fully exact: every distinct
  // p2 is docked. A positive value collapses p2 poses within that lattice cell to a
  // single dock, trading completeness for speed (a fully exact search at the 0.2°/0.05 m
  // input grid is intractable). Set deliberately by the caller.
  const dedupP = prm.dedupPos || 0, dedupA = rad(prm.dedupAng || 0);
  const docked = dedupP > 0 ? new Set() : null;
  let iter = 0;
  for (let i1 = prm.sliceLo; i1 < prm.sliceHi; i1++) {
    const sd1 = STEERS[i1], s1 = rad(sd1);
    for (let dr1 = 1; dr1 >= -1; dr1 -= 2) {
      for (let n1 = 1; n1 <= N1; n1++) {
        const dist1 = round2(dr1 * n1 * DIST_Q);
        const p1 = advance(start, s1, dist1);
        if (hits(p1)) break;                                    // arc blocked beyond here
        if (inGoal(p1, goal)) { emit([{ steer: sd1, dist: dist1 }]); continue; }   // 1-turn
        if (hDist(p1) > gate1) continue;
        const m1 = { steer: sd1, dist: dist1 };
        for (let i2 = 0; i2 < STEERS.length; i2++) {
          const sd2 = STEERS[i2], s2 = rad(sd2);
          if (circleApproach(p1, s2, goal.cx, goal.cy).cd > dockRange) continue;   // broad-phase
          for (let dr2 = 1; dr2 >= -1; dr2 -= 2) {
            if (sd2 === sd1 && dr2 === dr1) continue;            // same arc = still 1 turn
            for (let n2 = 1; n2 <= N2; n2++) {
              const dist2 = round2(dr2 * n2 * DIST_Q);
              const p2 = advance(p1, s2, dist2);
              if (hits(p2)) break;
              const m2 = { steer: sd2, dist: dist2 };
              if (inGoal(p2, goal)) { emit([m1, m2]); continue; }   // 2-turn
              if (best.v <= 2) continue;                          // can't beat a known ≤2-turn
              if (hDist(p2) > dockRange) continue;
              if (docked) {
                const ck = Math.round(p2.x / dedupP) + ',' + Math.round(p2.y / dedupP) +
                           ',' + Math.round(normAng(p2.h) / dedupA);
                if (docked.has(ck)) continue;
                docked.add(ck);
              }
              dock(p2, m1, m2);
        }
        if ((++iter & 31) === 0) {
          if (shouldStop && shouldStop()) return;
          if (yieldHook) await yieldHook();
          if (progressHook && (iter & 0x3ff) === 0) progressHook(iter);
        }
      }
    }
  }
  }
}
}

// Run bruteForceKernel across a pool of Web Workers (one steer-slice each),
// streaming validated candidates back through `consume`. Resolves when every
// worker finishes, the deadline passes, or shouldStop() trips. Falls back to a
// single inline kernel run when Workers are unavailable (Node, older browsers).
async function bruteForceParallel(def, prm, consume, shouldStop, deadline, nowFn, progressCb = null) {
  const best = { v: Infinity };
  const onCand = moves => {                       // shared validate→consume→track-best
    const v = validateMoves(prm._start, moves, prm._obstacles, prm._goal);
    if (v) { if (v.length < best.v) best.v = v.length; consume(v); }
    return v;
  };

  const haveWorkers = typeof Worker !== 'undefined' && prm.workers !== 0;
  if (haveWorkers) {
    let workers = [];
    try {
      const want = Math.min(prm.workers || 8, prm.STEERS.length);
      // Interleave steer values across workers (round-robin) so each worker gets a mix
      // of fast (high-lock) and slow (near-zero) steers — avoids stragglers.
      const workerSteers = Array.from({ length: want }, () => []);
      prm.STEERS.forEach((s, i) => workerSteers[i % want].push(s));
      const wprm = Object.assign({}, prm);
      delete wprm._start; delete wprm._obstacles; delete wprm._goal;   // rebuilt in worker
      const N1est = Math.floor((prm.arc1 || 8) / prm.DIST_Q);
      const totalIters = prm.STEERS.length * 2 * N1est * prm.STEERS.length;
      await new Promise((resolve) => {
        let done = 0, finished = false;
        const wIters = new Array(want).fill(0);
        const finish = () => { if (finished) return; finished = true;
          for (const w of workers) { try { w.postMessage({ type: 'stop' }); w.terminate(); } catch (e) {} }
          resolve();
        };
        const emitProgress = () => {
          const iters = wIters.reduce((a, b) => a + b, 0);
          progressCb && progressCb({ type: 'bf_progress', done, total: want, iters, totalIters });
        };
        const timer = setInterval(() => {
          if ((shouldStop && shouldStop()) || (deadline && nowFn() > deadline)) { clearInterval(timer); finish(); }
          else emitProgress();
        }, 500);
        for (let w = 0; w < want; w++) {
          const wi = w;
          const wk = new Worker('solver-worker.js');
          workers.push(wk);
          wk.onmessage = (e) => {
            const m = e.data;
            if (m.type === 'sol') {
              if (m.moves.length < best.v) best.v = m.moves.length;
              consume(m.moves);
              if (m.moves.length <= 2)                          // share the bound for pruning
                for (const o of workers) { try { o.postMessage({ type: 'best', v: best.v }); } catch (e2) {} }
            } else if (m.type === 'progress') {
              wIters[wi] = m.iter;
            } else if (m.type === 'done') {
              if (++done >= want) { clearInterval(timer); emitProgress(); finish(); }
            }
          };
          wk.onerror = () => { if (++done >= want) { clearInterval(timer); finish(); } };
          wk.postMessage({ type: 'run', def, vehicle: def.vehicle || 'default',
            prm: Object.assign({}, wprm, { STEERS: workerSteers[w], sliceLo: 0, sliceHi: workerSteers[w].length }) });
        }
      });
      return;
    } catch (e) {
      for (const w of workers) { try { w.terminate(); } catch (e2) {} }   // fall through to inline
    }
  }

  // Inline fallback (Node / no Worker support): one kernel over the whole grid.
  const geom = { obstacles: prm._obstacles, goal: prm._goal, start: prm._start };
  const yieldHook = async () => { await new Promise(r => setTimeout(r, 0)); };
  await bruteForceKernel(geom, Object.assign({}, prm, { sliceLo: 0, sliceHi: prm.STEERS.length }),
    onCand, () => (shouldStop && shouldStop()) || (deadline && nowFn() > deadline), yieldHook, best);
}

async function solveParkingLevel(def, opts = {}, progressCb = null) {
  const {
    weight = 1.0,          // heuristic weight: higher = greedier/faster first plan
    extraTurns = 2,        // also keep best plans at K*+1 .. K*+extraTurns
    maxExpand = 500000,    // safety cap on state expansions
    timeMs = Infinity,     // wall-clock budget
    idleMs = 2500,         // stop this long after the last improvement (once solved)
    posCell = 0.18,        // m, lattice position resolution
    angCellDeg = 8,        // deg, lattice heading resolution
    step = 0.1,            // m, arc rollout / collision sample (snapped to DIST_Q)
    steerSet = null,       // override steering candidates (deg)
    maxArc = null,         // override max single-turn arc length (m)
    shouldStop = null,     // ()=>bool cooperative cancel (Stop button)
    yield: doYield = true,
    diverseN = 3,          // max diverse solutions to keep per turn count
    diverseThresh = 1.5,   // m, min trajectory distance to count as a distinct solution
    diversityBias = 1.5,   // turn-equivalent bonus for paths diverged from all known patterns
  } = opts;

  const snap = (v, q) => Math.round(v / q) * q;
  const round2 = v => Math.round(v * 100) / 100;
  const savedCar = Object.assign({}, CAR);
  const restore = () => { Object.assign(CAR, savedCar); CAR.fOver = CAR.len - CAR.wb - CAR.rOver; };
  setVehicle(def.vehicle || 'default');
  const lvl = buildLevel(def);
  const { obstacles, goal, start } = lvl;
  const ms = CAR.maxSteer;

  // Steering candidates on the 0.2-deg grid (denser near full lock for tight work).
  const fracs = [-1, -0.9, -0.71, -0.7, -0.45, -0.29, -0.245, -0.17, -0.12, 0, 0.12, 0.17, 0.245, 0.29, 0.45, 0.7, 0.71, 0.9, 1];
  const STEERS = (steerSet || [...new Set(fracs.map(f => snap(f * ms, STEER_Q)))])
    .slice().sort((a, b) => a - b);
  const STEP = Math.max(DIST_Q, snap(step, DIST_Q));
  const ARC_MAX = maxArc || (Math.hypot(def.w, def.h) + 2);
  const NSTEP = Math.floor((ARC_MAX + 1e-9) / STEP);
  const angCell = rad(angCellDeg);
  const goalHeads = goal.heads.map(rad);

  const key = p =>
    Math.round(p.x / posCell) + ',' +
    Math.round(p.y / posCell) + ',' +
    Math.round(normAng(p.h) / angCell);

  // Single-pose collision test (no inflation — matches game geometry; the fine-step
  // validate() gate covers any sub-`step` gap). Broad-phase by bounding circle.
  const rCar = 0.5 * Math.hypot(CAR.len, CAR.wid) + 0.02;
  const offc = (CAR.wb + CAR.fOver - CAR.rOver) / 2;
  function hits(p) {
    const ccx = p.x + Math.cos(p.h) * offc, ccy = p.y + Math.sin(p.h) * offc;
    let poly = null;
    for (let i = 0; i < obstacles.length; i++) {
      const o = obstacles[i];
      const bc = o.bc || (o.bc = polyBC(o.poly));
      const dx = ccx - bc.x, dy = ccy - bc.y, rr = rCar + bc.r;
      if (dx * dx + dy * dy > rr * rr) continue;
      if (!poly) poly = carPoly(p);
      if (polysCollide(poly, o.poly)) return true;
    }
    return false;
  }

  const TYP = 6, maxSteerRad = rad(ms);
  const hDist = p => Math.hypot(p.x - goal.cx, p.y - goal.cy);
  function hTurns(p) {                       // rough estimate of turns remaining
    let he = Infinity;
    for (const g of goalHeads) { const e = Math.abs(normAng(p.h - g)); if (e < he) he = e; }
    return hDist(p) / TYP + he / maxSteerRad;
  }

  function buildMoves(node, lastMove) {
    const out = [lastMove];
    for (let n = node; n && n.move; n = n.parent) out.push(n.move);
    return out.reverse();
  }
  // Ground-truth gate: replay the plan at the game's fine collision step.
  function validate(moves) {
    let p = start;
    for (const m of moves) {
      const sim = simulateMove(p, rad(m.steer), m.dist, obstacles, SAMPLE_STEP);
      if (sim.hit) return null;
      p = sim.end;
    }
    return inGoal(p, goal) ? moves : null;
  }

  // Fine final-approach sweep — lands precisely in a tight goal the coarse steer
  // grid can't hit (e.g. a slot barely wider than the car). One extra turn.
  const dockReach = Math.max(goal.w, goal.h) / 2 + (opts.dockReach || 2.4);
  const DOCKN = Math.round(4 / DIST_Q);
  const dockCells = new Set();
  let dockBudget = opts.dock === false ? 0 : (opts.dockBudget || 4000);
  function tryDock(cur) {
    for (const sd of STEERS) {
      const s = rad(sd);
      for (let d = 0; d < 2; d++) {
        const dir = d === 0 ? 1 : -1;
        if (cur.inSd === sd && cur.inDir === dir) continue;
        for (let i = 1; i <= DOCKN; i++) {
          const dist = round2(dir * i * DIST_Q);
          const p = advance(cur.pose, s, dist);
          if (hits(p)) break;
          if (inGoal(p, goal)) {
            const mv = validate(buildMoves(cur, { steer: sd, dist }));
            if (mv) return mv;
          }
        }
      }
    }
    return null;
  }

  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const startTime = now();
  let expand = 0, lastYield = startTime, lastImprove = startTime, solvedAt = 0;
  const budgetLeft = () =>
    !(shouldStop && shouldStop()) && expand < maxExpand && now() - startTime <= timeMs &&
    (!solvedAt || now() - lastImprove <= idleMs);

  // Trajectory signature: steer-direction pattern + rear-axle (x,y) at each turn end.
  // The pattern (e.g. "Lf,Rr") is the primary discriminator: plans with different
  // steer-sign sequences represent genuinely different driving strategies and are
  // always kept as distinct options regardless of positional proximity.
  function planSig(moves) {
    let p = start;
    const poses = moves.map(m => { p = advance(p, rad(m.steer), m.dist); return { x: p.x, y: p.y }; });
    const pat = moves.map(m =>
      (m.steer > 1 ? 'L' : m.steer < -1 ? 'R' : 'S') + (m.dist >= 0 ? 'f' : 'r')
    ).join(',');
    return { poses, pat };
  }
  function sigDist(sa, sb) {
    if (sa.pat !== sb.pat) return Infinity;   // different strategy → always diverse
    let mx = 0;
    for (let i = 0; i < sa.poses.length; i++) {
      const d = Math.hypot(sa.poses[i].x - sb.poses[i].x, sa.poses[i].y - sb.poses[i].y);
      if (d > mx) mx = d;
    }
    return mx;
  }

  // Diversity bias: paths whose steer-direction pattern has already diverged from
  // every known solution get a heuristic bonus (lower f → expanded sooner), steering
  // the search toward genuinely different strategies once some solutions are known.
  const knownPats = [];
  function updateKnownPats() {
    knownPats.length = 0;
    for (const set of solSets.values()) for (const e of set) knownPats.push(e.sig.pat);
  }
  function diverseBonus(partialPat) {
    if (diversityBias <= 0 || !partialPat || knownPats.length === 0) return 0;
    for (const kp of knownPats) {
      // Still potentially converging: partial is a prefix of a known, or known is a prefix of partial
      if (kp.startsWith(partialPat) || partialPat.startsWith(kp)) return 0;
    }
    return diversityBias;   // definitively diverged from all known patterns
  }

  // Anytime incumbents: up to diverseN distinct-trajectory plans per turn count.
  // Each entry: { dist, moves, sig, id }. Streamed events carry id + replaces.
  let bestTurns = Infinity;
  const solSets = new Map();   // turns → Entry[]
  let _nextId = 0;
  function offer(moves) {
    if (!moves) return;
    const tn = moves.length;
    const nd = moves.reduce((a, m) => a + Math.abs(m.dist), 0);
    const sig = planSig(moves);

    if (!solSets.has(tn)) solSets.set(tn, []);
    const set = solSets.get(tn);

    // Find most-similar existing entry.
    let minD = Infinity, minI = -1;
    for (let i = 0; i < set.length; i++) {
      const d = sigDist(sig, set[i].sig);
      if (d < minD) { minD = d; minI = i; }
    }

    const emit = (entry, replaces) => {
      if (tn < bestTurns) bestTurns = tn;
      lastImprove = now(); if (!solvedAt) solvedAt = now();
      updateKnownPats();   // refresh diversity bias after each accepted solution
      progressCb && progressCb({ type: 'solution', moves: entry.moves, turns: tn,
        dist: round2(entry.dist), id: entry.id, replaces, poses: entry.sig.poses });
    };

    if (minI >= 0 && minD < diverseThresh) {
      // Similar to existing: replace only if shorter distance.
      if (nd < set[minI].dist - 1e-9) {
        const old = set[minI];
        set[minI] = { dist: nd, moves, sig, id: _nextId++ };
        emit(set[minI], old.id);
      }
      return;
    }

    // Genuinely diverse path.
    const entry = { dist: nd, moves, sig, id: _nextId++ };
    if (set.length < diverseN) {
      set.push(entry);
      emit(entry, null);
      return;
    }

    // Set full: find the entry whose nearest neighbor is closest (least unique)
    // and swap it out only if the new plan would increase the min pairwise distance.
    let worstNear = Infinity, worstI = -1;
    for (let i = 0; i < set.length; i++) {
      let nearD = Infinity;
      for (let j = 0; j < set.length; j++) {
        if (i !== j) { const d = sigDist(set[i].sig, set[j].sig); if (d < nearD) nearD = d; }
      }
      if (nearD < worstNear) { worstNear = nearD; worstI = i; }
    }
    const newNear = Math.min(...set.map(e => sigDist(sig, e.sig)));
    if (newNear > worstNear) {
      const old = set[worstI];
      set[worstI] = entry;
      emit(entry, old.id);
    }
  }

  if (inGoal(start, goal)) offer([]);

  const open = [];
  const closed = new Map();   // cell -> { turns, dist }  (re-openable)

  // ── Brute-force warmup: turns 1 & 2 full grid + analytic 3rd-turn dock ───────
  // Multicore brute force across Web Workers (8 by default; one steer-slice each),
  // streaming validated candidates into offer(). Steering stays on the player input
  // grid — default the full 0.2° grid, or pass a coarser opts.bfSteerStep for a fast
  // pass. Time-boxed (opts.bfTimeMs, default 25 s) so the lattice A* below still gets
  // a share of the budget on deep (>3-turn) levels. See bruteForceKernel for the
  // turn-2 circle broad-phase and the analytic dock that make 3 turns feasible.
  if (opts.bf !== false) {
    const bfStep = Math.max(STEER_Q, opts.bfSteerStep || STEER_Q);    // default = full input grid
    const bfNq = Math.floor(ms / bfStep + 1e-9);
    const BFS = [snap(ms, STEER_Q), snap(-ms, STEER_Q)];              // always include full lock
    for (let q = -bfNq; q <= bfNq; q++)
      BFS.push(snap(Math.max(-ms, Math.min(ms, q * bfStep)), STEER_Q));
    const bfPrm = {
      STEERS: [...new Set(BFS)].sort((a, b) => a - b),
      DIST_Q,
      arc1: opts.bfArc1 || Math.min(8, ARC_MAX),
      arc2: opts.bfArc2 || 6,
      arc3: opts.bfArc3 || 6,
      workers:  opts.workers,
      _start: start, _obstacles: obstacles, _goal: goal,
    };
    const bfDeadline = opts.bfTimeMs ? startTime + opts.bfTimeMs : Infinity;
    const bfWorkers = Math.min(bfPrm.workers || 8, bfPrm.STEERS.length);
    progressCb && progressCb({ type: 'phase', phase: 'bf', workers: bfWorkers });
    await bruteForceParallel(def, bfPrm, m => offer(m), shouldStop, bfDeadline, now, progressCb);
  }
  progressCb && progressCb({ type: 'phase', phase: 'astar' });

  const s0 = { pose: start, turns: 0, dist: 0, parent: null, move: null,
               inSd: null, inDir: 0, cell: key(start), pat: '', f: weight * hTurns(start) };
  _heapPush(open, s0);
  closed.set(s0.cell, { turns: 0, dist: 0 });

  while (open.length && budgetLeft()) {
    const cur = _heapPop(open); expand++;
    if (cur.turns >= bestTurns + extraTurns) {   // can't extend a wanted candidate
      await yieldMaybe(); continue;
    }
    const stale = closed.get(cur.cell);           // superseded by a better label?
    if (stale && (stale.turns < cur.turns || (stale.turns === cur.turns && stale.dist < cur.dist - 1e-9))) {
      await yieldMaybe(); continue;
    }
    const nt = cur.turns + 1;
    // Fine docking sweep from states near the goal (each region tried once).
    if (dockBudget > 0 && cur.turns < bestTurns + extraTurns && hDist(cur.pose) < dockReach) {
      const dk = Math.round(cur.pose.x / 0.5) + ',' + Math.round(cur.pose.y / 0.5) +
                 ',' + Math.round(normAng(cur.pose.h) / rad(20));
      if (!dockCells.has(dk)) { dockCells.add(dk); dockBudget--; offer(tryDock(cur)); }
    }
    for (let si = 0; si < STEERS.length; si++) {
      const sd = STEERS[si], s = rad(sd);
      for (let d = 0; d < 2; d++) {
        const dir = d === 0 ? 1 : -1;
        if (cur.inSd === sd && cur.inDir === dir) continue;   // would just extend the same turn
        // Steer-direction symbol for this arc; build the path pattern incrementally.
        const sym = (sd > 1 ? 'L' : sd < -1 ? 'R' : 'S') + (dir > 0 ? 'f' : 'r');
        const newPat = cur.pat ? cur.pat + ',' + sym : sym;
        const dBonus = diverseBonus(newPat);
        let lastCell = cur.cell;
        for (let i = 1; i <= NSTEP; i++) {
          const dist = round2(dir * i * STEP);                // quantised signed distance
          const p = advance(cur.pose, s, dist);
          if (hits(p)) break;                                 // arc blocked beyond here
          const nd = cur.dist + i * STEP;
          if (inGoal(p, goal)) offer(validate(buildMoves(cur, { steer: sd, dist })));
          const ck = key(p);
          if (ck === lastCell) continue;
          lastCell = ck;
          const pv = closed.get(ck);
          if (pv && (pv.turns < nt || (pv.turns === nt && pv.dist <= nd + 1e-9))) continue;
          closed.set(ck, { turns: nt, dist: nd });
          _heapPush(open, { pose: p, turns: nt, dist: nd, parent: cur,
            move: { steer: sd, dist }, inSd: sd, inDir: dir, cell: ck, pat: newPat,
            f: nt + weight * hTurns(p) + 1e-4 * (nd + hDist(p)) - dBonus });
        }
      }
    }
    await yieldMaybe();
  }

  async function yieldMaybe() {
    if (doYield && now() - lastYield > 16) {
      await new Promise(r => setTimeout(r, 0));
      lastYield = now();
      progressCb && progressCb({ type: 'depth', depth: expand, beamSize: open.length });
    }
  }

  restore();
  const bestSet = solSets.has(bestTurns) ? solSets.get(bestTurns) : [];
  const bestEntry = bestSet.length ? bestSet.reduce((a, e) => e.dist < a.dist ? e : a) : null;
  progressCb && progressCb({ type: 'done', expand,
    turns: bestEntry ? bestTurns : null,
    dist:  bestEntry ? round2(bestEntry.dist) : null });
  return bestEntry ? bestEntry.moves : null;
}

if (typeof module !== 'undefined' && module.exports) module.exports = { solveParkingLevel };
