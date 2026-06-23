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

  // ── Brute-force 2-turn warmup ──────────────────────────────────────────────
  // Exhaustively tries all (steer1, dist1, steer2, dist2) on the solver distance
  // grid (step STEP, arc ≤ bf2ArcMax). Complements the lattice A*: guarantees
  // any 2-turn solution that cell-merging in the closed set would prune is found.
  // Key pruning: skip all turn-1 endpoints that cannot reach the goal in one more
  // arc — this cuts 60–80 % of work on typical closed levels. Async with yields
  // every 16 ms so the UI stays responsive. Does NOT count toward `expand` so it
  // doesn't eat into the A* budget.
  const bf2ArcMax = opts.bf2ArcMax !== undefined ? opts.bf2ArcMax : Math.min(6, ARC_MAX);
  if (opts.bf2 !== false) {
    const BN = Math.floor((bf2ArcMax + 1e-9) / DIST_Q);
    const goalReach = bf2ArcMax + Math.max(goal.w, goal.h) + 1;
    outer2:
    for (let si1 = 0; si1 < STEERS.length; si1++) {
      const sd1 = STEERS[si1], s1 = rad(sd1);
      for (let di1 = 0; di1 < 2; di1++) {
        const dir1 = di1 === 0 ? 1 : -1;
        for (let n1 = 1; n1 <= BN; n1++) {
          const dist1 = round2(dir1 * n1 * DIST_Q);
          const p1 = advance(start, s1, dist1);
          if (hits(p1)) break;
          if (hDist(p1) <= goalReach) {
            for (let si2 = 0; si2 < STEERS.length; si2++) {
              const sd2 = STEERS[si2], s2 = rad(sd2);
              for (let di2 = 0; di2 < 2; di2++) {
                if (sd2 === sd1 && di2 === di1) continue;   // same arc = just 1 turn
                const dir2 = di2 === 0 ? 1 : -1;
                for (let n2 = 1; n2 <= BN; n2++) {
                  const dist2 = round2(dir2 * n2 * DIST_Q);
                  const p2 = advance(p1, s2, dist2);
                  if (hits(p2)) break;
                  if (inGoal(p2, goal)) offer(validate([
                    { steer: sd1, dist: dist1 },
                    { steer: sd2, dist: dist2 },
                  ]));
                }
              }
            }
          }
          await yieldMaybe();
          if (shouldStop && shouldStop()) break outer2;
          if (now() - startTime > timeMs) break outer2;
        }
      }
    }
  }

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
