#!/usr/bin/env node
// Fetches all leaderboard entries from Supabase and re-simulates them against
// the current physics engine (swept-hull collision check).
// Reports any entry that now collides or misses the goal.
//
// Usage: node tools/validate_leaderboard.js [--step 0.005]
// ── Physics engine: the refactored components, via the compat surface ────────
import https from 'node:https';
import { LEVELS } from '../levels.js';
import { simulateMove, buildLevel, inGoal, rad, deg, setVehicle,
         carPoly, advance, convexHull, polysCollide } from '../physics-compat.js';

// ── Config ──────────────────────────────────────────────────────────────────
const LB_URL = 'https://qvjorkpzlwvswsptkwyn.supabase.co';
const LB_KEY = 'sb_publishable_geHaaCkSfPilYWV3fYQHQA_KZdYNrpC';
const _si    = process.argv.indexOf('--step');
const STEP   = parseFloat(_si >= 0 ? process.argv[_si + 1] : '0.005');

// ── Helpers ─────────────────────────────────────────────────────────────────
function fetchJson(url: string): Promise<any> {
  return new Promise<any>((resolve, reject) => {
    const req = https.request(url, {
      headers: { apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}` },
    }, res => {
      let body = '';
      res.on('data', d => body += d);
      res.on('end', () => {
        if (res.statusCode >= 400) reject(new Error(`HTTP ${res.statusCode}: ${body}`));
        else resolve(JSON.parse(body));
      });
    });
    req.on('error', reject);
    req.end();
  });
}

// Parse compact format "steer_deg:dist_m,…"
// Returns array of {steer (rad), dist (m)} — same shape as game.js movesFromCompact.
function parseCompact(str) {
  if (!str) return null;
  try {
    return str.split(',').map(p => {
      const i = p.indexOf(':');
      return { steer: rad(+p.slice(0, i)), dist: +p.slice(i + 1) };
    });
  } catch { return null; }
}

// Parse legacy base64 format (old #sol= style)
function parseLegacy(str) {
  try {
    let s = str.replace(/-/g, '+').replace(/_/g, '/');
    while (s.length % 4) s += '=';
    const arr = JSON.parse(Buffer.from(s, 'base64').toString('utf8'));
    if (!Array.isArray(arr)) return null;
    return arr.map(([steer, dist]) => ({ steer: rad(steer), dist }));
  } catch { return null; }
}

function parseMoves(str) {
  if (!str) return null;
  return str.includes(':') ? parseCompact(str) : parseLegacy(str);
}

// Build a lookup from level_id → level definition
const levelById = new Map();
for (const lv of LEVELS) {
  if (lv.id) levelById.set(lv.id, lv);
  if (lv.name) levelById.set(lv.name, lv); // legacy fallback
}

// Minimum penetration depth (m) between two convex polygons that overlap, via
// the smallest SAT axis overlap. Used to quantify how deep a clip is.
function penetration(A, B) {
  let min = Infinity;
  for (const [P, Q] of [[A, B], [B, A]]) {
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      let nx = b.y - a.y, ny = a.x - b.x;
      const len = Math.hypot(nx, ny); if (!len) continue; nx /= len; ny /= len;
      let minP = Infinity, maxP = -Infinity, minQ = Infinity, maxQ = -Infinity;
      for (const v of P) { const d = v.x*nx+v.y*ny; if (d<minP) minP=d; if (d>maxP) maxP=d; }
      for (const v of Q) { const d = v.x*nx+v.y*ny; if (d<minQ) minQ=d; if (d>maxQ) maxQ=d; }
      const overlap = Math.min(maxP, maxQ) - Math.max(minP, minQ);
      if (overlap < min) min = overlap;
    }
  }
  return min;
}

// Deepest clip (m) + which obstacle, for the move that first collided.
function clipDepth(startPose, m, obstacles) {
  const n = Math.max(2, Math.ceil(Math.abs(m.dist) / STEP));
  let prev = carPoly(startPose), best = { depth: 0, obs: null };
  for (let i = 1; i <= n; i++) {
    const cur = carPoly(advance(startPose, m.steer, m.dist * i / n));
    const swept = convexHull(prev.concat(cur));
    for (const o of obstacles) {
      if (polysCollide(swept, o.poly)) {
        const d = penetration(swept, o.poly);
        if (d > best.depth) best = { depth: d, obs: o.kind };
      }
    }
    prev = cur;
  }
  return best;
}

// Simulate moves; returns {ok, collision, move, depthMM, obs, finalPose}
function validate(lv, moves) {
  setVehicle(lv.vehicle || 'default');
  const level = buildLevel(lv);
  let pose = { ...lv.start };
  for (const m of moves) {
    const r = simulateMove(pose, m.steer, m.dist, level.obstacles, STEP);
    if (r.hit) {
      const c = clipDepth(pose, m, level.obstacles);
      return { ok: false, collision: true, move: m, depthMM: c.depth * 1000, obs: c.obs };
    }
    pose = r.end;
  }
  const goal = inGoal(pose, lv.goal);
  return { ok: goal, collision: false, finalPose: pose };
}

// ── Main ─────────────────────────────────────────────────────────────────────
async function main() {
  console.log(`Fetching leaderboard from ${LB_URL} …`);
  const rows = await fetchJson(
    `${LB_URL}/rest/v1/leaderboard?select=player,level_id,level_name,moves,dist,solution,submitted_at&order=moves.asc,dist.asc&limit=2000`
  );
  console.log(`Fetched ${rows.length} rows. Validating with step=${STEP} m …\n`);

  let total = 0, ok = 0, noSol = 0, unknownLevel = 0, failed = 0;
  const failures = [];

  for (const row of rows) {
    const lv = levelById.get(row.level_id) || levelById.get(row.level_name);
    if (!lv || lv.type === 'cutscene') { unknownLevel++; continue; }
    if (!row.solution) { noSol++; continue; }

    const moves = parseMoves(row.solution);
    if (!moves || moves.length === 0) { noSol++; continue; }

    total++;
    const result = validate(lv, moves);
    if (result.ok) {
      ok++;
    } else {
      failed++;
      failures.push({ row, lv, result });
    }
  }

  // ── Report ──────────────────────────────────────────────────────────────
  console.log(`Results: ${ok} pass, ${failed} fail, ${noSol} no-solution, ${unknownLevel} unknown level (of ${rows.length} total rows)\n`);

  if (failures.length === 0) {
    console.log('All leaderboard solutions pass the swept-hull collision check. ✓');
    return;
  }

  console.log('=== FAILURES ===');
  for (const { row, lv, result } of failures) {
    const ts  = row.submitted_at ? row.submitted_at.slice(0, 10) : '?';
    const why = result.collision
      ? `collision (move steer=${+deg(result.move.steer).toFixed(1)}° dist=${result.move.dist}m) — clip ${result.depthMM.toFixed(2)}mm into ${result.obs}`
      : 'missed goal';
    console.log(`  FAIL  "${lv.name}" (${lv.id || row.level_id})  player=${row.player}  moves=${row.moves}  dist=${row.dist}  date=${ts}`);
    console.log(`        reason: ${why}`);
    console.log(`        solution: ${row.solution}`);
  }
}

main().catch(err => { console.error('Error:', err.message); process.exit(1); });
