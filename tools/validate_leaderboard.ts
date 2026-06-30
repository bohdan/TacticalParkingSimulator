#!/usr/bin/env node
// Fetches all leaderboard entries from Supabase and re-simulates them against
// the current physics engine (swept-hull collision check).
// Reports any entry that now collides or misses the goal.
//
// Usage: node tools/validate_leaderboard.js [--step 0.005]
// ── Physics engine: the refactored components, via the compat surface ────────
// Collision uses the SAME analytic check the game runs (kernel simulateMove /
// sweepCollides) — no separate sampled re-implementation — so a pass here means
// the plan plays cleanly in the game, including its clean-touch tolerance.
import https from 'node:https';
import { LEVELS } from '../levels.js';
import { simulateMove, buildLevel, inGoal, rad, deg, setVehicle } from '../physics-compat.js';

// ── Config ──────────────────────────────────────────────────────────────────
const LB_URL = 'https://qvjorkpzlwvswsptkwyn.supabase.co';
const LB_KEY = 'sb_publishable_geHaaCkSfPilYWV3fYQHQA_KZdYNrpC';

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

// Simulate moves through the kernel's collision check (the single source of
// truth). Returns {ok, collision, move, moveIdx, point, finalPose}.
function validate(lv, moves) {
  setVehicle(lv.vehicle || 'default');
  const level = buildLevel(lv);
  let pose = { ...lv.start };
  for (let i = 0; i < moves.length; i++) {
    const m = moves[i];
    const r = simulateMove(pose, m.steer, m.dist, level.obstacles);
    if (r.hit) return { ok: false, collision: true, move: m, moveIdx: i, point: r.hit.point };
    pose = r.end;
  }
  return { ok: inGoal(pose, lv.goal), collision: false, finalPose: pose };
}

// ── Main ─────────────────────────────────────────────────────────────────────
async function main() {
  console.log(`Fetching leaderboard from ${LB_URL} …`);
  const rows = await fetchJson(
    `${LB_URL}/rest/v1/leaderboard?select=player,level_id,level_name,moves,dist,solution,submitted_at&order=moves.asc,dist.asc&limit=2000`
  );
  console.log(`Fetched ${rows.length} rows. Validating with the kernel collision check …\n`);

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
    console.log('All leaderboard solutions pass the kernel collision check. ✓');
    return;
  }

  console.log('=== FAILURES ===');
  for (const { row, lv, result } of failures) {
    const ts  = row.submitted_at ? row.submitted_at.slice(0, 10) : '?';
    const why = result.collision
      ? `collision on move #${result.moveIdx + 1} (steer=${+deg(result.move.steer).toFixed(1)}° dist=${result.move.dist}m) near (${result.point.x.toFixed(2)}, ${result.point.y.toFixed(2)})`
      : 'missed goal';
    console.log(`  FAIL  "${lv.name}" (${lv.id || row.level_id})  player=${row.player}  moves=${row.moves}  dist=${row.dist}  date=${ts}`);
    console.log(`        reason: ${why}`);
    console.log(`        solution: ${row.solution}`);
  }
}

main().catch(err => { console.error('Error:', err.message); process.exit(1); });
