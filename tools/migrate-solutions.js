#!/usr/bin/env node
// Migrate leaderboard `solution` column from legacy base64-JSON format to the
// new compact format ("steer:dist,steer:dist,…"). Safe to run multiple times —
// rows already in the new format (contain ':') are skipped.
//
// Usage:
//   LB_KEY=<service-role-PAT> node tools/migrate-solutions.js [--dry-run]
//
// Requires Node 18+ (built-in fetch). Set LB_KEY to the Supabase service-role
// key (not the anon key) so the RLS UPDATE policy is satisfied.

const LB_URL = 'https://qvjorkpzlwvswsptkwyn.supabase.co';
const LB_KEY = process.env.LB_KEY || '';
const DRY_RUN = process.argv.includes('--dry-run');

if (!LB_KEY) { console.error('Set LB_KEY env var to your Supabase service-role key'); process.exit(1); }

const headers = {
  apikey: LB_KEY,
  Authorization: `Bearer ${LB_KEY}`,
  'Content-Type': 'application/json',
  Prefer: 'return=minimal',
};

// ── Decode legacy base64-JSON solution ───────────────────────────────────────
function decodeOld(str) {
  let s = str.replace(/-/g, '+').replace(/_/g, '/');
  while (s.length % 4) s += '=';
  const bin = atob(s);
  const bytes = Uint8Array.from(bin, c => c.charCodeAt(0));
  const arr = JSON.parse(new TextDecoder().decode(bytes));
  if (!Array.isArray(arr)) throw new Error('not an array');
  return arr; // [[steer_deg, dist], …]
}

// ── Encode new compact format ─────────────────────────────────────────────────
function encodeNew(arr) {
  return arr.map(([steer, dist]) => +steer.toFixed(1) + ':' + +dist.toFixed(2)).join(',');
}

// ── Fetch all rows with a non-null solution ───────────────────────────────────
async function fetchRows() {
  const params = new URLSearchParams({
    select: 'id,solution',
    solution: 'not.is.null',
    order: 'id.asc',
    limit: '10000',
  });
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard?${params}`, { headers });
  if (!r.ok) throw new Error(`Fetch failed: ${r.status} ${await r.text()}`);
  return r.json();
}

// ── Patch a single row ────────────────────────────────────────────────────────
async function patchRow(id, solution) {
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard?id=eq.${id}`, {
    method: 'PATCH',
    headers,
    body: JSON.stringify({ solution }),
  });
  if (!r.ok) throw new Error(`Patch ${id} failed: ${r.status} ${await r.text()}`);
}

// ── Main ──────────────────────────────────────────────────────────────────────
(async () => {
  console.log(DRY_RUN ? '[DRY RUN] ' : '' + 'Fetching rows…');
  const rows = await fetchRows();
  console.log(`Found ${rows.length} rows with solutions`);

  let skipped = 0, converted = 0, errored = 0;

  for (const row of rows) {
    const { id, solution } = row;
    // Already in new format if it contains ':'
    if (solution.includes(':')) { skipped++; continue; }

    let newSol;
    try {
      const arr = decodeOld(solution);
      newSol = encodeNew(arr);
    } catch (e) {
      console.error(`  Row ${id}: decode error — ${e.message}`);
      errored++;
      continue;
    }

    if (DRY_RUN) {
      console.log(`  [dry] Row ${id}: ${solution.slice(0,30)}… → ${newSol}`);
    } else {
      try {
        await patchRow(id, newSol);
        process.stdout.write('.');
      } catch (e) {
        console.error(`\n  Row ${id}: patch error — ${e.message}`);
        errored++;
        continue;
      }
    }
    converted++;
  }

  console.log(`\nDone. Converted: ${converted}, Skipped (already new): ${skipped}, Errors: ${errored}`);
})();
