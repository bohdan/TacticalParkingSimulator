# Next Session: Validate Leaderboard Solutions

## Context

The physics engine (`physics.js`) was upgraded to a **swept-hull collision check**
(convex hull of consecutive car poses instead of discrete point samples).  All
39 in-repo level solutions were re-validated and pass.  One level solution was
re-authored as a result (The Gauntlet).

The one remaining task is to validate every entry in the Supabase leaderboard to
make sure no stored player solution now collides or misses the goal under the new
check.  This couldn't be done in the previous session because the egress allowlist
for `qvjorkpzlwvswsptkwyn.supabase.co` was not yet applied.

## What to run

```
node tools/validate_leaderboard.js
```

This script:
1. GETs all rows from `leaderboard` (up to 2000, sorted best-first)
2. Parses each `solution` field (compact `"steer:dist,…"` or legacy base64)
3. Re-simulates every move through the current `simulateMove` (step = 0.005 m)
4. Reports rows that now collide or miss the goal

Expected output if everything is fine:
```
Fetching leaderboard … 
Fetched N rows. Validating with step=0.005 m …

Results: N pass, 0 fail, M no-solution, K unknown level (of N total rows)

All leaderboard solutions pass the swept-hull collision check. ✓
```

## If failures are found

A failure means a player's stored solution clips an obstacle or ends outside the
goal polygon under the stricter check.  These are *existing* leaderboard entries
— we can't change them retroactively, but we should:

1. Note which level(s) are affected and by how much (the script prints the
   offending move).
2. Decide whether to flag those rows in Supabase (e.g., add a `void` boolean
   column) or just document them.
3. If the fail is tiny (< 2 mm) and the level has been tightened unfairly by the
   new check, consider whether the level geometry should be relaxed by that amount
   instead of invalidating existing scores.

## Branch / repo

All physics and level changes are on `main`.  Work on `main` directly (the
feature branch `claude/vibeplayer-game-recovery-iwl6ji` mirrors it).

## Files changed in the previous session (for context)

- `physics.js` — added `convexHull()`, rewrote `simulateMove()` to swept hull
- `levels.js` — fixed Inner Bend Park solution, fixed 5 typos, moved Miata
  section to end, replaced The Gauntlet solution (8-move → clean 10-move)
- `editor.html` — Try button strips `solutions` array before encoding `#try=` URL
- `game.js` — `#try=` catch block now logs decode errors to console
- `index.html` — cache-bust hash updated
- `tools/validate_leaderboard.js` — this validation script (new)
