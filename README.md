# Tactical Parking Simulator

A turn-based, top-down car-parking puzzle. Instead of driving in real time,
you plan a sequence of moves — like plotting orders in a tactics game — then
run the whole plan and watch it play out. The goal is to park in as few moves
as possible.

## How it works

- The car uses a kinematic bicycle model, so every move is a true circular arc —
  exactly like a real car, including the mirrored geometry when reversing.
  Four vehicle types are available (default sedan, Miata, bus, tractor), each
  with its own wheelbase and maximum steering angle.
- A **move** = one steering angle + one signed distance (forward or reverse).
- While you adjust a move you see a live ghost preview of the path. If the move
  would clip an obstacle the path is shown hitting it; on **Run** each move is
  truncated to the distance actually driven, so the saved plan is exactly what
  the car does.
- The **turn strip** above the sliders lists every move; tap a chip (or a
  numbered badge on the canvas) to edit that move — later moves automatically
  re-base on top. **＋** starts a new move, **Delete** removes the selected one,
  **Reset** clears the plan, **Run** animates it from the start. The plan stays
  editable after a run.

## Scoring

Golf-style, against each level's **Par** (target move count):

- ★★★ — at or under Par
- ★★☆ — Par + 1 (bogey)
- ★☆☆ — parked

Best result per level is kept in `localStorage`, and there's an optional online
leaderboard (Supabase) ranked by moves, then drive time. Solutions can be
shared as `#sol=` links and replayed.

## Controls (mobile-first)

- Drag directly on the playfield: the ghost car follows your finger along a
  drivable arc (continuous, full precision).
- Or use the two relative sliders (steering, distance — left of centre =
  reverse); each step lands on the input grid.

## Project layout

```
physics-kernel.ts   — bicycle-model kinematics, collision, per-level PhysicsKernel
geometry2d.ts       — generic 2D geometry (SAT, hull, segment math)
scene.ts            — obstacle/goal construction from level definitions
solver.ts           — anytime min-turn A* + brute-force solver (bound to a kernel)
render.ts           — pure drawing layer (no physics globals)
game.ts             — game orchestration, input, HUD, animation loop
editor.ts           — standalone level editor with Solve helper
levels.ts           — all levels as plain data, each with id, geometry, solution
leaderboard.ts      — leaderboard UI and Supabase integration
solver-worker.ts    — Web Worker entry for parallel brute-force search
```

## Run it

Install dependencies and start the dev server:

```sh
npm install
npm run dev
```

Then open the URL printed by Vite (typically `http://localhost:5173`).

## Build for production

```sh
npm run build
```

Output goes to `dist/`. Serve any static file host.

## Numerical determinism (caveat)

Plans are stored as quantized moves (`STEER_Q` steering, `DIST_Q` distance), so the
*input* of a shared/leaderboard solution round-trips bit-for-bit on any machine. The
*simulation* of that plan, however, is **not guaranteed identical across JavaScript
engines**.

- IEEE-754 `+ − × ÷` and `sqrt` are correctly rounded and bit-identical on every
  engine and CPU (and JS never fuses multiply-add, so `a*b + c` is portable too).
- The trig the bicycle model leans on (`sin`/`cos`/`tan`/`atan2`/`hypot`) is **not**
  spec-mandated bit-exact — engines may differ by ~1 ULP, and so may different versions
  of the same engine. V8 (Chrome + Node) ships its own `fdlibm`, so all Chrome/Node
  builds agree across OS/CPU; a different engine (Firefox, Safari) can drift slightly.

In practice the grid-quantized inputs, the goal heading/zone tolerances, and the 1 µm
clean-touch collision margin (`COLLISION_EPS`) absorb this, so replays and leaderboard
validation stay consistent. The unguarded edge: a solution parked *exactly* on the fit
limit, or a truncation sitting *exactly* on a grid step (`clearGridDist` branches on a
trig-dependent collision test), could in principle flip pass/fail or shift by one step
on a different engine.

### Potential fix (not implemented)

Every trig *argument* is effectively quantized (per-steer angles), so the kinematics
can be made bit-identical across all engines:

1. Drive motion entirely from **baked** lookup tables — precomputed offline and shipped
   as literal constants, *not* recomputed at load with `Math.*` (which would just move
   the per-engine difference to startup). The kernel already precomputes local-frame arc
   deltas per `(steer, step)`; extend that to the per-steer rotation `(cos δ, sin δ)`.
2. Track orientation as a **unit vector** advanced by complex-multiply composition
   (`± × ÷` only), renormalized periodically via `sqrt`. No accumulated-heading
   `sin`/`cos` in the hot path.
3. Replace the collision arc test's `atan2` / angle comparisons with **cross/dot-product
   sign tests** (`± ×` only).

This removes the transcendental dependency entirely, giving provable cross-engine
lockstep — worth doing only if deterministic replays or an independent verifier ever
become a requirement.
