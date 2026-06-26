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
