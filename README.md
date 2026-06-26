# Tactical Parking Simulator

A turn-based, top-down car-parking puzzle. Instead of driving in real time,
you plan a sequence of moves — like plotting orders in a tactics game — then
run the whole plan and watch it play out. The goal is to park in as few moves
as possible.

## How it works

- The car uses a kinematic bicycle model (2.7 m wheelbase, steering limited to
  ±35°), so every move is a true circular arc — exactly like a real car,
  including the mirrored geometry when reversing.
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

- `index.html` / `game.js` — the game. `physics.js` is the shared engine.
- `levels.js` — all levels as plain data (`LEVELS`), each with an `id`,
  geometry, and a verified `solution`.
- `editor.html` — standalone level editor (reads/writes `levels.js` via the
  GitHub API behind a PAT) with a beam/A\* **Solve** helper.

## Run it

No build step. Open `index.html` in a browser, or serve the folder:

```sh
python3 -m http.server 8000   # then open http://localhost:8000
```

## Build system

This project now includes a Vite + TypeScript build setup for gradual migration.

Install dependencies:

```sh
npm install
```

Run the development server:

```sh
npm run dev
```

Build for production:

```sh
npm run build
```
