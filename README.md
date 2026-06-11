# Park Planner

A turn-based, top-down car parking puzzle. Instead of driving in real time,
you plan a sequence of moves — like plotting orders in a tactics game — then
run the whole plan and watch it play out. The goal is to optimize the plan:
fewest moves, or shortest distance, depending on the level.

## How it works

- The car uses a kinematic bicycle model: 2.7 m wheelbase, steering limited
  to ±35°, so every move is a true circular arc — exactly like a real car,
  including the mirrored geometry when reversing.
- A **move** = one steering angle + one signed distance (forward or reverse).
- While you adjust a move you see a live ghost preview of the path. If the
  move would clip anything, the path turns red and a pulsing marker shows the
  exact contact point — colliding moves can't be added.
- **Add move** commits it to the plan, **Undo**/**Reset** edit it freely,
  **Run** animates the full plan from the start. The plan stays editable
  after a run, so you can keep shaving moves/meters for 3 stars.

## Controls (mobile-first)

- Drag directly on the playfield: vertical = distance, horizontal = steering.
- Or use the two big sliders (steering, distance — left of center = reverse).
- Best scores per level are kept in localStorage.

## Levels

1. **First Steps** — get around a block (fewest moves, 3★ = 2).
2. **Parallel Squeeze** — parallel park in a 6.7 m gap (3★ = 3).
3. **Tight Bay** — back into a 3 m bay (3★ = 3).
4. **Dead End** — turn around in a 6.8 m corridor (shortest distance, 3★ ≤ 19 m).

Every 3★ threshold is verified reachable by exact simulation.

## Run it

No build step. Open `index.html` in a browser, or serve the folder:

```sh
python3 -m http.server 8000   # then open http://localhost:8000
```

Levels are plain data objects in `game.js` (`LEVELS`) — add your own by
defining walls, parked cars, a start pose and a goal zone.
