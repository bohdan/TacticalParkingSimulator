'use strict';

// physics.js (loaded before this file) provides all shared math and physics:
// CAR, SEDAN, VEHICLES, SAMPLE_STEP, rad, deg, clamp, normAng, advance,
// carPoly, polysCollide, ptSegDist, rectPoly, obbPoly, goalPoly, pointInPoly,
// centroid, contactPoint, simulateMove, buildLevel, inGoal, setVehicle.

/* ===================== Game-specific helpers ===================== */

// Min distance from any car corner to the goal zone boundary.
function parkingClearance(pose) {
  const cp = carPoly(pose);
  const zone = goalPoly(level.goal);
  let minGap = Infinity;
  for (const v of cp)
    for (let j = 0; j < zone.length; j++) {
      const a = zone[j], b = zone[(j+1) % zone.length];
      minGap = Math.min(minGap, ptSegDist(v.x, v.y, a.x, a.y, b.x, b.y));
    }
  return isFinite(minGap) ? minGap : 0;
}

/* ===================== Levels ===================== */

// Editor test level: passed via URL hash (#try=<base64url>) by editor.html.
// Scoped to this tab/URL only, so it never reorders the real level list.
let testLevelLoaded = false;
(()=>{
  try { localStorage.removeItem('parkplanner_testlevel'); } catch (e) {}
  const m = location.hash.match(/[#&]try=([A-Za-z0-9\-_]+)/);
  if (!m) return;
  try {
    let s = m[1].replace(/-/g, '+').replace(/_/g, '/');
    while (s.length % 4) s += '=';
    const lv = JSON.parse(new TextDecoder().decode(Uint8Array.from(atob(s), c => c.charCodeAt(0))));
    lv.name = '★ ' + lv.name;
    lv._isTest = true;
    LEVELS.unshift(lv);
    testLevelLoaded = true;
  } catch (e) {}
})();

/* ===================== Solution encode / decode ===================== */

// Encode a moves array [{steer (rad), dist}] to a URL-safe base64 string.
function movesToString(mvs) {
  // High precision so a loaded/shared plan replays to the same end pose.
  // Coarse rounding here used to accumulate enough drift to miss the goal.
  const arr = mvs.map(m => [+(deg(m.steer).toFixed(4)), +m.dist.toFixed(4)]);
  const bytes = new TextEncoder().encode(JSON.stringify(arr));
  let b = '';
  for (const byte of bytes) b += String.fromCharCode(byte);
  return btoa(b).replace(/\+/g, '-').replace(/\//g, '_').replace(/=/g, '');
}

// Decode back to [{steer (rad), dist}], returns null on bad input.
function movesFromString(str) {
  try {
    let s = str.replace(/-/g, '+').replace(/_/g, '/');
    while (s.length % 4) s += '=';
    const arr = JSON.parse(new TextDecoder().decode(
      Uint8Array.from(atob(s), c => c.charCodeAt(0))));
    if (!Array.isArray(arr)) return null;
    return arr.map(([steer, dist]) => ({ steer: rad(steer), dist }));
  } catch { return null; }
}

// Shared solution loaded via #sol= URL hash — applied after boot setLevel().
let _solHash = null;
(()=>{
  const m = location.hash.match(/[#&]sol=([A-Za-z0-9\-_]+)/);
  if (m) _solHash = movesFromString(m[1]);
})();

// Compact human-readable move format: "steer1:dist1,steer2:dist2,…"
// Used for the live URL hash and leaderboard storage.
function movesToCompact(mvs) {
  return mvs.map(m => +deg(m.steer).toFixed(1) + ':' + +m.dist.toFixed(2)).join(',');
}
function movesFromCompact(str) {
  if (!str) return [];
  return str.split(',').map(p => {
    const i = p.indexOf(':');
    return { steer: rad(+p.slice(0, i)), dist: +p.slice(i + 1) };
  });
}
// Decode either compact ("35:-4.9,…") or legacy base64 format.
function movesFromAny(str) {
  if (!str) return null;
  try {
    if (str.includes(':')) return movesFromCompact(str);
    return movesFromString(str);
  } catch { return null; }
}

// New-format game state in URL: #<level_id>  or  #<level_id>/<moves>
// Distinct from #sol= (old) and #try= (editor test) which contain "=".
let _gameHash = null;
(()=>{
  if (location.hash.includes('=')) return;  // old #sol= / #try= — handled elsewhere
  const m = location.hash.match(/^#([a-z0-9]{6})(\/(.*))?$/i);
  if (!m) return;
  try { _gameHash = { id: m[1], moves: m[3] ? movesFromCompact(m[3]) : [] }; }
  catch {}
})();

// ── Leaderboard (Supabase) ─────────────────────────────────────────────────
// Schema setup — run once in the Supabase SQL editor:
//
//   create table leaderboard (
//     id bigserial primary key,          -- row id (auto), unrelated to levels
//     player text not null,
//     level int, level_id text,          -- level_id = stable per-level key
//     level_name text,
//     moves int, dist real, time_s real, -- score; stars derived from moves/par
//     solution text,                     -- encoded moves for replay
//     submitted_at timestamptz default now()
//   );
//   alter table leaderboard enable row level security;
//   create policy "public read"   on leaderboard for select using (true);
//   create policy "public insert" on leaderboard for insert
//     with check (char_length(player) between 1 and 20);
//
// If upgrading an existing table:
//   alter table leaderboard add column if not exists solution text;
//   alter table leaderboard add column if not exists level_id text;
//   alter table leaderboard alter column stars drop not null; -- no longer sent
//   alter table leaderboard alter column mode  drop not null; -- deprecated
//
const LB_URL = 'https://qvjorkpzlwvswsptkwyn.supabase.co';
const LB_KEY = 'sb_publishable_geHaaCkSfPilYWV3fYQHQA_KZdYNrpC';

const V_MAX = 3.0;         // m/s top speed
const ACCEL = 2.0;         // m/s² acceleration / braking
const STEER_RATE_DS = 60;  // degrees per second
const DIR_CHANGE_T = 1.5;  // seconds per direction reversal

// Time for one move: trapezoid profile (accel from rest → cruise → brake to rest)
function moveTime(dist) {
  const d = Math.abs(dist);
  const dFull = V_MAX * V_MAX / ACCEL; // dist needed to reach V_MAX and brake
  if (d >= dFull) return 2 * V_MAX / ACCEL + (d - dFull) / V_MAX;
  return 2 * Math.sqrt(d / ACCEL);
}

function planTime(mvs) {
  let t = 0, prevDeg = 0, prevSign = 0;
  for (let i = 0; i < mvs.length; i++) {
    const m = mvs[i];
    const d = deg(m.steer);
    t += Math.abs(d - prevDeg) / STEER_RATE_DS;
    if (i > 0 && Math.sign(m.dist) !== prevSign) t += DIR_CHANGE_T;
    t += moveTime(m.dist);
    prevDeg = d; prevSign = Math.sign(m.dist);
  }
  return t;
}

/* ===================== Game state ===================== */

const $ = id => document.getElementById(id);
const cv = $('cv'), ctx = cv.getContext('2d');

// A "cutscene" level isn't a parking puzzle — it plays a briefing animation.
const isCutscene = def => !!def && def.type === 'cutscene';

// Draft levels live in levels.js for editing but are hidden from the game.
// The editor's "Try" injects a test level (possibly a draft) — never strip it.
for (let i = LEVELS.length - 1; i >= 0; i--)
  if (LEVELS[i].draft && !LEVELS[i]._isTest) LEVELS.splice(i, 1);

let levelIdx = testLevelLoaded
  ? 0  // start on the ★ test level when one was passed in
  : clamp(parseInt(localStorage.getItem('parking.level') || '0', 10) || 0, 0, LEVELS.length - 1);
let level = isCutscene(LEVELS[levelIdx]) ? null : buildLevel(LEVELS[levelIdx]);

let moves = [];        // [{steer (rad), dist (m)}]
let planSims = [];     // cached simulateMove result per move
let editSteer = 0;     // degrees, from slider
let editDist = 0;      // meters, signed
let distMin = 0, distMax = 0; // drivable range at current steer/start pose
let editSim = null;
let editSimOpp = null; // preview for the opposite direction (same |dist|, opposite sign)
let editIdx = null;    // index of the move being tweaked (null = composing next move)

let anim = null;       // {samples, cum, total, t0, speed}
let pendingLb = null;  // {levelIdx, stars, st} — awaiting leaderboard submit
let solutionUsed = false; // viewing the solution locks leaderboard until Reset
let view = { scale: 1, ox: 0, oy: 0 };

function planEnd() {
  return planSims.length ? planSims[planSims.length - 1].end : level.start;
}

// How far the car can roll from `pose` before hitting something — at most
// one full circle when steering, the field span when straight. Rounded down
// to the slider step so 0 stays reachable on the range input.
function driveLimit(pose, steer, dir) {
  const R = Math.abs(steer) < 1e-4 ? Infinity : Math.abs(CAR.wb / Math.tan(steer));
  const cap = Math.min(level.w + level.h, 2 * Math.PI * R);
  const n = Math.max(2, Math.ceil(cap / SAMPLE_STEP));
  const sim = simulateMove(pose, steer, dir * cap, level.obstacles);
  return Math.floor((sim.pts.length - 1) / n * cap * 10) / 10;
}

function updateHash() {
  if (!level || !level.id) return;
  const compact = movesToCompact(moves);
  history.replaceState(null, '', location.pathname + '#' + level.id + (compact ? '/' + compact : ''));
}

function recomputePlan() {
  planSims = [];
  let pose = level.start;
  for (const m of moves) {
    const sim = simulateMove(pose, m.steer, m.dist, level.obstacles);
    planSims.push(sim);
    pose = sim.end;
  }
  recomputeEdit();
  updateHUD();
  updateHash();
}

function recomputeEdit() {
  const startPose = editIdx !== null
    ? (editIdx === 0 ? level.start : planSims[editIdx - 1].end)
    : planEnd();
  const s = rad(editSteer);
  // Fixed symmetric range so the distance value is preserved when the player
  // adjusts the steering angle. Collision is shown via yellow highlight instead.
  const fieldRange = Math.max(40, Math.ceil(Math.hypot(level.w, level.h)));
  distMax = fieldRange; distMin = -fieldRange;
  if (Math.abs(editDist) > 0.01) {
    editSim    = simulateMove(startPose, s,  editDist, level.obstacles);
    editSimOpp = simulateMove(startPose, s, -editDist, level.obstacles);
  } else {
    editSim = editSimOpp = null;
  }
  const hit = !!(editSim?.hit);
  distEl.classList.toggle('hit', hit);
  $('distVal').textContent = editDist === 0 ? '—'
    : `${editDist < 0 ? 'Rev' : 'Fwd'} ${Math.abs(editDist).toFixed(2)} m${hit ? ' ⚠' : ''}`;
  // While editing a move the change is already live, so the button just closes
  // the edit; while composing it adds the pending move.
  $('addBtn').disabled = !!anim || (editIdx === null && (!editSim || editSim.pts.length < 2));
  $('addBtn').innerHTML = editIdx !== null ? '&#10003; Done' : '&#65291; Add move';
}

function planStats() {
  let dist = 0;
  for (const m of moves) dist += Math.abs(m.dist);
  return { moves: moves.length, dist, time: planTime(moves) };
}

// Par = target move count. Defaults to the recorded solution's length
// (the solver's optimal), or an explicit level.par override.
function levelPar() {
  if (typeof level.par === 'number') return level.par;
  if (level.solution && level.solution.length) return level.solution.length;
  return 4; // fallback for levels without a recorded solution
}

// Golf-style scoring: Par → 3★, Bogey (Par+1) → 2★, worse → 1★.
function starsForMoves(mvCount, par) {
  if (mvCount <= par) return 3;
  if (mvCount <= par + 1) return 2;
  return 1;
}
function computeStars(st) {
  return starsForMoves(st.moves, levelPar());
}

/* ===================== HUD ===================== */

function starStr(n, total = 3) {
  let s = '';
  for (let i = 0; i < total; i++) s += i < n ? '★' : '☆';
  return s;
}

// Difficulty emoji by tier — shown in the level dropdown and objective line.
const TIER_EMOJI = {
  Tutorial: '🎓', Easy: '🟢', Medium: '🔵', Hard: '🟠', Expert: '🔴',
};
const tierEmoji = def => TIER_EMOJI[def && def.tier] || '⚪';

// ── Level unlock progression ───────────────────────────────────────────────
// A level is unlocked when the player reaches it; the title of locked levels
// is hidden in the selector. `maxUnlocked` is the highest unlocked index.
let maxUnlocked = (() => {
  const m = parseInt(localStorage.getItem('parking.maxUnlocked') || '-1', 10);
  return isNaN(m) ? -1 : m;
})();
function setMaxUnlocked(idx) {
  if (idx > maxUnlocked) {
    maxUnlocked = idx;
    try { localStorage.setItem('parking.maxUnlocked', String(idx)); } catch (e) {}
  }
}
const isUnlocked = idx => isCutscene(LEVELS[idx]) || idx <= maxUnlocked;

// 1-based position counting only playable (non-cutscene) levels.
function playableRank(idx) {
  let r = 0;
  for (let i = 0; i <= idx; i++) if (!isCutscene(LEVELS[i])) r++;
  return r;
}

// Build the header dropdown: unlocked levels show emoji + name; locked ones
// hide their title and are disabled.
function rebuildLevelSelect() {
  const sel = $('lvSelect');
  if (!sel) return;
  let html = '';
  for (let i = 0; i < LEVELS.length; i++) {
    const def = LEVELS[i];
    if (isCutscene(def)) continue;        // cutscenes aren't selectable levels
    const n = playableRank(i);
    const label = isUnlocked(i)
      ? `${tierEmoji(def)} ${n}. ${def.name}`
      : `🔒 ${n}. ? ? ?`;
    html += `<option value="${i}"${i === levelIdx ? ' selected' : ''}` +
            `${isUnlocked(i) ? '' : ' disabled'}>${label}</option>`;
  }
  sel.innerHTML = html;
  sel.value = String(levelIdx);
}

function updateHUD() {
  $('objective').textContent = `${tierEmoji(level)} ${level.tier} · Par ${levelPar()}`;
  const planning = moves.length > 0 || Math.abs(editDist) > 0.01;
  const best = loadBest();
  if (planning) {
    const st = planStats();
    $('stats').innerHTML =
      `Moves <b>${st.moves}</b> / Par ${levelPar()} · ~${st.time.toFixed(1)}s` +
      (best ? ` · Best <span class="star">${starStr(best.stars)}</span>` : '');
  } else {
    $('stats').innerHTML = escHtml(level.tut || level.hint) +
      (best ? ` · <span class="star">${starStr(best.stars)}</span>` : '');
  }
  $('delBtn').disabled = (moves.length === 0 && editIdx === null && Math.abs(editDist) < 0.01) || !!anim;
  $('delBtn').innerHTML = editIdx !== null ? `&#128465; #${editIdx + 1}` : '&#128465; Delete';
  $('resetBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('goBtn').disabled = (moves.length === 0 && (!editSim || editSim.pts.length < 2)) || !!anim;
  renderMoveList();
}

// Horizontal strip of move chips; the active (being-edited) one is highlighted.
// The trailing slot shows the move currently being composed (live preview) when
// it has length, otherwise a ＋ to start one. Tapping a chip selects it.
function moveChip(num, steerDeg, dist, active, dataI) {
  const st = Math.abs(steerDeg) < 0.1 ? '0°' : `${+Math.abs(steerDeg).toFixed(1)}°${steerDeg < 0 ? 'L' : 'R'}`;
  const d = `${dist < 0 ? '−' : '+'}${+Math.abs(dist).toFixed(2)}`;
  return `<div class="mv-chip${active ? ' active' : ''}" data-i="${dataI}">` +
         `<span class="mv-n">${num}</span>${st} ${d}</div>`;
}
function renderMoveList() {
  const el = $('moveList');
  if (!el) return;
  const composing = editIdx === null;
  const pending = composing && Math.abs(editDist) >= 0.01;
  // Always render (at least the ＋ chip) so the strip keeps its height and the
  // panel never jumps when the first move appears.
  let html = '';
  for (let i = 0; i < moves.length; i++) {
    const active = editIdx === i;          // show the live edit on the active chip
    const sDeg = active ? editSteer : deg(moves[i].steer);
    const dist = active ? editDist : moves[i].dist;
    html += moveChip(i + 1, sDeg, dist, active, i);
  }
  html += pending
    ? moveChip(moves.length + 1, editSteer, editDist, true, 'new')
    : `<div class="mv-chip add${composing ? ' active' : ''}" data-i="new">&#65291;</div>`;
  el.innerHTML = html;
  const act = el.querySelector('.active');
  if (act) act.scrollIntoView({ inline: 'nearest', block: 'nearest' });
}

function loadBest() {
  try { return JSON.parse(localStorage.getItem(`parking.best.${levelIdx}`)); }
  catch (e) { return null; }
}

function saveBest(st, stars) {
  const prev = loadBest();
  const prevMoves = prev ? (prev.moves ?? 999) : Infinity;
  // Better = more stars, or same stars with fewer moves (time as tiebreaker).
  if (!prev || stars > prev.stars ||
      (stars === prev.stars && st.moves < prevMoves) ||
      (stars === prev.stars && st.moves === prevMoves && st.time < (prev.time || 999))) {
    localStorage.setItem(`parking.best.${levelIdx}`,
      JSON.stringify({ moves: st.moves, dist: st.dist, time: st.time, stars }));
  }
}

let toastTimer = null;
function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.remove('hidden');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add('hidden'), 1800);
}

/* ===================== Rendering ===================== */

function fitView() {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
  }
  const m = 8; // px margin
  view.scale = Math.min((w - 2 * m) / level.w, (h - 2 * m) / level.h);
  view.ox = (w - level.w * view.scale) / 2;
  view.oy = (h - level.h * view.scale) / 2;
  view.dpr = dpr;
}

function worldTransform() {
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
  ctx.translate(view.ox, view.oy);
  ctx.scale(view.scale, view.scale);
}

function screenTransform() {
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
}

function toScreen(p) {
  return { x: view.ox + p.x * view.scale, y: view.oy + p.y * view.scale };
}

function drawPoly(poly) {
  ctx.beginPath();
  ctx.moveTo(poly[0].x, poly[0].y);
  for (let i = 1; i < poly.length; i++) ctx.lineTo(poly[i].x, poly[i].y);
  ctx.closePath();
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function drawCarBody(pose, opts, spec) {
  spec = spec || CAR;
  const vtype = opts.vehicle || 'default';
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  const x0 = -spec.rOver, len = spec.len, w = spec.wid;
  // wheel offset from car centerline (inset by ~0.16 m regardless of vehicle width)
  const wy = w / 2 - 0.16;
  // wheel box scales with vehicle length; bus wheels are larger
  const wl = Math.min(0.9, len * 0.075), wt = Math.min(0.18, w * 0.10);

  if (opts.wheels && vtype === 'tractor') {
    // Rear: large drive wheels with tread and brass hub; outer edge pinned to bounding box.
    const rRad = 0.24, rLen = 0.65, rCy = w / 2 - rRad;
    // Front: narrow steered wheels.
    const fRad = 0.10, fLen = 0.28, fCy = w / 2 - fRad;
    for (const sign of [-1, 1]) {
      ctx.save(); ctx.translate(0, sign * rCy);
      ctx.fillStyle = '#0d0f14'; ctx.fillRect(-rLen / 2, -rRad, rLen, rRad * 2);
      ctx.fillStyle = '#1e2228';
      for (let g = 0; g < 5; g++) { // tread bands
        const ty = -rRad + rRad * 2 * (g + 0.25) / 5;
        ctx.fillRect(-rLen * 0.45, ty, rLen * 0.9, rRad * 0.22);
      }
      ctx.fillStyle = '#c8a030'; ctx.beginPath(); ctx.arc(0, 0, rRad * 0.44, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = '#111';    ctx.beginPath(); ctx.arc(0, 0, rRad * 0.20, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = '#c8a030';
      for (let i = 0; i < 6; i++) { // lug bolts
        const a = i * Math.PI / 3;
        ctx.beginPath(); ctx.arc(Math.cos(a) * rRad * 0.32, Math.sin(a) * rRad * 0.32, rRad * 0.07, 0, 2 * Math.PI); ctx.fill();
      }
      ctx.restore();
      ctx.save(); ctx.translate(spec.wb, sign * fCy); ctx.rotate(opts.steer || 0);
      ctx.fillStyle = '#0d0f14'; ctx.fillRect(-fLen / 2, -fRad, fLen, fRad * 2);
      ctx.fillStyle = '#888'; ctx.beginPath(); ctx.arc(0, 0, fRad * 0.40, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = '#333'; ctx.beginPath(); ctx.arc(0, 0, fRad * 0.18, 0, 2 * Math.PI); ctx.fill();
      ctx.restore();
    }
    // front axle beam (rotates with steer)
    ctx.save(); ctx.translate(spec.wb, 0); ctx.rotate(opts.steer || 0);
    ctx.fillStyle = '#3a4050';
    ctx.fillRect(-fLen * 0.5, -fCy, fLen, fCy * 2);
    ctx.restore();
  } else if (opts.wheels) {
    ctx.fillStyle = '#10131a';
    // Bus rides on a dual rear axle; others have a single rear pair.
    const axles = vtype === 'bus'
      ? [[spec.wb * 0.06, 0], [spec.wb * 0.92, opts.steer || 0], [spec.wb, opts.steer || 0]]
      : [[0, 0], [spec.wb, opts.steer || 0]];
    for (const [wx, a] of axles)
      for (const wya of [-wy, wy]) {
        ctx.save();
        ctx.translate(wx, wya);
        ctx.rotate(a);
        ctx.fillRect(-wl / 2, -wt, wl, wt * 2);
        ctx.restore();
      }
  }

  // Body — tractor is Lamborghini orange T-shape; Miata red; bus/sedan normal.
  if (vtype === 'tractor') {
    const jx = x0 + len * 0.54;
    const cabW = w * 0.80, hoodW = w * 0.44;
    ctx.fillStyle = '#d46020'; ctx.lineWidth = 0.07; ctx.strokeStyle = '#7a3500';
    roundRect(x0, -cabW / 2, jx - x0, cabW, 0.12); ctx.fill(); if (opts.stroke) ctx.stroke();
    roundRect(jx, -hoodW / 2, x0 + len - jx, hoodW, 0.10); ctx.fill(); if (opts.stroke) ctx.stroke();
  } else {
    const fill = vtype === 'miata' ? '#d23b3b' : opts.fill;
    const corner = vtype === 'bus' ? Math.min(0.18, w * 0.08) : Math.min(0.3, w * 0.17);
    roundRect(x0, -w / 2, len, w, corner);
    ctx.fillStyle = fill; ctx.fill();
    if (opts.stroke) {
      ctx.lineWidth = 0.07;
      ctx.strokeStyle = vtype === 'miata' ? '#7d1f1f' : opts.stroke;
      ctx.stroke();
    }
  }

  if (opts.detail) {
    if (vtype === 'bus')        drawBusDetail(x0, len, w);
    else if (vtype === 'miata') drawConvertibleDetail(x0, len, w);
    else if (vtype === 'tractor') drawTractorDetail(x0, len, w);
    else                        drawSedanDetail(x0, len, w);
  }
  ctx.restore();
}

// Top-down Lamborghini R480: open cab with ROPS arch, large rear wheels flanking.
function drawTractorDetail(x0, len, w) {
  const front = x0 + len;
  const jx = x0 + len * 0.54;
  const cabW = w * 0.80, hoodW = w * 0.44;

  // ROPS arch — thick bar across cab just behind the hood junction
  ctx.strokeStyle = '#9aab8a'; ctx.lineWidth = 0.13; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(jx - 0.12, -cabW * 0.44); ctx.lineTo(jx - 0.12, cabW * 0.44); ctx.stroke();

  // Operator platform floor
  ctx.fillStyle = 'rgba(10,8,5,0.58)';
  roundRect(x0 + len * 0.06, -cabW / 2 + 0.18, len * 0.44, cabW - 0.36, 0.10); ctx.fill();

  // Seat
  ctx.fillStyle = '#1a1210';
  roundRect(x0 + len * 0.10, -0.20, len * 0.14, 0.40, 0.07); ctx.fill();

  // Steering wheel
  ctx.strokeStyle = '#1a1c20'; ctx.lineWidth = 0.07;
  ctx.beginPath(); ctx.arc(x0 + len * 0.34, 0, 0.17, 0, 2 * Math.PI); ctx.stroke();

  // Hood vent slats
  ctx.strokeStyle = 'rgba(0,0,0,0.28)'; ctx.lineWidth = 0.04; ctx.lineCap = 'butt';
  const vS = jx + (front - jx) * 0.12, vE = front - 0.28;
  for (let v = -1; v <= 1; v++) {
    ctx.beginPath(); ctx.moveTo(vS, v * hoodW * 0.24); ctx.lineTo(vE, v * hoodW * 0.24); ctx.stroke();
  }

  // Exhaust stack (right side, mid-hood)
  const exX = jx + (front - jx) * 0.52, exY = hoodW / 2 - 0.13;
  ctx.fillStyle = '#111318'; ctx.beginPath(); ctx.arc(exX, exY, 0.09, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = '#2d2d2d'; ctx.beginPath(); ctx.arc(exX, exY, 0.055, 0, 2 * Math.PI); ctx.fill();

  // Headlights at front
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(front - 0.13, -hoodW / 2 + 0.06, 0.09, 0.16);
  ctx.fillRect(front - 0.13,  hoodW / 2 - 0.22, 0.09, 0.16);

  // Taillights at rear cab corners
  ctx.fillStyle = '#cc2020';
  ctx.fillRect(x0, -cabW / 2 + 0.06, 0.09, 0.14);
  ctx.fillRect(x0,  cabW / 2 - 0.20, 0.09, 0.14);
}

function drawSedanDetail(x0, len, w) {
  const wsX = x0 + len * 0.30, rwX = x0 + len * 0.09, glH = w - 0.44;
  ctx.fillStyle = 'rgba(8,12,18,0.45)';
  roundRect(wsX, -w / 2 + 0.22, Math.min(0.85, len * 0.20), glH, 0.15); ctx.fill();
  roundRect(rwX, -w / 2 + 0.25, Math.min(0.6, len * 0.13), glH, 0.15); ctx.fill();
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(x0 + len - 0.18, -w / 2 + 0.15, 0.12, Math.min(0.3, w * 0.17));
  ctx.fillRect(x0 + len - 0.18,  w / 2 - 0.45, 0.12, Math.min(0.3, w * 0.17));
}

// Top-down convertible: open cockpit (no roof), small raked windshield,
// two seats and a roll hoop behind them.
function drawConvertibleDetail(x0, len, w) {
  const cockpitX = x0 + len * 0.16, cockpitLen = len * 0.46;
  // open interior tub
  ctx.fillStyle = '#2a1010';
  roundRect(cockpitX, -w / 2 + 0.20, cockpitLen, w - 0.40, 0.12); ctx.fill();
  // two seats
  ctx.fillStyle = '#3a2424';
  const seatW = cockpitLen * 0.42, seatH = (w - 0.40) / 2 - 0.12;
  roundRect(cockpitX + cockpitLen * 0.12, -w / 2 + 0.30, seatW, seatH, 0.08); ctx.fill();
  roundRect(cockpitX + cockpitLen * 0.12,  0.06,          seatW, seatH, 0.08); ctx.fill();
  // raked windshield at the front of the cockpit
  ctx.fillStyle = 'rgba(150,200,230,0.55)';
  roundRect(cockpitX + cockpitLen - 0.04, -w / 2 + 0.24, 0.14, w - 0.48, 0.06); ctx.fill();
  // roll hoop behind the seats
  ctx.fillStyle = '#1a1414';
  ctx.fillRect(cockpitX - 0.02, -w / 2 + 0.30, 0.12, w - 0.60);
  // headlights
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(x0 + len - 0.16, -w / 2 + 0.14, 0.10, 0.26);
  ctx.fillRect(x0 + len - 0.16,  w / 2 - 0.40, 0.10, 0.26);
}

// Bus: full-width front windscreen, a long row of side windows on each
// flank, and a door line near the front.
function drawBusDetail(x0, len, w) {
  const front = x0 + len;
  // wraparound windscreen
  ctx.fillStyle = 'rgba(120,170,210,0.55)';
  roundRect(front - 0.5, -w / 2 + 0.18, 0.34, w - 0.36, 0.1); ctx.fill();
  // side window strips
  const winX = x0 + len * 0.12, winLen = len * 0.66, strip = 0.26;
  ctx.fillStyle = 'rgba(120,170,210,0.45)';
  roundRect(winX, -w / 2 + 0.14, winLen, strip, 0.08); ctx.fill();
  roundRect(winX,  w / 2 - 0.14 - strip, winLen, strip, 0.08); ctx.fill();
  // window mullions
  ctx.strokeStyle = 'rgba(20,30,40,0.5)'; ctx.lineWidth = 0.04;
  const n = 6;
  for (let i = 1; i < n; i++) {
    const mx = winX + winLen * i / n;
    ctx.beginPath(); ctx.moveTo(mx, -w / 2 + 0.14); ctx.lineTo(mx, -w / 2 + 0.14 + strip); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(mx,  w / 2 - 0.14 - strip); ctx.lineTo(mx, w / 2 - 0.14); ctx.stroke();
  }
  // front door
  ctx.fillStyle = 'rgba(30,40,52,0.7)';
  roundRect(front - 1.2, w / 2 - 0.16 - strip - 0.02, 0.5, strip + 0.04, 0.05); ctx.fill();
  // headlights
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(front - 0.12, -w / 2 + 0.16, 0.10, 0.3);
  ctx.fillRect(front - 0.12,  w / 2 - 0.46, 0.10, 0.3);
}

function drawGhost(pose, color, steer = 0) {
  const wy = CAR.wid / 2 - 0.13;
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  ctx.lineWidth = 0.07;
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  for (const [wx, wya, a] of [
    [0, -wy, 0], [0, wy, 0],
    [CAR.wb, -wy, steer], [CAR.wb, wy, steer],
  ]) {
    ctx.save();
    ctx.translate(wx, wya);
    ctx.rotate(a);
    ctx.strokeRect(-0.33, -0.13, 0.66, 0.26);
    ctx.restore();
  }
  ctx.restore();
  ctx.setLineDash([]);

  drawPoly(carPoly(pose));
  ctx.lineWidth = 0.07;
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  ctx.stroke();
  ctx.setLineDash([]);
  // heading notch at the nose
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const nx = pose.x + c * (CAR.wb + CAR.fOver), ny = pose.y + s * (CAR.wb + CAR.fOver);
  ctx.beginPath();
  ctx.moveTo(nx + c * 0.45, ny + s * 0.45);
  ctx.lineTo(nx - s * 0.3, ny + c * 0.3);
  ctx.lineTo(nx + s * 0.3, ny - c * 0.3);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function drawPath(pts, color, dashed, lw = 0.09) {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.lineWidth = lw;
  ctx.strokeStyle = color;
  if (dashed) ctx.setLineDash([0.35, 0.25]);
  ctx.stroke();
  ctx.setLineDash([]);
}

// Arc guides: always shown when wheels are turned and no animation running.
// Four solid curves = all 4 bounding-box corners swept path.
// Four dashed curves = all 4 wheel tracks (front-left, front-right, rear-left, rear-right).
function drawArcGuides(pose, steerRad) {
  const N = 60;
  const fwdLimit = driveLimit(pose, steerRad, 1);
  const bwdLimit = driveLimit(pose, steerRad, -1);
  const fLen = CAR.wb + CAR.fOver;
  const half = CAR.wid / 2;

  function sampleArc(limit, dir) {
    const cFL = [], cFR = [], cRL = [], cRR = [];
    const wFL = [], wFR = [], wRL = [], wRR = [];
    for (let i = 0; i <= N; i++) {
      const p = advance(pose, steerRad, dir * limit * i / N);
      const cs = Math.cos(p.h), sn = Math.sin(p.h);
      const w = (lx, ly) => ({ x: p.x + cs * lx - sn * ly, y: p.y + sn * lx + cs * ly });
      cFL.push(w(fLen,        half));
      cFR.push(w(fLen,       -half));
      cRL.push(w(-CAR.rOver,  half));
      cRR.push(w(-CAR.rOver, -half));
      wFL.push(w(CAR.wb,  half));
      wFR.push(w(CAR.wb, -half));
      wRL.push(w(0,        half));
      wRR.push(w(0,       -half));
    }
    return { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR };
  }

  for (const [limit, dir] of [[fwdLimit, 1], [bwdLimit, -1]]) {
    if (limit < 0.2) continue;
    const { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR } = sampleArc(limit, dir);
    const col  = dir > 0 ? 'rgba(69,196,255,0.28)' : 'rgba(255,159,67,0.28)';
    const wCol = dir > 0 ? 'rgba(69,196,255,0.50)' : 'rgba(255,159,67,0.50)';
    drawPath(cFL, col, false, 0.06);
    drawPath(cFR, col, false, 0.06);
    drawPath(cRL, col, false, 0.06);
    drawPath(cRR, col, false, 0.06);
    drawPath(wFL, wCol, true, 0.05);
    drawPath(wFR, wCol, true, 0.05);
    drawPath(wRL, wCol, true, 0.05);
    drawPath(wRR, wCol, true, 0.05);
  }
}

// Steering geometry overlay: the rear-axle axis (perpendicular to heading),
// the instantaneous turn centre sitting on it, and the radius lines from the
// rear axle and both front wheels to that centre. Each front wheel rolls
// perpendicular to its own radius line (classic Ackermann); the rear-axle →
// centre segment is the turn radius R = wheelbase / tan(steer).
function drawSteerGeometry(pose, steerRad, preview) {
  if (Math.abs(steerRad) < rad(0.5)) return;   // ~straight: centre at infinity
  const R = CAR.wb / Math.tan(steerRad);
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const ux = -s, uy = c;                       // rear-axle axis direction (toward centre)
  const O = { x: pose.x + R * ux, y: pose.y + R * uy };
  const sgn = Math.sign(R);
  const at = t => ({ x: pose.x + t * ux, y: pose.y + t * uy });
  const half = CAR.wid / 2 - 0.16;             // matches drawn wheel inset

  // rear-axle axis — thin, extends just past the centre and the opposite side
  drawPath([at(-sgn * 1.0), at(R + sgn * 1.0)], 'rgba(255,255,255,0.28)', false, 0.035);

  // the turn radius itself: rear-axle centre → turn centre
  drawPath([{ x: pose.x, y: pose.y }, O], 'rgba(120,220,255,0.8)', false, 0.045);

  // Front wheels turned to the pending steer at the *current* (move-start)
  // pose, so the steering angle is visible here too — not only on the preview.
  drawSteerWheels(pose, steerRad);

  // Radius lines (perpendicular to each front wheel) drawn from the *preview*
  // pose's front wheels to the shared turn centre. The whole arc orbits the
  // same O, so these line up with where the car is heading.
  const fp = preview || pose;
  const fc = Math.cos(fp.h), fs = Math.sin(fp.h);
  const fw = ly => ({ x: fp.x + CAR.wb * fc - ly * fs, y: fp.y + CAR.wb * fs + ly * fc });
  drawPath([fw(half),  O], 'rgba(255,255,255,0.28)', false, 0.03);
  drawPath([fw(-half), O], 'rgba(255,255,255,0.28)', false, 0.03);

  // turn-centre marker
  ctx.beginPath();
  ctx.arc(O.x, O.y, 0.12, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(120,220,255,0.9)';
  ctx.fill();
}

// Draw just the two front wheels at `pose`, rotated to `steerRad`, with a
// cyan outline so they read as part of the steering overlay.
function drawSteerWheels(pose, steerRad) {
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const wy = CAR.wid / 2 - 0.16;
  const wl = Math.min(0.9, CAR.len * 0.075), wt = Math.min(0.18, CAR.wid * 0.10);
  for (const ly of [wy, -wy]) {
    const wx = pose.x + CAR.wb * c - ly * s;
    const wyp = pose.y + CAR.wb * s + ly * c;
    ctx.save();
    ctx.translate(wx, wyp);
    ctx.rotate(pose.h + steerRad);
    ctx.fillStyle = '#10131a';
    ctx.fillRect(-wl / 2, -wt, wl, wt * 2);
    ctx.lineWidth = 0.03;
    ctx.strokeStyle = 'rgba(120,220,255,0.9)';
    ctx.strokeRect(-wl / 2, -wt, wl, wt * 2);
    ctx.restore();
  }
}

function drawArrow(x, y, ang, len, color) {
  const c = Math.cos(ang), s = Math.sin(ang);
  ctx.beginPath();
  ctx.moveTo(x - c * len / 2, y - s * len / 2);
  ctx.lineTo(x + c * len / 2, y + s * len / 2);
  ctx.lineWidth = 0.12;
  ctx.strokeStyle = color;
  ctx.stroke();
  const tx = x + c * len / 2, ty = y + s * len / 2;
  ctx.beginPath();
  ctx.moveTo(tx + c * 0.45, ty + s * 0.45);
  ctx.lineTo(tx - s * 0.28, ty + c * 0.28);
  ctx.lineTo(tx + s * 0.28, ty - c * 0.28);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

function draw(now) {
  // On a cutscene there's no puzzle to render — the briefing overlay covers
  // the screen. Keep the RAF alive so play resumes when we leave it.
  if (!level) { requestAnimationFrame(draw); return; }
  fitView();
  screenTransform();
  ctx.fillStyle = '#171a21';
  ctx.fillRect(0, 0, cv.clientWidth, cv.clientHeight);

  worldTransform();

  // asphalt
  ctx.fillStyle = '#23272f';
  ctx.fillRect(0, 0, level.w, level.h);
  // 1 m grid
  ctx.lineWidth = 0.02;
  ctx.strokeStyle = 'rgba(255,255,255,0.05)';
  ctx.beginPath();
  for (let x = 1; x < level.w; x++) { ctx.moveTo(x, 0); ctx.lineTo(x, level.h); }
  for (let y = 1; y < level.h; y++) { ctx.moveTo(0, y); ctx.lineTo(level.w, y); }
  ctx.stroke();

  // goal zone — yellow normally, green once the car will come to rest parked
  const g = level.goal;
  const gPoly = goalPoly(g);
  let restPose;
  if (anim) {
    const trav = Math.min(anim.total, (now - anim.t0) / 1000 * anim.speed);
    restPose = anim.samples[sampleAt(anim, trav)].pose;
  } else if (editSim && !editSim.hit) {
    restPose = editSim.end;
  } else {
    restPose = planEnd();
  }
  const parked = inGoal(restPose, g);
  const gStroke = parked ? '#3ddc84' : '#f2c84b';
  drawPoly(gPoly);
  ctx.fillStyle = parked ? 'rgba(61,220,132,0.12)' : 'rgba(242,200,75,0.10)';
  ctx.fill();
  ctx.lineWidth = 0.05;
  ctx.strokeStyle = gStroke;
  ctx.setLineDash([0.4, 0.28]);
  ctx.stroke();
  ctx.setLineDash([]);
  for (const hd of g.heads) {
    drawArrow(g.cx, g.cy, rad(hd), Math.min(g.w, g.h) * 0.45,
              parked ? 'rgba(61,220,132,0.7)' : 'rgba(242,200,75,0.75)');
  }

  // decorative traffic (non-collision, animated sedans outside parking zone)
  if (level.traffic) {
    const tSec = now / 1000;
    for (const tr of level.traffic) {
      const d = (tSec * tr.speed + tr.offset) % tr.loop;
      const tx = tr.x + Math.cos(tr.h) * d;
      const ty = tr.y + Math.sin(tr.h) * d;
      const rearX = tx - Math.cos(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      const rearY = ty - Math.sin(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      drawCarBody({ x: rearX, y: rearY, h: tr.h },
                  { fill: tr.color || '#4e5a6e', stroke: '#3a4255', detail: false, wheels: false },
                  SEDAN);
    }
  }

  // obstacles
  for (const o of level.obstacles) {
    if (o.kind === 'car') {
      const sp = o.carSpec || SEDAN;
      drawCarBody({ x: o.pose.cx - Math.cos(o.pose.h) * (sp.len / 2 - sp.rOver),
                    y: o.pose.cy - Math.sin(o.pose.h) * (sp.len / 2 - sp.rOver),
                    h: o.pose.h },
                  { fill: '#737d8c', stroke: '#525a66', detail: true, wheels: true,
                    vehicle: o.pose.type || 'default' }, sp);
    } else {
      drawPoly(o.poly);
      ctx.fillStyle = o.kind === 'curb' ? '#3a4148' : '#39404e';
      ctx.fill();
      ctx.lineWidth = 0.06;
      ctx.strokeStyle = o.kind === 'curb' ? '#4d565e' : '#4a5568';
      ctx.stroke();
    }
  }

  // start pad
  drawPoly(carPoly(level.start));
  ctx.lineWidth = 0.05;
  ctx.strokeStyle = 'rgba(255,255,255,0.22)';
  ctx.setLineDash([0.2, 0.2]);
  ctx.stroke();
  ctx.setLineDash([]);

  // arc guides (all 4 corners + all 4 wheels) — drawn first so they sit behind everything
  if (!anim) {
    drawArcGuides(editStartPose(), rad(editSteer));
    const previewPose = editSim ? (editSim.hit ? editSim.hit.pose : editSim.end) : null;
    drawSteerGeometry(editStartPose(), rad(editSteer), previewPose);
  }

  // committed plan: paths + ghosts
  // When editing move editIdx: skip that move's arc (replaced by live preview),
  // and dim any moves that come after it (their start poses will shift on commit).
  for (let i = 0; i < planSims.length; i++) {
    if (editIdx !== null && i === editIdx) continue;
    const sim = planSims[i];
    const dimmed = editIdx !== null && i > editIdx;
    ctx.save();
    if (dimmed) ctx.globalAlpha = 0.3;
    drawPath(sim.pts, moves[i].dist >= 0 ? 'rgba(69,196,255,0.85)' : 'rgba(255,159,67,0.85)',
             moves[i].dist < 0);
    ctx.restore();
  }
  for (let i = 0; i < planSims.length; i++) {
    if (editIdx !== null && i === editIdx) continue;
    const dimmed = editIdx !== null && i > editIdx;
    const isAnchor = editIdx !== null ? i === editIdx - 1 : i === planSims.length - 1 && !editSim;
    ctx.save();
    if (dimmed) ctx.globalAlpha = 0.3;
    drawGhost(planSims[i].end, isAnchor
      ? 'rgba(233,240,250,0.85)' : 'rgba(160,175,195,0.5)', moves[i].steer);
    ctx.restore();
  }

  // live edit preview (active direction)
  let hitInfo = null;
  if (editSim) {
    const bad = !!editSim.hit;
    drawPath(editSim.pts,
             bad ? 'rgba(255,82,82,0.9)'
                 : editDist >= 0 ? 'rgba(69,196,255,0.95)' : 'rgba(255,159,67,0.95)',
             editDist < 0);
    drawGhost(bad ? editSim.hit.pose : editSim.end,
              bad ? 'rgba(255,82,82,0.95)' : 'rgba(233,240,250,0.95)', rad(editSteer));
    if (bad) hitInfo = editSim.hit;
  }

  // collision marker (pulsing)
  if (hitInfo) {
    const t = (now % 900) / 900;
    const p = hitInfo.point;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 0.25 + t * 0.6, 0, 2 * Math.PI);
    ctx.lineWidth = 0.09;
    ctx.strokeStyle = `rgba(255,82,82,${1 - t})`;
    ctx.stroke();
    ctx.beginPath();
    for (let i = 0; i < 8; i++) {
      const a = i * Math.PI / 4, r1 = 0.14, r2 = 0.34;
      ctx.moveTo(p.x + Math.cos(a) * r1, p.y + Math.sin(a) * r1);
      ctx.lineTo(p.x + Math.cos(a) * r2, p.y + Math.sin(a) * r2);
    }
    ctx.lineWidth = 0.07;
    ctx.strokeStyle = '#ff5252';
    ctx.stroke();
  }

  // the car: animated along the plan, or sitting at start
  let carPose = level.start, carSteer = rad(editSteer);
  if (anim) {
    const trav = Math.min(anim.total, (now - anim.t0) / 1000 * anim.speed);
    const i = sampleAt(anim, trav);
    carPose = anim.samples[i].pose;
    carSteer = anim.samples[i].steer;
    if (trav >= anim.total) finishRun();
  }
  drawCarBody(carPose, { fill: '#4fc3f7', stroke: '#1c5f80', detail: true,
                         wheels: true, steer: carSteer, vehicle: level.vehicle || 'default' });

  // move numbers (screen space so text stays crisp)
  screenTransform();
  ctx.font = '700 12px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let i = 0; i < planSims.length; i++) {
    const e = planSims[i].end;
    const sp = toScreen({ x: e.x + Math.cos(e.h) * CAR.wb / 2,
                          y: e.y + Math.sin(e.h) * CAR.wb / 2 });
    const selected = editIdx === i;
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 9, 0, 2 * Math.PI);
    ctx.fillStyle = selected ? '#ffd700' : 'rgba(20,25,33,0.85)';
    ctx.fill();
    ctx.fillStyle = selected ? '#1a1400' : '#cfd9e6';
    ctx.fillText(String(i + 1), sp.x, sp.y + 0.5);
  }

  requestAnimationFrame(draw);
}

/* ===================== Run / animate ===================== */

// Distance the car actually covers in a move, given its simulation (a move
// that hits a wall stops short — simulateMove truncates at the contact point).
function traveledDist(dist, sim) {
  const n = Math.max(2, Math.ceil(Math.abs(dist) / SAMPLE_STEP));
  return dist * (sim.pts.length - 1) / n;
}

function startRun() {
  if (!moves.length || anim) return;
  editIdx = null;
  // Bake every move down to the distance actually travelled, so the plan that
  // runs (and gets saved/shared) is the precise sequence the car performs —
  // no over-the-limit values that silently truncate on replay.
  for (let i = 0; i < planSims.length; i++)
    moves[i].dist = Math.round(traveledDist(moves[i].dist, planSims[i]) * 100) / 100;
  moves = moves.filter(m => Math.abs(m.dist) >= 0.01); // drop any no-op moves
  recomputePlan();

  const samples = [];
  const cum = [];
  let total = 0;
  for (let i = 0; i < planSims.length; i++) {
    const sim = planSims[i];
    const step = Math.abs(moves[i].dist) / (sim.pts.length - 1);
    for (let j = (i === 0 ? 0 : 1); j < sim.pts.length; j++) {
      if (j > 0) total += step;
      samples.push({ pose: sim.pts[j], steer: moves[i].steer });
      cum.push(total);
    }
  }
  const speed = clamp(total / 3, 2.5, 7); // whole run in ~3 s
  anim = { samples, cum, total, t0: performance.now(), speed };
  updateHUD();
}

function sampleAt(a, trav) {
  let lo = 0, hi = a.cum.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (a.cum[mid] < trav) lo = mid + 1; else hi = mid;
  }
  return lo;
}

function parkRejectionReason(pose, goal) {
  const insideZone = carPoly(pose).every(v => pointInPoly(v, goalPoly(goal)));
  if (!insideZone) return 'Car is not fully inside the parking zone';
  const best = goal.heads.reduce((best, hd) => {
    const err = Math.abs(normAng(pose.h - rad(hd)));
    return err < best.err ? { err, hd } : best;
  }, { err: Infinity, hd: 0 });
  return `Angle off by ${deg(best.err).toFixed(0)}° — need to be within ${goal.tol}° of the target heading`;
}

function finishRun() {
  anim = null;
  updateHUD();
  const end = planEnd();
  if (inGoal(end, level.goal)) {
    const st = planStats();
    const stars = computeStars(st);
    saveBest(st, stars);
    // Clearing a level unlocks the next one.
    const nxt = nextPlayable(levelIdx, +1);
    if (nxt >= 0) { setMaxUnlocked(nxt); rebuildLevelSelect(); }
    $('ovTitle').textContent = 'Parked!';
    $('ovStars').innerHTML =
      starStr(stars).replace(/☆/g, '<span class="dim">★</span>');
    const best = loadBest();
    const par = levelPar();
    const overPar = st.moves - par;
    const parLabel = overPar <= 0 ? 'Par' : overPar === 1 ? 'Bogey' : `+${overPar}`;
    $('ovStats').innerHTML =
      `<div class="sc-row"><span class="sc-label">Moves</span><span class="sc-val">${st.moves} <span class="sc-note">/ Par ${par} · ${parLabel}</span></span></div>` +
      `<div class="sc-row"><span class="sc-label">Time</span><span class="sc-val">${st.time.toFixed(1)} s</span></div>` +
      (best && best.moves < st.moves
        ? `<div class="sub" style="margin-top:6px">Best: ${best.moves} moves &nbsp;·&nbsp; ${starStr(best.stars)}</div>`
        : '');
    $('ovTip').textContent = stars === 3 ? (overPar < 0 ? 'Under par!' : 'On par!') :
      `★★★ at Par ${par} move${par !== 1 ? 's' : ''}`;
    $('ovNext').style.display = nextPlayable(levelIdx, +1) >= 0 ? '' : 'none';
    pendingLb = (solutionUsed || !lbAllowed()) ? null : { levelIdx, stars, st: { ...st }, moves: [...moves] };
    $('ovSubmitRow').style.display = (lbAllowed() && !solutionUsed) ? '' : 'none';
    $('ovIneligRow').style.display = (lbAllowed() && solutionUsed) ? '' : 'none';
    $('ovSubmit').disabled = false;
    $('ovSubmit').textContent = 'Submit to leaderboard';
    $('overlay').classList.remove('hidden');
  } else {
    toast(parkRejectionReason(end, level.goal));
  }
}

/* ===================== Level switching ===================== */

// Next/previous level index that isn't a cutscene, in the given direction.
// Cutscenes are skipped during normal navigation (only "Replay intro" enters
// one explicitly). Returns -1 when there's no playable level that way.
function nextPlayable(from, dir) {
  for (let i = from + dir; i >= 0 && i < LEVELS.length; i += dir)
    if (!isCutscene(LEVELS[i])) return i;
  return -1;
}

function setLevel(i) {
  levelIdx = (i + LEVELS.length) % LEVELS.length;
  const def = LEVELS[levelIdx];
  // Don't persist progress while previewing a test level (shifted indices) or
  // while on a cutscene (so the daily intro never overwrites real progress).
  if (!testLevelLoaded && !isCutscene(def)) localStorage.setItem('parking.level', String(levelIdx));
  if (!isCutscene(def)) setMaxUnlocked(levelIdx); // reaching a level unlocks it
  if (isCutscene(def)) { level = null; showCutscene(def); return; }
  $('intro').classList.add('hidden');  // leaving a cutscene
  setVehicle(def.vehicle || 'default');
  level = buildLevel(def);
  moves = [];
  planSims = [];
  anim = null;
  editIdx = null;
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();   // also calls updateHash()
  rebuildLevelSelect();
}

/* ===================== Input ===================== */

const steerEl = $('steer'), distEl = $('dist');
steerEl.style.setProperty('--zero', '50%');
distEl.style.setProperty('--zero', '50%');

// Single input grid: every move value (from canvas drag OR sliders) is snapped
// here, so the on-screen number IS the stored value — fully reproducible, no
// hidden precision. STEER_Q/DIST_Q match the slider step and the display digits.
const STEER_Q = 0.2;  // degrees
const DIST_Q  = 0.05; // metres
const snapTo = (v, q) => Math.round(v / q) * q;

// Normalise a loaded plan to the displayed precision (steer on the input grid,
// distance to the 2-decimal display resolution). Distances are NOT snapped to
// the coarse input grid so baked/truncated values survive intact.
function quantizeMoves(mvs) {
  return mvs.map(m => ({
    steer: rad(+snapTo(deg(m.steer), STEER_Q).toFixed(1)),
    dist:  +m.dist.toFixed(2),
  }));
}

function setEdit(steerDeg, dist) {
  const s = Math.abs(steerDeg) < 0.1 ? 0 : snapTo(steerDeg, STEER_Q);
  editSteer = +clamp(s, -CAR.maxSteer, CAR.maxSteer).toFixed(1);
  editDist  = Math.abs(dist) < DIST_Q / 2 ? 0 : +snapTo(dist, DIST_Q).toFixed(2);
  $('steerVal').textContent = editSteer === 0 ? '0°'
    : `${Math.abs(editSteer).toFixed(1)}° ${editSteer < 0 ? 'left' : 'right'}`;
  // Editing an existing move applies live: write it and re-base the rest of the
  // plan immediately, so no Update step is needed.
  if (editIdx !== null) {
    moves[editIdx] = { steer: rad(editSteer), dist: editDist };
    recomputePlan();   // re-sims all moves (rebases successors) + recomputeEdit + updateHUD
  } else {
    recomputeEdit();
    updateHUD();
  }
}

// Relative-drag sliders: dragging accumulates a delta from the value at
// touch-start rather than jumping to the touch position. This means you can
// always make tiny adjustments from any starting angle/distance.
// sensitivity = value-units per pixel of horizontal drag.
function makeRelativeSlider(el, range, sensitivity, getVal, applyVal) {
  el.min = -range; el.max = range; el.step = 'any'; el.value = 0;
  let startX = null, startVal = 0;
  el.addEventListener('pointerdown', e => {
    startX = e.clientX;
    startVal = getVal();
    el.setPointerCapture(e.pointerId);
    el.value = startVal; // show current position before drag begins
  });
  el.addEventListener('pointermove', e => {
    if (startX === null) return;
    const newVal = startVal + (e.clientX - startX) * sensitivity;
    applyVal(newVal);
    el.value = clamp(newVal, -range, range); // thumb tracks value
  });
  const end = () => { startX = null; el.value = 0; };
  el.addEventListener('pointerup', end);
  el.addEventListener('lostpointercapture', end);
}

// Slider step == the input grid: one pixel = one grid step, so the slider can
// land on every value the readout shows (re-drag, it recentres, for big swings).
makeRelativeSlider(steerEl, 55, STEER_Q,  // 55° covers the tractor's tight lock
  () => editSteer,
  v  => setEdit(v, editDist));

makeRelativeSlider(distEl, 25, DIST_Q,
  () => editDist,
  v  => setEdit(editSteer, v));

function commitMove() {
  if (anim) return false;
  if (editIdx !== null) {
    editIdx = null;                 // the edit is already applied live
  } else {
    if (!editSim || editSim.pts.length < 2) return false;
    moves.push({ steer: rad(editSteer), dist: editDist });
  }
  setEdit(editSteer, 0);            // back to composing; keep the steering angle
  recomputePlan();
  return true;
}

$('addBtn').addEventListener('click', commitMove);

// Delete: the selected move (then its successors auto-rebase), else a pending
// edit, else the last move.
$('delBtn').addEventListener('click', () => {
  if (anim) return;
  if (editIdx !== null) {
    moves.splice(editIdx, 1);
    editIdx = null;
    setEdit(0, 0);
    recomputePlan();
  } else if (Math.abs(editDist) >= 0.01) {
    setEdit(editSteer, 0);
  } else if (moves.length) {
    moves.pop();
    recomputePlan();
  }
});

// Move chips: tap to edit that move; tap ＋ to compose a new one.
$('moveList').addEventListener('click', e => {
  if (anim) return;
  const chip = e.target.closest('.mv-chip');
  if (!chip) return;
  if (chip.dataset.i === 'new') commitMove(); // bank the in-progress move, start fresh
  else selectMove(+chip.dataset.i);           // selectMove banks any pending edit
});

$('resetBtn').addEventListener('click', () => {
  if (anim) return;
  editIdx = null;
  moves = [];
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
});

$('goBtn').addEventListener('click', () => {
  if (anim) return;
  // Auto-commit whatever's pending — a new move OR an unsaved edit to an
  // existing one — so the last move never needs an explicit Add/Update.
  commitMove();
  if (!moves.length) { toast('Add some moves first'); return; }
  startRun();
});

$('lvSelect').addEventListener('change', e => {
  const t = parseInt(e.target.value, 10);
  if (!isNaN(t) && isUnlocked(t)) setLevel(t);
  else rebuildLevelSelect(); // revert selection for locked picks
});

$('menuBtn').addEventListener('click', () => $('menuOverlay').classList.remove('hidden'));
$('menuClose').addEventListener('click', () => $('menuOverlay').classList.add('hidden'));
$('menuOverlay').addEventListener('click', e => {
  if (e.target === $('menuOverlay')) $('menuOverlay').classList.add('hidden');
});
$('menuHelp').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  $('helpOverlay').classList.remove('hidden');
});
$('menuSol').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  showSolution();
});
$('menuLb').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  if (!lbEnabled()) toast('Leaderboard not configured — see LB_URL / LB_KEY in game.js');
  else if (!lbAllowed()) toast('No leaderboard for tutorial levels');
  else openLeaderboard(levelIdx);
});
$('menuNewGame').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  if (!confirm('Start a new game? This resets your progress.')) return;
  try {
    localStorage.removeItem('parking.level');
    localStorage.removeItem('parking.maxUnlocked');
    localStorage.removeItem('parking.introDay');
  } catch (e) {}
  maxUnlocked = -1;
  const firstPlayable = nextPlayable(-1, +1);
  if (firstPlayable >= 0) setMaxUnlocked(firstPlayable);
  // Replay the intro; it flows into the first level when it ends.
  const ci = LEVELS.findIndex(isCutscene);
  setLevel(ci >= 0 ? ci : (firstPlayable >= 0 ? firstPlayable : 0));
});
$('helpClose').addEventListener('click', () => $('helpOverlay').classList.add('hidden'));
$('lbClose').addEventListener('click', () => $('lbOverlay').classList.add('hidden'));

$('ovSubmit').addEventListener('click', async () => {
  if (!pendingLb) return;
  const player = localStorage.getItem('parking.player');
  if (!player) {
    $('nameInput').value = '';
    $('nameOverlay').classList.remove('hidden');
    setTimeout(() => $('nameInput').focus(), 50);
    return;
  }
  await doLbSubmit(player);
});

$('nameCancel').addEventListener('click', () => $('nameOverlay').classList.add('hidden'));
$('nameOk').addEventListener('click', async () => {
  const name = $('nameInput').value.trim().slice(0, 20);
  if (!name) return;
  localStorage.setItem('parking.player', name);
  $('nameOverlay').classList.add('hidden');
  if (pendingLb) await doLbSubmit(name);
});
$('nameInput').addEventListener('keydown', e => { if (e.key === 'Enter') $('nameOk').click(); });

$('ovImprove').addEventListener('click', () => $('overlay').classList.add('hidden'));
$('ovShare').addEventListener('click', () => {
  const url = location.href;  // hash is already up-to-date via updateHash()
  navigator.clipboard?.writeText(url)
    .then(() => toast('Solution link copied!'))
    .catch(() => prompt('Copy this solution link:', url));
});
$('ovRetry').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  moves = [];
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
});
$('ovResetSol').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  moves = [];
  solutionUsed = false;
  setEdit(0, 0);
  recomputePlan();
});
$('ovNext').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  // Step to the very next item: a cutscene there will play, otherwise the
  // next level loads. (Cutscenes are part of the linear progression.)
  const t = levelIdx + 1;
  if (t < LEVELS.length) setLevel(t);
});

function selectMove(i) {
  if (anim) return;
  // Tapping the move you're already on banks it and closes the edit.
  if (editIdx === i) { commitMove(); return; }
  // Switching to another move banks whatever's in progress first (a new move
  // or an edit), so nothing needs an explicit Add/Update. Appends never shift
  // earlier indices, so `i` stays valid afterwards.
  commitMove();
  editIdx = i;
  setEdit(deg(moves[i].steer), moves[i].dist);
}

// ── Leaderboard functions ────────────────────────────────────────────────────
const lbEnabled = () => !!(LB_URL && LB_KEY);
// Tutorial levels are practice — they don't have a leaderboard.
const lbAllowed = (def = level) => lbEnabled() && !!def && def.tier !== 'Tutorial';

// Stable per-level key for the leaderboard: the level's id (survives rename /
// reorder), falling back to its name for any legacy level without an id.
const levelKey = idx => LEVELS[idx].id || LEVELS[idx].name;

function escHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

async function lbPost(levelIdx, player, stars, st, solutionStr) {
  const body = {
    player, level: levelIdx, level_id: levelKey(levelIdx), level_name: level.name,
    moves: st.moves,
    dist: +st.dist.toFixed(2), time_s: +st.time.toFixed(1),
    solution: solutionStr || null,
    submitted_at: new Date().toISOString(),
  };
  let r = await fetch(`${LB_URL}/rest/v1/leaderboard`, {
    method: 'POST',
    headers: {
      apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}`,
      'Content-Type': 'application/json', Prefer: 'return=minimal',
    },
    body: JSON.stringify(body),
  });
  if (!r.ok && r.status === 400) {
    // Newer columns (solution, level_id) may not exist yet — retry without them
    delete body.solution;
    delete body.level_id;
    r = await fetch(`${LB_URL}/rest/v1/leaderboard`, {
      method: 'POST',
      headers: {
        apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}`,
        'Content-Type': 'application/json', Prefer: 'return=minimal',
      },
      body: JSON.stringify(body),
    });
  }
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
}

async function lbGet(levelIdx) {
  const p = new URLSearchParams({
    select: 'player,moves,dist,solution,submitted_at',
    level_id: `eq.${levelKey(levelIdx)}`,
    order: 'moves.asc,dist.asc', limit: '100',
  });
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard?${p}`, {
    headers: { apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}` },
  });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

async function lbGetAll() {
  const p = new URLSearchParams({
    select: 'player,level_id,level_name,moves,dist,solution,submitted_at',
    order: 'moves.asc,dist.asc', limit: '500',
  });
  const r = await fetch(`${LB_URL}/rest/v1/leaderboard?${p}`, {
    headers: { apikey: LB_KEY, Authorization: `Bearer ${LB_KEY}` },
  });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

async function renderLbAll(allRows, autoSelectIdx) {
  // Build best-per-level map from already-fetched rows (sorted best-first)
  const bestByLevel = new Map();
  for (const r of allRows) {
    const key = r.level_id ?? '';
    if (!bestByLevel.has(key)) bestByLevel.set(key, r);
  }
  const playable = LEVELS.flatMap((l, i) =>
    (!l.draft && l.type !== 'cutscene') ? [{ l, i }] : []);
  $('lbTable').innerHTML =
    `<tr class="lb-head"><td class="lb-name">Level</td><td class="lb-name">Record</td>` +
    `<td class="lb-metric">Moves</td><td class="lb-metric">Dist</td><td></td></tr>` +
    playable.map(({ l, i }) => {
      const r = bestByLevel.get(l.id || l.name);
      const locked = !isUnlocked(i);
      const name = locked ? '???' : escHtml(l.name);
      const player = (!locked && r) ? escHtml(r.player) : '—';
      const movesStr = (!locked && r) ? r.moves : '—';
      const distStr = (!locked && r && r.dist != null) ? r.dist.toFixed(0) + 'm' : '—';
      const playBtn = (!locked && r?.solution)
        ? `<button class="lb-sol-btn" data-sol="${escHtml(r.solution)}" data-level-idx="${i}">&#9654;</button>` : '';
      const sel = i === autoSelectIdx ? ' lb-row-sel' : '';
      return `<tr class="lb-row${sel}" data-idx="${i}" style="cursor:pointer">` +
        `<td class="lb-name">${name}</td><td class="lb-name">${player}</td>` +
        `<td class="lb-metric">${movesStr}</td><td class="lb-metric">${distStr}</td>` +
        `<td>${playBtn}</td></tr>`;
    }).join('');
}

function renderLbDetail(idx, allRows) {
  const l = LEVELS[idx];
  const par = l.par ?? (l.solution ? l.solution.length : 4);
  $('lbCurTitle').textContent = l.name;
  const key = l.id || l.name;
  const seen = new Set();
  const top = allRows
    .filter(r => (r.level_id ?? '') === key && !seen.has(r.player) && seen.add(r.player))
    .slice(0, 10);
  const tbl = $('lbDetailTable');
  if (!top.length) {
    tbl.innerHTML = '<tr><td colspan="5" class="lb-empty">No entries yet — be first!</td></tr>';
    return;
  }
  tbl.innerHTML =
    `<tr class="lb-head"><td></td><td class="lb-name">Player</td><td class="lb-stars">★</td>` +
    `<td class="lb-metric">Moves</td><td class="lb-metric">Dist</td><td></td></tr>` +
    top.map((r, i) => {
      const cls = i === 0 ? 'lb-gold' : i === 1 ? 'lb-silver' : i === 2 ? 'lb-bronze' : '';
      const sc = starsForMoves(r.moves, par);
      const stars = '★'.repeat(sc) + `<span class="lb-dim">★</span>`.repeat(3 - sc);
      const playBtn = r.solution
        ? `<td><button class="lb-sol-btn" data-sol="${escHtml(r.solution)}" data-level-idx="${idx}">&#9654;</button></td>`
        : '<td></td>';
      const when = r.submitted_at ? new Date(r.submitted_at).toLocaleDateString() : '';
      const dist = r.dist != null ? r.dist.toFixed(0) + 'm' : '—';
      return `<tr class="${cls}" title="${when}"><td class="lb-rank">${i+1}</td>` +
        `<td class="lb-name">${escHtml(r.player)}</td>` +
        `<td class="lb-stars">${stars}</td><td class="lb-metric">${r.moves}</td>` +
        `<td class="lb-metric">${dist}</td>${playBtn}</tr>`;
    }).join('');
}

let _lbAllRows = [];

async function openLeaderboard(idx) {
  $('lbDetailTable').innerHTML = '<tr><td colspan="5" class="lb-empty">Loading…</td></tr>';
  $('lbTable').innerHTML = '<tr><td colspan="5" class="lb-empty">Loading…</td></tr>';
  $('lbOverlay').classList.remove('hidden');
  try {
    _lbAllRows = await lbGetAll();
    renderLbAll(_lbAllRows, idx);
    if (idx != null && isUnlocked(idx)) renderLbDetail(idx, _lbAllRows);
  } catch (e) {
    $('lbTable').innerHTML = `<tr><td colspan="5" class="lb-empty" style="color:#ff7070">Error: ${escHtml(e.message)}</td></tr>`;
  }
}

function lbLoadSolution(btn) {
  const mvs = movesFromAny(btn.dataset.sol);
  if (!mvs) { toast('Could not decode solution'); return; }
  $('lbOverlay').classList.add('hidden');
  const targetIdx = btn.dataset.levelIdx != null ? parseInt(btn.dataset.levelIdx, 10) : null;
  if (targetIdx != null && targetIdx !== levelIdx) setLevel(targetIdx);
  moves = quantizeMoves(mvs);
  solutionUsed = true;
  editIdx = null;
  setEdit(0, 0);
  recomputePlan();
  toast('Solution loaded — leaderboard disabled until Reset');
}

$('lbTable').addEventListener('click', e => {
  const btn = e.target.closest('.lb-sol-btn');
  if (btn) { lbLoadSolution(btn); return; }
  // Row click → update left column detail
  const row = e.target.closest('tr[data-idx]');
  if (!row) return;
  const idx = parseInt(row.dataset.idx, 10);
  if (!isUnlocked(idx)) return;
  $('lbTable').querySelectorAll('.lb-row-sel').forEach(r => r.classList.remove('lb-row-sel'));
  row.classList.add('lb-row-sel');
  renderLbDetail(idx, _lbAllRows);
});

$('lbDetailTable').addEventListener('click', e => {
  const btn = e.target.closest('.lb-sol-btn');
  if (btn) lbLoadSolution(btn);
});

async function doLbSubmit(player) {
  const { levelIdx: li, stars, st, moves: pendingMoves } = pendingLb;
  $('ovSubmit').disabled = true;
  $('ovSubmit').textContent = '⏳ Submitting…';
  try {
    await lbPost(li, player, stars, st, movesToCompact(pendingMoves));
    pendingLb = null;
    $('overlay').classList.add('hidden');
    await openLeaderboard(li);
  } catch (e) {
    toast(`Submit failed: ${e.message}`);
    $('ovSubmit').disabled = false;
    $('ovSubmit').textContent = 'Submit to leaderboard';
  }
}

function showSolution() {
  if (!level.solution) { toast('No solution on record for this level'); return; }
  if (anim) return;
  editIdx = null;
  solutionUsed = true;
  moves = quantizeMoves(level.solution.map(m => ({ steer: rad(m.steer), dist: m.dist })));
  setEdit(0, 0);
  recomputePlan();
  toast('Solution loaded — leaderboard disabled until Reset');
}


// Drag directly on the canvas: the ghost car chases the pointer. The arc
// from the current move's start pose through the pointer's world position
// determines both steering angle and signed distance, so dragging feels
// like placing the car where you want it to go.
// A tap (minimal movement) on a move badge selects it for tweaking.
function pointerToWorld(e) {
  const r = cv.getBoundingClientRect();
  return { x: (e.clientX - r.left - view.ox) / view.scale,
           y: (e.clientY - r.top - view.oy) / view.scale };
}

// Given a target point in world space, find the constant-steer arc from the
// edit start pose whose rear axle passes through it: a circle tangent to the
// pose heading. Returns {steer (deg), dist (m, signed)}.
function arcToPoint(pose, wp) {
  const dx = wp.x - pose.x, dy = wp.y - pose.y;
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const lx = dx * c + dy * s;    // forward component
  const ly = -dx * s + dy * c;   // lateral component (steer>0 side)
  if (Math.abs(ly) < 0.05) return { steer: 0, dist: lx };
  const R = (lx * lx + ly * ly) / (2 * ly);
  const th = Math.atan2(lx / R, (R - ly) / R);
  return { steer: deg(Math.atan(CAR.wb / R)), dist: R * th };
}

function editStartPose() {
  return editIdx !== null
    ? (editIdx === 0 ? level.start : planSims[editIdx - 1].end)
    : planEnd();
}

// Relative drag: the ghost is "grabbed" wherever it currently is and moves
// 1:1 with the finger in world space — no jump on touch, and the finger
// never has to cover the ghost. Double-tap commits the pending move.
let drag = null, lastTap = 0;
cv.addEventListener('pointerdown', e => {
  if (anim) return;
  cv.setPointerCapture(e.pointerId);
  const t = (editSim && Math.abs(editDist) > 0.01) ? editSim.end : editStartPose();
  drag = { x: e.clientX, y: e.clientY, tx: t.x, ty: t.y, moved: false };
});
cv.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
  if (Math.abs(dx) > 5 || Math.abs(dy) > 5) drag.moved = true;
  if (!drag.moved) return;
  const a = arcToPoint(editStartPose(),
    { x: drag.tx + dx / view.scale, y: drag.ty + dy / view.scale });
  setEdit(a.steer, a.dist);
});
cv.addEventListener('pointerup', e => {
  if (drag && !drag.moved) {
    const cx = e.clientX, cy = e.clientY;
    let hit = -1;
    for (let i = 0; i < planSims.length; i++) {
      const ep = planSims[i].end;
      const sp = toScreen({ x: ep.x + Math.cos(ep.h) * CAR.wb / 2,
                             y: ep.y + Math.sin(ep.h) * CAR.wb / 2 });
      if (Math.hypot(cx - sp.x, cy - sp.y) < 22) { hit = i; break; }
    }
    if (hit >= 0) {
      selectMove(hit);
    } else {
      const now = performance.now();
      if (now - lastTap < 350 && Math.abs(editDist) > 0.01) {
        if (commitMove()) toast('Move added');
        lastTap = 0;
      } else {
        lastTap = now;
        if (editIdx !== null) { editIdx = null; setEdit(0, 0); }
      }
    }
  }
  drag = null;
});
cv.addEventListener('pointercancel', () => { drag = null; });

document.addEventListener('gesturestart', e => e.preventDefault());

/* ===================== Cutscene / dashboard briefing ===================== */

// Default briefing text, used when a cutscene level omits its own `message`.
const DEFAULT_CUTSCENE_MSG = [
  '> MSG INCOMING', '', '  AGENT 7', '  VALET', '',
  '  PKG BY 0300.', '  NO SCRATCHES.', '', '> MISSION: GO.',
];
let cutsceneMessage = DEFAULT_CUTSCENE_MSG;

function showCutscene(def) {
  cutsceneMessage = (def.message && def.message.length) ? def.message : DEFAULT_CUTSCENE_MSG;
  playIntroDash();
}

let introAnimId = null;

function playIntroDash() {
  cancelAnimationFrame(introAnimId);
  $('intro').classList.remove('hidden');
  $('introGo').classList.add('hidden');

  const canvas = $('introCanvas');
  const c = canvas.getContext('2d');

  // Full device-pixel-ratio resolution — smooth rendering, no virtual grid.
  // Use window.innerWidth/Height (current visual viewport) and let CSS
  // width:100%;height:100% control the display size so it adapts if the
  // browser chrome shifts (URL bar show/hide on mobile).
  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const W = window.innerWidth;
  const H = window.innerHeight;
  canvas.width  = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  c.scale(dpr, dpr);

  const portrait = H > W * 1.1;

  // Layout bands
  const SKY_H = H * 0.37;
  const DASH_Y = SKY_H;
  const DASH_H = H * 0.46;

  // Centre CRT screen
  const SC_W = portrait ? W * 0.84 : Math.min(W * 0.52, H * 0.72);
  const SC_X = (W - SC_W) / 2;
  const SC_Y = DASH_Y + DASH_H * 0.09;
  const SC_H = DASH_H * 0.84;
  const SC_R = 8;

  const MSG = cutsceneMessage;
  const SC_PAD = Math.max(8, SC_W * 0.05);
  // Auto-fit font so the message fits the CRT screen on BOTH axes.
  const LINE_SP = 1.5;                                   // line-height multiplier
  const longest = MSG.reduce((m, l) => Math.max(m, l.length), 1);
  const fsByHeight = (SC_H - 2 * SC_PAD) / (MSG.length * LINE_SP);
  // Courier New advance width ≈ 0.6em per glyph.
  const fsByWidth  = (SC_W - 2 * SC_PAD) / (longest * 0.6);
  const FS = Math.max(7, Math.min(18, Math.floor(Math.min(fsByHeight, fsByWidth))));
  const LH = Math.ceil(FS * LINE_SP);

  // Side clusters only in landscape
  const showSide = !portrait;
  const SP_R  = showSide ? Math.min((SC_X - 12) * 0.82, DASH_H * 0.38) : 0;
  const SP_CX = SC_X / 2;
  const SP_CY = DASH_Y + DASH_H * 0.52;
  const RC_X  = Math.ceil(SC_X + SC_W + 8);
  const RC_W  = W - RC_X - 8;

  // Mirror
  const MIR_W = W * 0.28, MIR_H = Math.max(14, SKY_H * 0.18);
  const MIR_X = (W - MIR_W) / 2;

  // LEDs top-centre of dash
  const LED_XS = [-2,-1,0,1,2].map(i => W / 2 + i * W * 0.04);
  const LED_Y  = DASH_Y + 8;
  const LED_W  = Math.max(10, W * 0.024), LED_H = Math.max(5, H * 0.012);

  const rnd = n => { const x = Math.sin(n) * 1e4; return x - Math.floor(x); };

  // Rain
  const RAIN = Array.from({ length: Math.floor(W * 0.16) }, (_, i) => ({
    x: rnd(i * 7.3) * W, y0: rnd(i * 3.1) * SKY_H,
    spd: 55 + rnd(i * 5.7) * 90, len: 7 + rnd(i * 2.2) * 10,
  }));

  // Buildings with pre-generated windows
  const BLDGS = [
    [0.00,0.70,0.056],[0.04,0.53,0.038],[0.08,0.76,0.044],[0.12,0.65,0.033],
    [0.15,0.73,0.046],[0.19,0.56,0.036],[0.23,0.80,0.038],
    [0.63,0.73,0.046],[0.67,0.56,0.038],[0.71,0.74,0.046],[0.75,0.52,0.036],
    [0.79,0.70,0.053],[0.83,0.63,0.040],[0.88,0.77,0.050],[0.93,0.68,0.07],
  ].map(([xf, yf, wf]) => {
    const bx = xf * W, by = yf * SKY_H;
    const bw = Math.max(10, wf * W), bh = (1 - yf) * SKY_H;
    const cols = Math.max(1, Math.round(bw / 12)), rows = Math.max(1, Math.round(bh / 10));
    const wins = [];
    for (let ri = 0; ri < rows; ri++)
      for (let ci = 0; ci < cols; ci++)
        if (rnd((bx + ci) * 11 + (by + ri) * 7) < 0.32)
          wins.push({
            x: bx + (ci + 0.25) * (bw / cols), y: by + (ri + 0.3) * (bh / rows),
            w: bw / cols * 0.45, h: bh / rows * 0.5,
            col: rnd(bx + ci * 3.1 + ri) < 0.55 ? '#ffe880' : '#8899cc',
          });
    return { bx, by, bw, bh, wins };
  });

  let msgShown = false;
  const t0 = performance.now();

  function rrect(x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y); c.lineTo(x + w - r, y);
    c.arcTo(x + w, y, x + w, y + r, r); c.lineTo(x + w, y + h - r);
    c.arcTo(x + w, y + h, x + w - r, y + h, r); c.lineTo(x + r, y + h);
    c.arcTo(x, y + h, x, y + h - r, r); c.lineTo(x, y + r);
    c.arcTo(x, y, x + r, y, r); c.closePath();
  }

  function frame(now) {
    const ms = now - t0, t = ms / 1000;

    // ── Sky ───────────────────────────────────────────────────────────
    const skyGrd = c.createLinearGradient(0, 0, 0, SKY_H);
    skyGrd.addColorStop(0, '#05070d'); skyGrd.addColorStop(1, '#0b0f1c');
    c.fillStyle = skyGrd; c.fillRect(0, 0, W, SKY_H);

    for (const { bx, by, bw, bh, wins } of BLDGS) {
      c.fillStyle = '#090c18'; c.fillRect(bx, by, bw, bh);
      for (const w of wins) {
        c.fillStyle = w.col;
        c.globalAlpha = 0.5 + 0.12 * Math.sin(t * 0.4 + bx);
        c.fillRect(w.x, w.y, w.w, w.h);
      }
      c.globalAlpha = 1;
    }

    // Rearview mirror
    if (ms > 200) {
      c.fillStyle = '#1c2030';
      rrect(MIR_X, 4, MIR_W, MIR_H, 4); c.fill();
      c.fillStyle = '#0e1120';
      rrect(MIR_X + 2, 6, MIR_W - 4, MIR_H - 4, 3); c.fill();
      c.fillStyle = '#161924';
      c.fillRect(MIR_X + MIR_W * 0.08, 7, MIR_W * 0.16, MIR_H - 6);
      c.fillRect(MIR_X + MIR_W * 0.38, 7, MIR_W * 0.14, MIR_H - 6);
      c.fillRect(MIR_X + MIR_W * 0.68, 7, MIR_W * 0.16, MIR_H - 6);
    }

    // Rain
    if (ms > 400) {
      c.strokeStyle = 'rgba(100,150,220,0.45)'; c.lineWidth = 0.8;
      for (const dr of RAIN) {
        const y = (dr.y0 + dr.spd * t) % SKY_H;
        c.beginPath(); c.moveTo(dr.x, y); c.lineTo(dr.x - 1, y + dr.len); c.stroke();
      }
    }

    // ── Dashboard ─────────────────────────────────────────────────────
    const dashGrd = c.createLinearGradient(0, DASH_Y, 0, DASH_Y + DASH_H);
    dashGrd.addColorStop(0, '#181c27'); dashGrd.addColorStop(1, '#0e1118');
    c.fillStyle = dashGrd; c.fillRect(0, DASH_Y, W, H - DASH_Y);
    c.strokeStyle = '#252c3c'; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, DASH_Y); c.lineTo(W, DASH_Y); c.stroke();

    // LEDs
    {
      const blink = ms > 1600 && Math.floor(ms / 210) % 2 === 0;
      for (let i = 0; i < LED_XS.length; i++) {
        const lx = LED_XS[i] - LED_W / 2, on = i === 3 && blink;
        c.fillStyle = '#0d1018';
        rrect(lx - 1, LED_Y - 1, LED_W + 2, LED_H + 2, 2); c.fill();
        c.fillStyle = on ? '#4eff6a' : (i === 3 ? '#1a4a28' : '#1a2030');
        rrect(lx, LED_Y, LED_W, LED_H, 1); c.fill();
        if (on) {
          c.shadowColor = '#4eff6a'; c.shadowBlur = 10;
          rrect(lx, LED_Y, LED_W, LED_H, 1); c.fill();
          c.shadowBlur = 0;
        }
      }
    }

    // Speedometer (landscape only)
    if (showSide && SP_R > 8) {
      c.save();
      c.shadowColor = 'rgba(0,0,0,0.6)'; c.shadowBlur = 16;
      c.fillStyle = '#0b0e18';
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R, 0, Math.PI * 2); c.fill();
      c.shadowBlur = 0;
      const spGrd = c.createRadialGradient(SP_CX, SP_CY - SP_R * 0.3, 0, SP_CX, SP_CY, SP_R);
      spGrd.addColorStop(0, '#1c2232'); spGrd.addColorStop(1, '#0c0f1c');
      c.fillStyle = spGrd;
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R * 0.86, 0, Math.PI * 2); c.fill();
      for (let ti = 0; ti <= 10; ti++) {
        const a = Math.PI * 0.75 + ti * (Math.PI * 1.5 / 10), major = ti % 2 === 0;
        c.strokeStyle = major ? '#4a5a80' : '#28304e';
        c.lineWidth = major ? 1.5 : 0.8;
        const r1 = SP_R * 0.72, r2 = SP_R * (major ? 0.58 : 0.64);
        c.beginPath();
        c.moveTo(SP_CX + Math.cos(a) * r1, SP_CY + Math.sin(a) * r1);
        c.lineTo(SP_CX + Math.cos(a) * r2, SP_CY + Math.sin(a) * r2);
        c.stroke();
      }
      const na = Math.PI * 0.77;
      c.shadowColor = '#cc3333'; c.shadowBlur = 6;
      c.strokeStyle = '#dd3333'; c.lineWidth = 2;
      c.beginPath();
      c.moveTo(SP_CX + Math.cos(na + Math.PI) * SP_R * 0.14, SP_CY + Math.sin(na + Math.PI) * SP_R * 0.14);
      c.lineTo(SP_CX + Math.cos(na) * SP_R * 0.66, SP_CY + Math.sin(na) * SP_R * 0.66);
      c.stroke(); c.shadowBlur = 0;
      c.fillStyle = '#dd3333';
      c.beginPath(); c.arc(SP_CX, SP_CY, SP_R * 0.07, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#38486a'; c.font = `${Math.max(8, SP_R * 0.16)}px sans-serif`;
      c.textAlign = 'center'; c.textBaseline = 'alphabetic';
      c.fillText('km/h', SP_CX, SP_CY + SP_R * 0.42);
      c.restore();
    }

    // Fuel gauge (landscape only)
    if (showSide && RC_W > 24) {
      c.save();
      const rcH = DASH_H * 0.38, rcY = SC_Y;
      c.fillStyle = '#0c0f1c'; rrect(RC_X, rcY, RC_W, rcH, 6); c.fill();
      c.fillStyle = '#10131e'; rrect(RC_X + 2, rcY + 2, RC_W - 4, rcH - 4, 5); c.fill();
      c.fillStyle = '#2e3c54'; c.font = `bold ${Math.max(9, RC_W * 0.28)}px monospace`;
      c.textAlign = 'center'; c.textBaseline = 'top';
      c.fillText('FUEL', RC_X + RC_W / 2, rcY + 7);
      const fbY = rcY + rcH * 0.55, fbH = Math.max(3, H * 0.013);
      c.fillStyle = '#1c2436'; c.fillRect(RC_X + 5, fbY, RC_W - 10, fbH);
      c.fillStyle = '#b36a00'; c.fillRect(RC_X + 5, fbY, (RC_W - 10) * 0.14, fbH);
      c.textBaseline = 'alphabetic'; c.restore();
    }

    // ── CRT Screen ────────────────────────────────────────────────────
    c.shadowColor = 'rgba(0,0,0,0.75)'; c.shadowBlur = 24;
    c.fillStyle = '#0c1018'; rrect(SC_X - 5, SC_Y - 5, SC_W + 10, SC_H + 10, SC_R + 2); c.fill();
    c.shadowBlur = 0;
    const bezGrd = c.createLinearGradient(SC_X, SC_Y, SC_X, SC_Y + SC_H);
    bezGrd.addColorStop(0, '#1e2430'); bezGrd.addColorStop(1, '#131620');
    c.fillStyle = bezGrd; rrect(SC_X - 4, SC_Y - 4, SC_W + 8, SC_H + 8, SC_R + 1); c.fill();

    if (ms < 1400) {
      c.fillStyle = '#010208'; rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
    } else if (ms < 2100) {
      c.fillStyle = '#010208'; rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
      c.save(); rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.clip();
      const n = Math.floor((ms - 1400) / 700 * SC_W * SC_H * 0.05);
      for (let i = 0; i < n; i++) {
        c.fillStyle = Math.random() > 0.45 ? '#28b040' : '#102a18';
        c.fillRect(SC_X + Math.random() * SC_W, SC_Y + Math.random() * SC_H, 2, 2);
      }
      c.restore();
    } else if (ms < 2500) {
      const p = (ms - 2100) / 400;
      c.fillStyle = `rgb(${Math.floor(p*2)},${Math.floor(p*16)},${Math.floor(p*7)})`;
      rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.fill();
    } else {
      c.save(); rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.clip();
      const scrGrd = c.createRadialGradient(
        SC_X + SC_W * 0.5, SC_Y + SC_H * 0.35, 0,
        SC_X + SC_W * 0.5, SC_Y + SC_H * 0.5, SC_W * 0.7);
      scrGrd.addColorStop(0, '#021108'); scrGrd.addColorStop(1, '#010604');
      c.fillStyle = scrGrd; c.fillRect(SC_X, SC_Y, SC_W, SC_H);
      // Scanlines
      c.fillStyle = 'rgba(0,0,0,0.18)';
      for (let sy = SC_Y; sy < SC_Y + SC_H; sy += 3) c.fillRect(SC_X, sy, SC_W, 1);
      // Typed message
      c.fillStyle = '#3dfa65'; c.shadowColor = '#00ee55'; c.shadowBlur = 5;
      c.font = `${FS}px "Courier New", monospace`;
      c.textAlign = 'left'; c.textBaseline = 'top';
      let rem = Math.floor((ms - 2500) / 1000 * 18), done = true;
      for (let li = 0; li < MSG.length; li++) {
        const line = MSG[li];
        if (rem <= 0) { done = false; break; }
        const take = Math.max(1, line.length);
        if (rem >= take) {
          c.fillText(line, SC_X + SC_PAD, SC_Y + SC_PAD + li * LH); rem -= take;
        } else {
          c.fillText(line.slice(0, rem) + '█', SC_X + SC_PAD, SC_Y + SC_PAD + li * LH);
          rem = 0; done = false; break;
        }
      }
      c.shadowBlur = 0; c.restore();
      if (done && !msgShown) { msgShown = true; $('introGo').classList.remove('hidden'); }
      // Green glow rim
      c.shadowColor = 'rgba(0,200,60,0.4)'; c.shadowBlur = 20;
      c.strokeStyle = 'rgba(0,150,50,0.15)'; c.lineWidth = 2;
      rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.stroke();
      c.shadowBlur = 0;
    }

    // ── Steering wheel ────────────────────────────────────────────────
    {
      const wX = W / 2, wY = H + H * 0.22, wR = H * 0.44, hubR = wR * 0.13;
      c.strokeStyle = '#1e2334'; c.lineWidth = wR * 0.09;
      c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.12, Math.PI * 1.88); c.stroke();
      c.strokeStyle = '#2c3348'; c.lineWidth = wR * 0.04;
      c.beginPath(); c.arc(wX, wY, wR, Math.PI * 1.12, Math.PI * 1.88); c.stroke();
      c.strokeStyle = '#1a1e2c'; c.lineWidth = wR * 0.034;
      for (const a of [Math.PI * 1.22, Math.PI * 1.5, Math.PI * 1.78]) {
        c.beginPath();
        c.moveTo(wX + Math.cos(a) * hubR * 1.2, wY + Math.sin(a) * hubR * 1.2);
        c.lineTo(wX + Math.cos(a) * wR * 0.91, wY + Math.sin(a) * wR * 0.91);
        c.stroke();
      }
      const hGrd = c.createRadialGradient(wX - hubR * 0.3, wY - hubR * 0.3, 0, wX, wY, hubR);
      hGrd.addColorStop(0, '#2a3244'); hGrd.addColorStop(1, '#141824');
      c.fillStyle = hGrd;
      c.beginPath(); c.arc(wX, wY, hubR, 0, Math.PI * 2); c.fill();
    }

    if (!$('intro').classList.contains('hidden'))
      introAnimId = requestAnimationFrame(frame);
  }

  introAnimId = requestAnimationFrame(frame);
}

// A cutscene is a level — finishing it (Skip or Begin) advances to the next.
function endCutscene() {
  cancelAnimationFrame(introAnimId);
  introAnimId = null;
  $('introGo').classList.add('hidden');
  // Advance to the next item so consecutive cutscenes chain and the intro
  // flows straight into the first level.
  const t = levelIdx + 1;
  if (t < LEVELS.length) setLevel(t);
  else { const b = nextPlayable(levelIdx, -1); setLevel(b >= 0 ? b : 0); }
}
$('introSkip').addEventListener('click', endCutscene);
$('introGo').addEventListener('click', endCutscene);

/* ===================== Boot ===================== */

// Play the intro (first cutscene) at most once per calendar day. On the first
// visit of the day it runs, then resumes the saved level; afterwards it's
// skipped. A shared (#sol=) or test (#try=) link always skips the intro.
const INTRO_DAY_KEY = 'parking.introDay';
function introShownToday() {
  try { return localStorage.getItem(INTRO_DAY_KEY) === new Date().toDateString(); }
  catch (e) { return false; }
}
function markIntroShownToday() {
  try { localStorage.setItem(INTRO_DAY_KEY, new Date().toDateString()); } catch (e) {}
}

// Resume index: the saved level, never a cutscene.
let resumeIdx = levelIdx;
if (!testLevelLoaded && isCutscene(LEVELS[resumeIdx])) {
  const np = nextPlayable(resumeIdx, +1);
  resumeIdx = np >= 0 ? np : 0;
}

// Unlock everything up to where the player has already reached (migrates
// existing players who progressed before unlock-gating existed).
const firstPlayableIdx = nextPlayable(-1, +1);
if (firstPlayableIdx >= 0) setMaxUnlocked(firstPlayableIdx);
if (!testLevelLoaded) setMaxUnlocked(resumeIdx);

// Resolve which level to open: new-format hash (#id/moves) > saved progress > intro.
let _gameHashIdx = -1;
if (_gameHash) {
  _gameHashIdx = LEVELS.findIndex(l => l.id === _gameHash.id);
}

const introIdx = LEVELS.findIndex(isCutscene);
if (_gameHashIdx >= 0) {
  setLevel(_gameHashIdx);
} else if (!testLevelLoaded && !_solHash && introIdx >= 0 && !introShownToday()) {
  markIntroShownToday();
  setLevel(introIdx);     // intro plays, then flows into the first level
} else {
  setLevel(resumeIdx);
}

// Apply a shared solution from #sol= URL hash (set moves before first draw)
if (_solHash && level) {
  moves = quantizeMoves(_solHash);
  _solHash = null;
  recomputePlan();
  toast('Shared solution loaded');
}

// Apply moves from new-format hash (#<level_id>/<moves>)
if (_gameHash && _gameHashIdx >= 0 && level) {
  if (_gameHash.moves.length) {
    moves = quantizeMoves(_gameHash.moves);
    recomputePlan();
  }
  _gameHash = null;
}

requestAnimationFrame(draw);
