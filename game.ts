// @ts-nocheck

// Shared math/physics come from the refactored components via the compat surface
// (PhysicsKernel / Geom2D / Scene).
import { CAR, SEDAN, setVehicle, SAMPLE_STEP, advance, carPoly, goalPoly, pointInPoly,
         simulateMove, buildLevel, inGoal, distCarToGoal, normAng, rad, deg, clamp } from './physics-compat.js';
import { LEVELS } from './levels.js';
import * as Leaderboard from './leaderboard.js';
import * as Renderer from './render.js';
import { showCutscene } from './cutscene.js';
// render-3d.ts pulls in all of Three.js, so it's its own lazy-loaded bundle chunk
// (dynamic import at each call site below) instead of bloating the main game bundle
// with a 3D library most page loads never touch.

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
  } catch (e) { console.warn('[try] Failed to load test level:', e); }
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
// A trailing ~ marks a URL written by this session (page reload of own state).
// Shared / leaderboard URLs won't have it, so we can lock the leaderboard there.
let _gameHash = null, _gameHashIsOwn = false;
(()=>{
  if (location.hash.includes('=')) return;  // old #sol= / #try= — handled elsewhere
  const raw = location.hash;
  // /~ prefix on the moves segment marks a URL written by this session (own reload).
  const m = raw.match(/^#([a-z0-9]{6})(\/(\~?)(.*))?$/i);
  if (!m) return;
  _gameHashIsOwn = m[3] === '~';
  try { _gameHash = { id: m[1], moves: m[4] ? movesFromCompact(m[4]) : [] }; }
  catch {}
})();

// ── Leaderboard (Supabase) ─────────────────────────────────────────────────
// The REST client + table schema live in leaderboard.js. init() applies the default
// public-project config; pass { url, key } to override.
Leaderboard.init();

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
let editNearHit = false; // current move clears, but one more step would collide (amber)
let editCleanSim = null; // when colliding: stable, grid-quantized pre-collision sim (for drawing)
let editEffD = 0;        // cached effective (display) distance of the edit
let editIdx = null;    // index of the move being tweaked (null = composing next move)

let anim = null;       // {samples, cum, total, t0, speed}
let pendingLb = null;  // {levelIdx, stars, st} — awaiting leaderboard submit
let solutionUsed = false; // viewing the solution locks leaderboard until Reset
let view = {
  scale: 1, ox: 0, oy: 0, dpr: 1,
  // Focus mode: t=0 normal view, t=1 fully focused; animates between
  t: 0, focused: false,
  focusX: 0, focusY: 0, focusAngle: 0,
  animating: false, animFrom: 0, animTo: 0, animT0: 0, animDur: 400,
  lastNow: 0,
};
// World units per screen pixel at the current frame's zoom — updated once per
// draw() so overlay lines (ghost, path, guides) stay cosmetically thin.
let drawPX = 1 / 60;

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
  const compact = movesToCompact(effectiveMoves());
  history.replaceState(null, '', location.pathname + '#' + level.id + (compact ? '/' + (solutionUsed ? '' : '~') + compact : ''));
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
  // adjusts the steering angle. Collision is shown via red highlight instead.
  const fieldRange = Math.max(40, Math.ceil(Math.hypot(level.w, level.h)));
  distMax = fieldRange; distMin = -fieldRange;
  if (Math.abs(editDist) > 0.01) {
    editSim    = simulateMove(startPose, s,  editDist, level.obstacles);
    editSimOpp = simulateMove(startPose, s, -editDist, level.obstacles);
  } else {
    editSim = editSimOpp = null;
  }
  const hit = !!(editSim?.hit);
  editNearHit = false;
  editCleanSim = null;
  if (!editSim) {
    editEffD = 0;
  } else if (hit) {
    // Collision: trim to the largest collision-free grid distance (stable, see
    // clearGridDist) and preview that pose — never the requested sim's wandering
    // truncation, which jumps as the sample grid shifts with the input distance.
    editEffD = clearGridDist(startPose, s, editDist, level.obstacles);
    editCleanSim = simulateMove(startPose, s, editEffD, level.obstacles);
  } else {
    editEffD = +(+editDist).toFixed(2);
    // Near-collision: this move clears, but advancing one more grid step (further
    // in the current direction of travel) would collide — warn before it bites.
    const next = simulateMove(startPose, s, editDist + Math.sign(editDist) * DIST_Q, level.obstacles);
    editNearHit = !!next.hit;
  }
  distEl.classList.toggle('hit', hit);
  distEl.classList.toggle('near', editNearHit);
  // Allow adding when steer is set even at zero distance (creates a 0-dist turn).
  const canAdd = editIdx !== null || (editSim && editSim.pts.length >= 2) || Math.abs(editSteer) >= STEER_Q;
  $('addBtn').disabled = !!anim || (editIdx === null && !canAdd);
  $('addBtn').innerHTML = editIdx !== null ? '&#10003; Done' : '&#65291; Add move';
}

function planStats() {
  let dist = 0;
  for (let i = 0; i < moves.length; i++) dist += Math.abs(moveEffDist(i));
  let count = moves.length;
  // Fold in the move being composed (not yet committed) so the running total
  // counts it too — using its truncated effective distance, like the chips.
  if (editIdx === null && Math.abs(editDist) > 0.01) {
    dist += Math.abs(editEffDist());
    count += 1;
  }
  return { moves: count, dist };
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
  const prevBtn = $('lvPrev'), nextBtn = $('lvNext');
  if (prevBtn) prevBtn.disabled = adjacentUnlocked(-1) < 0;
  if (nextBtn) nextBtn.disabled = adjacentUnlocked(+1) < 0;
}

function fmtGoalDist(d) {
  if (d <= 0) return '<span class="goal-dist goal-in">&#10003; In spot</span>';
  const val = d >= 1 ? `${d.toFixed(2)} m` : `${(d * 100).toFixed(2)} cm`;
  return `<span class="goal-dist">${val} outside</span>`;
}

function updateHUD() {
  const planning = moves.length > 0 || Math.abs(editDist) > 0.01;
  const desc = level.tut || level.hint || '';

  $('objective').innerHTML = desc ? escHtml(desc) : '';

  // Pre-collision pose of the active preview (the stable grid-trimmed one when it
  // collides), so the goal readout reflects where the move actually leaves the car.
  const previewSim = editSim ? (editSim.hit ? (editCleanSim || editSim) : editSim) : null;
  const endPose = previewSim ? previewSim.end : planEnd();
  const goalDistHtml = fmtGoalDist(distCarToGoal(endPose, level.goal));

  if (planning) {
    const st = planStats();
    $('parInfo').innerHTML = `Moves <b>${st.moves}</b> / Par ${levelPar()} · ${(st.dist * 100).toFixed(0)}cm · ${goalDistHtml}`;
  } else {
    $('parInfo').innerHTML = `Par ${levelPar()} · ${goalDistHtml}`;
  }
  $('stats').innerHTML = '';
  $('delBtn').disabled = (moves.length === 0 && editIdx === null && Math.abs(editDist) < 0.01) || !!anim;
  $('delBtn').innerHTML = editIdx !== null ? `&#128465; #${editIdx + 1}` : '&#128465; Delete';
  $('resetBtn').disabled = (moves.length === 0 && Math.abs(editDist) < 0.01 && editIdx === null) || !!anim;
  $('goBtn').disabled = (moves.length === 0 && (!editSim || editSim.pts.length < 2)) || !!anim;
  renderMoveList();
}

let _introTimer = null;
function showLevelIntro(text) {
  if (_introTimer) { clearTimeout(_introTimer); _introTimer = null; }
  const overlay = $('lvIntroOverlay');
  const textEl  = $('lvIntroText');
  if (!overlay || !text) return;
  // Reset any leftover inline styles
  overlay.style.cssText = '';
  textEl.style.cssText  = '';
  textEl.textContent = text;
  overlay.classList.remove('hidden');

  _introTimer = setTimeout(() => {
    _introTimer = null;
    // FLIP: measure where the text sits now (centered in overlay)
    const srcRect  = textEl.getBoundingClientRect();
    const destRect = $('objective').getBoundingClientRect();
    const dx = (destRect.left + destRect.width  / 2) - (srcRect.left + srcRect.width  / 2);
    const dy = (destRect.top  + destRect.height / 2) - (srcRect.top  + srcRect.height / 2);
    const s  = 12 / 20;   // destination font-size / intro font-size

    textEl.style.transition = 'transform 0.5s ease-in-out, opacity 0.4s ease-in-out, background-color 0.4s ease-in-out';
    textEl.style.transform       = `translate(${dx}px, ${dy}px) scale(${s})`;
    textEl.style.opacity         = '0';
    textEl.style.backgroundColor = 'transparent';

    _introTimer = setTimeout(() => {
      _introTimer = null;
      overlay.classList.add('hidden');
      overlay.style.cssText = '';
      textEl.style.cssText  = '';
    }, 560);
  }, 1150);
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
    html += moveChip(i + 1, sDeg, moveEffDist(i), active, i);
  }
  html += pending
    ? moveChip(moves.length + 1, editSteer, editEffDist(), true, 'new')
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
  const prevDist = prev ? (prev.dist ?? Infinity) : Infinity;
  // Better = more stars, or same stars with fewer moves (distance as tiebreaker).
  if (!prev || stars > prev.stars ||
      (stars === prev.stars && st.moves < prevMoves) ||
      (stars === prev.stars && st.moves === prevMoves && st.dist < prevDist)) {
    localStorage.setItem(`parking.best.${levelIdx}`,
      JSON.stringify({ moves: st.moves, dist: st.dist, stars }));
  }
}

// In-progress plan persistence: keyed by stable level id so a half-finished
// plan survives switching levels (and page reloads). Test-level previews and
// loaded solutions are never saved as drafts.
const draftKey = idx => `parking.draft.${levelKey(idx)}`;

function saveDraft() {
  if (testLevelLoaded || !level || solutionUsed) return;
  try {
    if (moves.length) localStorage.setItem(draftKey(levelIdx), movesToString(moves));
    else localStorage.removeItem(draftKey(levelIdx));
  } catch (e) {}
}

function loadDraft(idx) {
  try {
    const s = localStorage.getItem(draftKey(idx));
    const mvs = s ? movesFromString(s) : null;
    return mvs && mvs.length ? mvs : null;
  } catch (e) { return null; }
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
  const m = 8;
  view.scale = Math.min((w - 2 * m) / level.w, (h - 2 * m) / level.h);
  view.ox = (w - level.w * view.scale) / 2;
  view.oy = (h - level.h * view.scale) / 2;
  view.dpr = dpr;
}

// Focus-mode scale: show ≈10 m across the short screen dimension.
function focusViewScale() {
  return Math.min(cv.clientWidth, cv.clientHeight) / 10;
}

// Lerped view parameters shared by worldTransform / toScreen / pointerToWorld.
function viewParams() {
  const t  = view.t;
  const wX = level.w / 2 + (view.focusX - level.w / 2) * t;
  const wY = level.h / 2 + (view.focusY - level.h / 2) * t;
  return { wX, wY, ang: view.focusAngle * t,
           sc: view.scale + (focusViewScale() - view.scale) * t };
}

function worldTransform() {
  const W = cv.clientWidth, H = cv.clientHeight;
  const { wX, wY, ang, sc } = viewParams();
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
  ctx.translate(W / 2, H / 2);
  ctx.rotate(ang);
  ctx.scale(sc, sc);
  ctx.translate(-wX, -wY);
}

function screenTransform() {
  ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
}

function toScreen(p) {
  const W = cv.clientWidth, H = cv.clientHeight;
  const { wX, wY, ang, sc } = viewParams();
  const dx = p.x - wX, dy = p.y - wY;
  const c = Math.cos(ang), s = Math.sin(ang);
  return { x: W / 2 + (dx * c - dy * s) * sc,
           y: H / 2 + (dx * s + dy * c) * sc };
}

function draw(now) {
  // On a cutscene there's no puzzle to render — the briefing overlay covers
  // the screen. Keep the RAF alive so play resumes when we leave it.
  if (!level) { requestAnimationFrame(draw); return; }
  fitView();

  // Advance focus-mode animation + live tracking of the current turn's end.
  const dt = view.lastNow ? Math.min(0.05, (now - view.lastNow) / 1000) : 0;
  view.lastNow = now;
  if (view.animating) {
    const p = Math.min(1, (now - view.animT0) / view.animDur);
    view.t = view.animFrom + (view.animTo - view.animFrom) * easeIO2d(p);
    if (p >= 1) { view.t = view.animTo; view.animating = false; }
  }
  trackFocus(dt);

  screenTransform();
  ctx.fillStyle = '#171a21';
  ctx.fillRect(0, 0, cv.clientWidth, cv.clientHeight);

  drawPX = 1 / viewParams().sc;
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

  // lane markings (cosmetic, no collision)
  if (level.markings && level.markings.length) {
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.5)';
    ctx.lineWidth = 0.10;
    ctx.lineCap = 'butt';
    ctx.setLineDash([1.0, 1.0]);
    for (const m of level.markings) {
      if (m.type !== 'lane' && m.type !== 'bay') continue;
      ctx.setLineDash(m.type === 'lane' ? [1.0, 1.0] : []);
      ctx.beginPath();
      ctx.moveTo(m.x, m.y);
      ctx.lineTo(m.x + Math.cos(m.ang) * m.len, m.y + Math.sin(m.ang) * m.len);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.setLineDash([]);
    ctx.restore();
  }

  // goal zone — yellow normally, green once the car will come to rest parked
  const g = level.goal;
  const gPoly = goalPoly(g);
  let restPose;
  if (anim) {
    const trav = Math.min(anim.total, (now - anim.t0) / 1000 * anim.speed);
    restPose = anim.samples[sampleAt(anim, trav)].pose;
  } else if (editSim) {
    restPose = (editSim.hit ? (editCleanSim || editSim) : editSim).end;
  } else {
    restPose = planEnd();
  }
  const parked = inGoal(restPose, g);
  const gStroke = parked ? '#3ddc84' : '#f2c84b';
  Renderer.drawPolygon(ctx, gPoly);
  ctx.fillStyle = parked ? 'rgba(61,220,132,0.12)' : 'rgba(242,200,75,0.10)';
  ctx.fill();
  ctx.lineWidth = 0.05;
  ctx.strokeStyle = gStroke;
  ctx.setLineDash([0.4, 0.28]);
  ctx.stroke();
  ctx.setLineDash([]);

  // decorative traffic (non-collision, animated sedans outside parking zone)
  if (level.traffic) {
    const tSec = now / 1000;
    for (const tr of level.traffic) {
      const d = (tSec * tr.speed + tr.offset) % tr.loop;
      const tx = tr.x + Math.cos(tr.h) * d;
      const ty = tr.y + Math.sin(tr.h) * d;
      const rearX = tx - Math.cos(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      const rearY = ty - Math.sin(tr.h) * (SEDAN.len / 2 - SEDAN.rOver);
      Renderer.drawCarBody(ctx, { x: rearX, y: rearY, h: tr.h },
                  { fill: tr.color || '#4e5a6e', stroke: '#3a4255', detail: false, wheels: false },
                  SEDAN);
    }
  }

  // obstacles
  for (const o of level.obstacles) {
    if (o.kind === 'car') {
      const sp = o.carSpec || SEDAN;
      const obstVeh = o.pose.type || 'default';
      const sameAsTractor = obstVeh === 'tractor' && (level.vehicle || 'default') === 'tractor';
      // Default obstacle color is gray; only a lone tractor (player is NOT a
      // tractor) keeps its natural orange body.
      const obstFill = (obstVeh === 'tractor' && !sameAsTractor) ? undefined : '#737d8c';
      Renderer.drawCarBody(ctx, { x: o.pose.cx - Math.cos(o.pose.h) * (sp.len / 2 - sp.rOver),
                    y: o.pose.cy - Math.sin(o.pose.h) * (sp.len / 2 - sp.rOver),
                    h: o.pose.h },
                  { fill: obstFill, stroke: '#525a66', detail: true, wheels: true,
                    vehicle: obstVeh }, sp);
    } else {
      Renderer.drawPolygon(ctx, o.poly);
      ctx.fillStyle = o.kind === 'curb' ? '#3a4148' : '#39404e';
      ctx.fill();
      ctx.lineWidth = 0.06;
      ctx.strokeStyle = o.kind === 'curb' ? '#4d565e' : '#4a5568';
      ctx.stroke();
    }
  }

  // start pad
  Renderer.drawPolygon(ctx, carPoly(level.start));
  ctx.lineWidth = 0.05;
  ctx.strokeStyle = 'rgba(255,255,255,0.22)';
  ctx.setLineDash([0.2, 0.2]);
  ctx.stroke();
  ctx.setLineDash([]);

  // arc guides (all 4 corners + all 4 wheels) — drawn first so they sit behind everything
  if (!anim) {
    const gPose = editStartPose(), gSteer = rad(editSteer);
    Renderer.drawArcGuides(ctx, gPose, CAR, gSteer,
      driveLimit(gPose, gSteer, 1), driveLimit(gPose, gSteer, -1), advance, drawPX);
    const previewPose = editSim ? editSim.end : null;   // pre-collision end pose
    Renderer.drawSteeringGeometry(ctx, gPose, CAR, gSteer, previewPose, drawPX);
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
    Renderer.drawPath(ctx, sim.pts, moves[i].dist >= 0 ? 'rgba(69,196,255,0.85)' : 'rgba(255,159,67,0.85)',
             moves[i].dist < 0, 0.09, drawPX);
    ctx.restore();
  }
  for (let i = 0; i < planSims.length; i++) {
    if (editIdx !== null && i === editIdx) continue;
    const dimmed = editIdx !== null && i > editIdx;
    const isAnchor = editIdx !== null ? i === editIdx - 1 : i === planSims.length - 1 && !editSim;
    ctx.save();
    if (dimmed) ctx.globalAlpha = 0.3;
    // A committed move that collides parks the car red at its pre-collision pose.
    const ghostColor = planSims[i].hit ? 'rgba(255,82,82,0.85)'
      : isAnchor ? 'rgba(233,240,250,0.85)' : 'rgba(160,175,195,0.5)';
    Renderer.drawGhost(ctx, planSims[i].end, CAR, ghostColor, moves[i].steer, drawPX);
    ctx.restore();
  }

  // live edit preview (active direction)
  let hitInfo = null;
  if (editSim) {
    const bad = !!editSim.hit;
    // When colliding, draw the stable grid-trimmed preview (editCleanSim), not the
    // requested sim's wandering truncation; keep editSim.hit for the contact marker.
    const showSim = bad ? (editCleanSim || editSim) : editSim;
    Renderer.drawPath(ctx, showSim.pts,
             bad ? 'rgba(255,82,82,0.9)'
                 : editDist >= 0 ? 'rgba(69,196,255,0.95)' : 'rgba(255,159,67,0.95)',
             editDist < 0, 0.09, drawPX);
    // Ghost sits at the pre-collision pose: red if this move collides, amber if
    // one more step would, otherwise the normal bright preview.
    const ghostColor = bad ? 'rgba(255,82,82,0.95)'
      : editNearHit ? 'rgba(255,167,38,0.95)'
      : 'rgba(233,240,250,0.95)';
    Renderer.drawGhost(ctx, showSim.end, CAR, ghostColor, rad(editSteer), drawPX);
    if (bad) hitInfo = editSim.hit;
  }

  // collision marker (pulsing)
  if (hitInfo) {
    const t = (now % 900) / 900;
    const p = hitInfo.point;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 0.25 + t * 0.6, 0, 2 * Math.PI);
    ctx.lineWidth = Math.min(0.09, 3 * drawPX);
    ctx.strokeStyle = `rgba(255,82,82,${1 - t})`;
    ctx.stroke();
    ctx.beginPath();
    for (let i = 0; i < 8; i++) {
      const a = i * Math.PI / 4, r1 = 0.14, r2 = 0.34;
      ctx.moveTo(p.x + Math.cos(a) * r1, p.y + Math.sin(a) * r1);
      ctx.lineTo(p.x + Math.cos(a) * r2, p.y + Math.sin(a) * r2);
    }
    ctx.lineWidth = Math.min(0.07, 2.5 * drawPX);
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
  const pveh = level.vehicle || 'default';
  // Tractor keeps its natural orange body; other vehicles use the blue player color.
  Renderer.drawCarBody(ctx, carPose, { fill: pveh === 'tractor' ? undefined : '#4fc3f7',
                         stroke: '#1c5f80', detail: true,
                         wheels: true, steer: carSteer, vehicle: pveh }, CAR);

  // goal arrows drawn after the car so they're always visible
  for (const hd of g.heads) {
    Renderer.drawArrow(ctx, g.cx, g.cy, rad(hd), Math.min(g.w, g.h) * 0.45,
              parked ? 'rgba(61,220,132,0.7)' : 'rgba(242,200,75,0.75)');
  }

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

// Largest DIST_Q-grid distance the car can roll from `pose` at `steer` toward the
// sign of `maxDist` (up to |maxDist|) without colliding. Collision-freeness is
// monotonic in distance — the swept arc only grows — so a binary search over the
// fixed step grid yields a quantized, stable truncation: it depends only on the
// obstacle geometry, never on how far past the wall the move was requested. So it
// never jumps as the input changes, and never reports a distance shorter than one
// that is itself valid.
function clearGridDist(pose, steer, maxDist, obstacles) {
  const dir = maxDist < 0 ? -1 : 1;
  let lo = 0, hi = Math.floor(Math.abs(maxDist) / DIST_Q + 1e-6);
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (simulateMove(pose, steer, dir * mid * DIST_Q, obstacles).hit) hi = mid - 1;
    else lo = mid;
  }
  return +(dir * lo * DIST_Q).toFixed(2);
}

// Effective (non-collision) distance: when a move collides, the car keeps only the
// pre-collision travel, trimmed to the input grid. The full requested distance stays
// in `moves`/`editDist` so a later steer change can re-extend the move, but display,
// playback and saving all use this trimmed value.
function moveEffDist(i) {
  const sim = planSims[i];
  // No sim yet (planSims out of sync with moves, e.g. mid-load) or a clean move:
  // use the raw distance. Only a collided move needs the grid-trim, and by then
  // planSims is built up to i, so planSims[i-1] is safe to read for the start pose.
  if (!(sim && sim.hit)) return +(+moves[i].dist).toFixed(2);
  const start = i === 0 ? level.start : planSims[i - 1].end;
  return clearGridDist(start, moves[i].steer, moves[i].dist, level.obstacles);
}
function editEffDist() {
  return +editEffD.toFixed(2);
}
// `moves` with each distance trimmed to its pre-collision travel — the form used
// for the URL hash and leaderboard (never the over-the-wall requested distance).
function effectiveMoves() {
  return moves.map((m, i) => ({ steer: m.steer, dist: moveEffDist(i) }));
}

function startRun() {
  if (!moves.length || anim) return;
  editIdx = null;
  if (view.focused) exitFocus(); // gently pull back to the full board for the run
  // Bake every move down to the distance actually travelled, so the plan that
  // runs (and gets saved/shared) is the precise sequence the car performs —
  // no over-the-limit values that silently truncate on replay. Baking to the
  // same grid-snapped effective distance the UI already shows.
  for (let i = 0; i < planSims.length; i++)
    moves[i].dist = moveEffDist(i);
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
      `<div class="sc-row"><span class="sc-label">Distance</span><span class="sc-val">${(st.dist * 100).toFixed(0)} cm</span></div>` +
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

// Nearest unlocked, non-cutscene level in `dir` from the current one (header arrows).
function adjacentUnlocked(dir) {
  for (let i = levelIdx + dir; i >= 0 && i < LEVELS.length; i += dir)
    if (!isCutscene(LEVELS[i]) && isUnlocked(i)) return i;
  return -1;
}

function setLevel(i) {
  const target = (i + LEVELS.length) % LEVELS.length;
  if (target !== levelIdx) saveDraft();  // stash the plan for the level we're leaving
  levelIdx = target;
  const def = LEVELS[levelIdx];
  // Don't persist progress while previewing a test level (shifted indices) or
  // while on a cutscene (so the daily intro never overwrites real progress).
  if (!testLevelLoaded && !isCutscene(def)) localStorage.setItem('parking.level', String(levelIdx));
  if (!isCutscene(def)) setMaxUnlocked(levelIdx); // reaching a level unlocks it
  if (isCutscene(def)) {
    level = null;
    showCutscene(def, () => {
      const t = levelIdx + 1;
      if (t < LEVELS.length) setLevel(t);
      else { const b = nextPlayable(levelIdx, -1); setLevel(b >= 0 ? b : 0); }
    });
    return;
  }
  $('intro').classList.add('hidden');  // leaving a cutscene
  setVehicle(def.vehicle || 'default');
  level = buildLevel(def);
  const draft = testLevelLoaded ? null : loadDraft(levelIdx);
  moves = draft ? quantizeMoves(draft) : [];
  planSims = [];
  anim = null;
  editIdx = null;
  solutionUsed = false;
  resetFocus();
  setEdit(0, 0);
  recomputePlan();   // also calls updateHash()
  rebuildLevelSelect();
  showLevelIntro(def.tut || def.hint || def.name || '');
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
  // Editing an existing move applies live: write it and re-base the rest of the
  // plan immediately, so no Update step is needed.
  if (editIdx !== null) {
    moves[editIdx] = { steer: rad(editSteer), dist: editDist };
    recomputePlan();   // re-sims all moves (rebases successors) + recomputeEdit + updateHUD
  } else {
    recomputeEdit();
    updateHUD();
  }
  scheduleFocusHint();
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

$('steerDec').addEventListener('click', () => setEdit(editSteer - STEER_Q, editDist));
$('steerInc').addEventListener('click', () => setEdit(editSteer + STEER_Q, editDist));
$('distDec').addEventListener('click',  () => setEdit(editSteer, editDist - DIST_Q));
$('distInc').addEventListener('click',  () => setEdit(editSteer, editDist + DIST_Q));

function commitMove() {
  if (anim) return false;
  if (editIdx !== null) {
    editIdx = null;                 // the edit is already applied live
  } else {
    if (!editSim && Math.abs(editSteer) < STEER_Q) return false;
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
    showDeleteTip();
  } else if (Math.abs(editDist) >= 0.01) {
    setEdit(editSteer, 0);
  } else if (moves.length) {
    moves.pop();
    recomputePlan();
    showDeleteTip();
  }
});
function showDeleteTip() {
  if (localStorage.getItem('parking.deleteTipSeen')) return;
  localStorage.setItem('parking.deleteTipSeen', '1');
  toast('Tip: tap a move chip to edit it in place instead of deleting');
}

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
  saveDraft();   // discard the stored plan too (moves is empty → removes the key)
});

let view3dActive = false;   // true while the 3-D visualisation is showing
let _3dSuppressClick = false; // long-press 2D run fired; eat the subsequent click

// render-3d.js (and the Three.js library it pulls in) is a separate, lazily-fetched
// bundle chunk so the initial page load stays small — but almost every player opens
// the 3D replay sooner or later, so start fetching it in the background as soon as the
// browser is idle, well before anyone clicks, instead of waiting for the click itself.
const render3dModule = new Promise(resolve => {
  const start = () => resolve(import('./render-3d.js'));
  ('requestIdleCallback' in window) ? requestIdleCallback(start) : setTimeout(start, 200);
});

// Click → 3D view
$('goBtn').addEventListener('click', () => {
  if (view3dActive) return;
  if (_3dSuppressClick) { _3dSuppressClick = false; return; }
  if (anim) return;
  commitMove();
  if (!moves.length) { toast('Add some moves first'); return; }
  editIdx = null;
  for (let i = 0; i < planSims.length; i++)
    moves[i].dist = moveEffDist(i);
  moves = moves.filter(m => Math.abs(m.dist) >= 0.01);
  recomputePlan();
  view3dActive = true;
  render3dModule.then(({ show3DView }) => show3DView({
    level, sedan: SEDAN, car: CAR, viewScale: view.scale, moves, planSims,
    cv, v3d: $('v3d'), cv3d: $('cv3d'), v3dClose: $('v3dClose'),
    planEnd, finishRun, toast, onClose: () => { view3dActive = false; } }));
});

// Long-press → 2D animated run
{ let _3dT = null;
  $('goBtn').addEventListener('pointerdown', () => {
    _3dT = setTimeout(() => {
      _3dT = null;
      if (view3dActive || anim) return;
      _3dSuppressClick = true;
      commitMove();
      if (!moves.length) { _3dSuppressClick = false; toast('Add some moves first'); return; }
      startRun();
    }, 650);
  });
  $('goBtn').addEventListener('pointerup',     () => { clearTimeout(_3dT); _3dT = null; });
  $('goBtn').addEventListener('pointercancel', () => { clearTimeout(_3dT); _3dT = null; });
  $('goBtn').addEventListener('contextmenu', e => e.preventDefault());
}

$('lvSelect').addEventListener('change', e => {
  const t = parseInt(e.target.value, 10);
  if (!isNaN(t) && isUnlocked(t)) setLevel(t);
  else rebuildLevelSelect(); // revert selection for locked picks
});

$('lvPrev').addEventListener('click', () => { const t = adjacentUnlocked(-1); if (t >= 0) setLevel(t); });
$('lvNext').addEventListener('click', () => { const t = adjacentUnlocked(+1); if (t >= 0) setLevel(t); });

// Persist the in-progress plan when the tab is hidden/closed too.
window.addEventListener('pagehide', saveDraft);

$('menuBtn').addEventListener('click', () => $('menuOverlay').classList.remove('hidden'));
$('menuClose').addEventListener('click', () => $('menuOverlay').classList.add('hidden'));
$('menuOverlay').addEventListener('click', e => {
  if (e.target === $('menuOverlay')) $('menuOverlay').classList.add('hidden');
});
$('menuHelp').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  $('helpOverlay').classList.remove('hidden');
});
function showConfirm(msg, onYes) {
  $('confirmMsg').textContent = msg;
  $('confirmOverlay').classList.remove('hidden');
  const yes = $('confirmYes'), no = $('confirmNo');
  const close = () => $('confirmOverlay').classList.add('hidden');
  yes.onclick = () => { close(); onYes(); };
  no.onclick  = close;
}
function _applyHintMove(solQ) {
  const n = moves.length;
  if (n >= solQ.length) { toast('No more hint moves'); return; }
  moves.push(solQ[n]);
  solutionUsed = true;
  editIdx = null;
  setEdit(0, 0);
  recomputePlan();
  toast(`Hint: move ${n + 1} of ${solQ.length}`);
}
function applyHint() {
  if (!level.solution || !level.solution.length) { toast('No hint for this level'); return; }
  if (anim) return;

  // Check whether existing moves are a prefix of the solution so we can
  // continue without resetting; otherwise warn before wiping the plan.
  const solQ = level.solution.map(m => {
    const [q] = quantizeMoves([{ steer: rad(m.steer), dist: m.dist }]); return q;
  });
  const onTrack = moves.length > 0 && moves.every((m, i) =>
    i < solQ.length &&
    Math.abs(m.steer - solQ[i].steer) < 0.001 &&
    Math.abs(m.dist  - solQ[i].dist)  < 0.001
  );

  if (moves.length > 0 && !onTrack) {
    showConfirm('Your moves don\'t follow the hint — reset and start from move 1?', () => {
      moves = []; editIdx = null; recomputePlan();
      _applyHintMove(solQ);
    });
    return;
  }

  _applyHintMove(solQ);
}
$('hintBtn').addEventListener('click', applyHint);
$('menuHint').addEventListener('click', () => { $('menuOverlay').classList.add('hidden'); applyHint(); });
$('menuSol').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  showSolution();
});
$('menuLb').addEventListener('click', () => {
  $('menuOverlay').classList.add('hidden');
  if (!lbEnabled()) toast('Leaderboard not configured — see leaderboard.js');
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
    for (let k = localStorage.length - 1; k >= 0; k--) {
      const key = localStorage.key(k);
      if (key && key.startsWith('parking.draft.')) localStorage.removeItem(key);
    }
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
$('ovReplay2d').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  startRun();
});
$('ovReplay3d').addEventListener('click', () => {
  $('overlay').classList.add('hidden');
  view3dActive = true;
  render3dModule.then(({ show3DView }) => show3DView({
    level, sedan: SEDAN, car: CAR, viewScale: view.scale, moves, planSims,
    cv, v3d: $('v3d'), cv3d: $('cv3d'), v3dClose: $('v3dClose'),
    planEnd, finishRun, toast, onClose: () => { view3dActive = false; } }));
});
$('ovShare').addEventListener('click', () => {
  const url = location.href.replace('/~', '/');  // strip session marker before sharing
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
const lbEnabled = () => Leaderboard.isEnabled();
// Tutorial levels are practice — they don't have a leaderboard.
const lbAllowed = (def = level) => lbEnabled() && !!def && def.tier !== 'Tutorial';

// Stable per-level key for the leaderboard: the level's id (survives rename /
// reorder), falling back to its name for any legacy level without an id.
const levelKey = idx => LEVELS[idx].id || LEVELS[idx].name;

function escHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

// Game-side adapters: map the current level/state onto the decoupled leaderboard API.
async function lbPost(levelIdx, player, stars, st, solutionStr) {
  return Leaderboard.submitToLeaderboard({
    player, level: levelIdx, levelId: levelKey(levelIdx), levelName: level.name,
    moves: st.moves, dist: st.dist, solution: solutionStr,
  });
}
const lbGet    = levelIdx => Leaderboard.loadLevelLeaderboard(levelKey(levelIdx));
const lbGetAll = () => Leaderboard.loadOverallLeaderboard();

async function renderLbAll(allRows, autoSelectIdx) {
  // Build best-per-level map from already-fetched rows (sorted best-first)
  const bestByLevel = new Map();
  for (const r of allRows) {
    const key = r.level_id ?? '';
    if (!bestByLevel.has(key)) bestByLevel.set(key, r);
  }
  const playable = LEVELS.flatMap((l, i) =>
    (!l.draft && l.type !== 'cutscene' && l.tier !== 'Tutorial') ? [{ l, i }] : []);
  $('lbTable').innerHTML =
    `<tr class="lb-head"><td class="lb-name">Level</td><td class="lb-name">Record</td>` +
    `<td class="lb-metric">Moves</td><td class="lb-metric">Dist</td><td></td></tr>` +
    playable.map(({ l, i }) => {
      const r = bestByLevel.get(l.id || l.name);
      const locked = !isUnlocked(i);
      const name = locked ? '???' : escHtml(l.name);
      const player = (!locked && r) ? escHtml(r.player) : '—';
      const movesStr = (!locked && r) ? r.moves : '—';
      const distStr = (!locked && r && r.dist != null) ? (r.dist * 100).toFixed(0) + 'cm' : '—';
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
      const dist = r.dist != null ? (r.dist * 100).toFixed(0) + 'cm' : '—';
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


/* ===================== Focus mode ===================== */

const easeIO2d = t => t < 0.5 ? 2*t*t : -1+(4-2*t)*t;

function startViewAnim(to, dur) {
  view.animFrom = view.t; view.animTo = to;
  view.animT0 = performance.now(); view.animDur = dur; view.animating = true;
}

// The pose the focus view tracks: the end of the *current turn* — the move
// being edited, the pending new move, or (with nothing pending) the plan end.
function currentTurnEnd() {
  if (editIdx !== null) return planSims[editIdx] ? planSims[editIdx].end : planEnd();
  if (editSim && Math.abs(editDist) > 0.01) return editSim.end;
  return planEnd();
}

// Desired view angle so the tracked car faces "up" on screen.
function focusAngleFor(pose) { return -(pose.h + Math.PI / 2); }

function enterFocus() {
  const pose = currentTurnEnd();
  // Snap to the target so the zoom-in animates from the right place.
  view.focusX     = pose.x;
  view.focusY     = pose.y;
  view.focusAngle = focusAngleFor(pose);
  view.focused    = true;
  startViewAnim(1, 420);
  hideFocusHint();
  $('focusBadge').classList.remove('hidden');
}

function exitFocus() {
  view.focused = false;
  startViewAnim(0, 320);
  $('focusBadge').classList.add('hidden');
}

function resetFocus() { // called on level change
  view.t = 0; view.focused = false; view.animating = false;
  $('focusBadge').classList.add('hidden');
}

// Smoothly glide the focus centre/angle toward the current turn's end. Called
// every frame while focus is active (or animating). Held still during an active
// canvas drag so the view doesn't chase its own input. dt in seconds.
function trackFocus(dt) {
  if (view.t <= 0 && !view.focused) return;
  if (drag && drag.moved) return;            // freeze while fine-dragging
  const pose = currentTurnEnd();
  const k    = 1 - Math.exp(-dt / 0.10);      // position: ~100 ms
  const kAng = 1 - Math.exp(-dt / 0.40);      // rotation: ~400 ms
  view.focusX += (pose.x - view.focusX) * k;
  view.focusY += (pose.y - view.focusY) * k;
  // Shortest-arc angle smoothing.
  let da = focusAngleFor(pose) - view.focusAngle;
  da = Math.atan2(Math.sin(da), Math.cos(da));
  view.focusAngle += da * kAng;
}

/* ===================== Focus hint ===================== */

let focusHintDismissed = false;
let focusHintTimer = null;

function showFocusHint() {
  if (focusHintDismissed || view.focused || anim) return;
  $('focusHint').classList.remove('hidden');
}

function hideFocusHint() {
  clearTimeout(focusHintTimer); focusHintTimer = null;
  $('focusHint').classList.add('hidden');
}

function scheduleFocusHint() {
  if (focusHintDismissed || view.focused) return;
  const d = Math.abs(editDist);
  if (d > 0.005 && d < 0.2) {
    if (!focusHintTimer) focusHintTimer = setTimeout(showFocusHint, 700);
  } else {
    hideFocusHint();
  }
}

$('focusHintClose').addEventListener('click', e => {
  e.stopPropagation(); focusHintDismissed = true; hideFocusHint();
});
$('focusBadgeClose').addEventListener('click', e => {
  e.stopPropagation(); exitFocus();
});

// Drag directly on the canvas: the ghost car chases the pointer. The arc
// from the current move's start pose through the pointer's world position
// determines both steering angle and signed distance, so dragging feels
// like placing the car where you want it to go.
// A tap (minimal movement) on a move badge selects it for tweaking.
function pointerToWorld(e) {
  const r = cv.getBoundingClientRect();
  const W = cv.clientWidth, H = cv.clientHeight;
  const { wX, wY, ang, sc } = viewParams();
  const sx = e.clientX - r.left - W / 2;
  const sy = e.clientY - r.top  - H / 2;
  const c = Math.cos(-ang), s = Math.sin(-ang);
  return { x: wX + (sx * c - sy * s) / sc,
           y: wY + (sx * s + sy * c) / sc };
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
  // Un-rotate canvas delta into world space (handles focus-mode rotation).
  const { ang, sc } = viewParams();
  const c = Math.cos(-ang), s = Math.sin(-ang);
  const wdx = (dx * c - dy * s) / sc;
  const wdy = (dx * s + dy * c) / sc;
  const a = arcToPoint(editStartPose(), { x: drag.tx + wdx, y: drag.ty + wdy });
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
      if (now - lastTap < 350) {
        if (Math.abs(editDist) > 0.01) {
          // Meaningful move pending → commit it. Focus (if active) glides to
          // the new plan end on its own via trackFocus().
          if (commitMove()) toast('Move added');
        } else {
          // No pending move → toggle focus mode.
          view.focused ? exitFocus() : enterFocus();
        }
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
  solutionUsed = true;
  _solHash = null;
  recomputePlan();
  toast('Shared solution loaded');
}

// Apply moves from new-format hash (#<level_id>/<moves>)
if (_gameHash && _gameHashIdx >= 0 && level) {
  if (_gameHash.moves.length) {
    moves = quantizeMoves(_gameHash.moves);
    if (!_gameHashIsOwn) solutionUsed = true;
    recomputePlan();
  }
  _gameHash = null;
}

requestAnimationFrame(draw);
