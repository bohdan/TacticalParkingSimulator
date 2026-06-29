/*
 * leaderboard.ts — Supabase leaderboard client.
 *
 * A small, self-contained REST client for the `leaderboard` table. It knows nothing
 * about levels, poses, or the game UI: callers pass plain values and get plain rows
 * back. The game maps its own state (level index, current level) onto this API.
 *
 *   init({ url, key })            configure (defaults to the public project below)
 *   isEnabled()                   → bool, true once configured
 *   submitToLeaderboard(entry)    POST a score
 *   loadLevelLeaderboard(levelId) → rows[]  (one level, best first)
 *   loadOverallLeaderboard()      → rows[]  (all levels, best first)
 *
 * Schema setup — run once in the Supabase SQL editor:
 *
 *   create table leaderboard (
 *     id bigserial primary key,          -- row id (auto), unrelated to levels
 *     player text not null,
 *     level int, level_id text,          -- level_id = stable per-level key
 *     level_name text,
 *     moves int, dist real,              -- score: moves, then dist as tiebreak
 *     time_s real,                        -- retired metric (kept for compat; sent as 0)
 *     solution text,                     -- encoded moves for replay
 *     submitted_at timestamptz default now()
 *   );
 *   alter table leaderboard enable row level security;
 *   create policy "public read"   on leaderboard for select using (true);
 *   create policy "public insert" on leaderboard for insert
 *     with check (char_length(player) between 1 and 20);
 *
 * If upgrading an existing table:
 *   alter table leaderboard add column if not exists solution text;
 *   alter table leaderboard add column if not exists level_id text;
 *   alter table leaderboard alter column stars drop not null; -- no longer sent
 *   alter table leaderboard alter column mode  drop not null; -- deprecated
 */

export interface LeaderboardEntry {
  player: string;
  level: number;
  levelId: string;
  levelName: string;
  moves: number;
  dist: number;
  solution?: string;
}

export interface LeaderboardRow {
  player: string;
  moves: number;
  dist: number;
  solution?: string;
  submitted_at: string;
  level_id?: string;
  level_name?: string;
}

const DEFAULTS = {
  url: 'https://qvjorkpzlwvswsptkwyn.supabase.co',
  key: 'sb_publishable_geHaaCkSfPilYWV3fYQHQA_KZdYNrpC',
};

let _url: string = '';
let _key: string = '';

// init({ url, key }?) — configure the client. Both fields default to the public project
// above, so init() with no args is the common case. Returns isEnabled().
export function init(config: { url?: string; key?: string } = {}): boolean {
  _url = config.url ?? DEFAULTS.url;
  _key = config.key ?? DEFAULTS.key;
  return isEnabled();
}

// isEnabled() — true once a URL + key are set (i.e. after init()).
export function isEnabled(): boolean {
  return !!(_url && _key);
}

function _headers(extra?: Record<string, string>): Record<string, string> {
  return { apikey: _key, Authorization: `Bearer ${_key}`, ...extra };
}

async function _getRows(params: URLSearchParams): Promise<LeaderboardRow[]> {
  const r = await fetch(`${_url}/rest/v1/leaderboard?${params}`, { headers: _headers() });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

// submitToLeaderboard(entry) — POST a single score.
// entry: { player, level, levelId, levelName, moves, dist, solution? }.
// Retries without the newer columns (solution, level_id) if the table predates them.
export async function submitToLeaderboard(entry: LeaderboardEntry): Promise<void> {
  const body = {
    player:       entry.player,
    level:        entry.level,
    level_id:     entry.levelId,
    level_name:   entry.levelName,
    moves:        entry.moves,
    dist:         +Number(entry.dist).toFixed(2),
    time_s:       0, // retired metric; column kept so inserts work on the existing schema
    solution:     entry.solution || null,
    submitted_at: new Date().toISOString(),
  };
  const post = () => fetch(`${_url}/rest/v1/leaderboard`, {
    method: 'POST',
    headers: _headers({ 'Content-Type': 'application/json', Prefer: 'return=minimal' }),
    body: JSON.stringify(body),
  });
  let r = await post();
  if (!r.ok && r.status === 400) {
    delete body.solution;
    delete body.level_id;
    r = await post();
  }
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
}

// loadLevelLeaderboard(levelId) — top entries for one level (best first).
export async function loadLevelLeaderboard(levelId: string): Promise<LeaderboardRow[]> {
  return _getRows(new URLSearchParams({
    select: 'player,moves,dist,solution,submitted_at',
    level_id: `eq.${levelId}`,
    order: 'moves.asc,dist.asc,submitted_at.asc', limit: '100',
  }));
}

// loadOverallLeaderboard() — best entries across every level (best first).
export async function loadOverallLeaderboard(): Promise<LeaderboardRow[]> {
  return _getRows(new URLSearchParams({
    select: 'player,level_id,level_name,moves,dist,solution,submitted_at',
    order: 'moves.asc,dist.asc,submitted_at.asc', limit: '500',
  }));
}
