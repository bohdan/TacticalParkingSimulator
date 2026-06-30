import type { Pose, Goal } from './physics-kernel.js';

export interface RawMove { steer: number; dist: number; }

export interface WallDef {
  x?: number; y?: number;
  cx?: number; cy?: number;
  w: number; h: number;
  ang?: number; kind?: string;
}

export interface CarDef { cx: number; cy: number; h: number; type?: string; }

export interface MarkingDef { type: string; x: number; y: number; len: number; ang: number; }

export interface PlayableLevelDef {
  id: string; name: string;
  tier?: string; mode?: string;
  w: number; h: number;
  vehicle?: string;
  start: Pose;
  goal: Goal;
  walls?: WallDef[];
  cars?: CarDef[];
  markings?: MarkingDef[];
  hint?: string;
  par?: number;
  draft?: boolean;
  traffic?: Array<{ x: number; y: number; h: number; speed: number; loop: number; offset: number; color?: string }>;
  _isTest?: boolean;
  tut?: string;
  solution?: RawMove[];
  solutions?: RawMove[][];
}

export interface CutsceneLevelDef {
  id: string; type: 'cutscene'; name: string; message: string[];
}

export type LevelDef = PlayableLevelDef | CutsceneLevelDef;

import { LEVELS as _LEVELS } from './level-data.js';
export const LEVELS: LevelDef[] = _LEVELS as LevelDef[];
