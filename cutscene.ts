/*
 * cutscene.ts — dashboard briefing animation for cutscene levels.
 *
 * Exports showCutscene(def, onEnd) which plays the noir intro sequence and calls
 * onEnd when the player clicks Skip or Begin. All DOM references are local.
 */
import type { CutsceneLevelDef } from './levels.js';

const $ = (id: string): HTMLElement => document.getElementById(id)!;

const DEFAULT_CUTSCENE_MSG = [
  '> MSG INCOMING', '', '  AGENT 7', '  VALET', '',
  '  PKG BY 0300.', '  NO SCRATCHES.', '', '> MISSION: GO.',
];

let cutsceneMessage = DEFAULT_CUTSCENE_MSG;
let introAnimId: number | null = null;
let pendingOnEnd: (() => void) | null = null;

function endCutscene(): void {
  if (introAnimId !== null) cancelAnimationFrame(introAnimId);
  introAnimId = null;
  $('introGo').classList.add('hidden');
  const cb = pendingOnEnd;
  pendingOnEnd = null;
  cb?.();
}

$('introSkip').addEventListener('click', endCutscene);
$('introGo').addEventListener('click', endCutscene);

export function showCutscene(def: CutsceneLevelDef, onEnd: () => void): void {
  pendingOnEnd = onEnd;
  cutsceneMessage = (def.message && def.message.length) ? def.message : DEFAULT_CUTSCENE_MSG;
  playIntroDash();
}

function playIntroDash(): void {
  if (introAnimId !== null) cancelAnimationFrame(introAnimId);
  $('intro').classList.remove('hidden');
  $('introGo').classList.add('hidden');

  const canvas = $('introCanvas') as HTMLCanvasElement;
  const c = canvas.getContext('2d')!;

  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const W = window.innerWidth;
  const H = window.innerHeight;
  canvas.width  = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  c.scale(dpr, dpr);

  const portrait = H > W * 1.1;

  const SKY_H = H * 0.37;
  const DASH_Y = SKY_H;
  const DASH_H = H * 0.46;

  const SC_W = portrait ? W * 0.84 : Math.min(W * 0.52, H * 0.72);
  const SC_X = (W - SC_W) / 2;
  const SC_Y = DASH_Y + DASH_H * 0.09;
  const SC_H = DASH_H * 0.84;
  const SC_R = 8;

  const MSG = cutsceneMessage;
  const SC_PAD = Math.max(8, SC_W * 0.05);
  const LINE_SP = 1.5;
  const longest = MSG.reduce((m, l) => Math.max(m, l.length), 1);
  const fsByHeight = (SC_H - 2 * SC_PAD) / (MSG.length * LINE_SP);
  const fsByWidth  = (SC_W - 2 * SC_PAD) / (longest * 0.6);
  const FS = Math.max(7, Math.min(18, Math.floor(Math.min(fsByHeight, fsByWidth))));
  const LH = Math.ceil(FS * LINE_SP);

  const showSide = !portrait;
  const SP_R  = showSide ? Math.min((SC_X - 12) * 0.82, DASH_H * 0.38) : 0;
  const SP_CX = SC_X / 2;
  const SP_CY = DASH_Y + DASH_H * 0.52;
  const RC_X  = Math.ceil(SC_X + SC_W + 8);
  const RC_W  = W - RC_X - 8;

  const MIR_W = W * 0.28, MIR_H = Math.max(14, SKY_H * 0.18);
  const MIR_X = (W - MIR_W) / 2;

  const LED_XS = [-2,-1,0,1,2].map(i => W / 2 + i * W * 0.04);
  const LED_Y  = DASH_Y + 8;
  const LED_W  = Math.max(10, W * 0.024), LED_H = Math.max(5, H * 0.012);

  const rnd = (n: number) => { const x = Math.sin(n) * 1e4; return x - Math.floor(x); };

  const RAIN = Array.from({ length: Math.floor(W * 0.16) }, (_, i) => ({
    x: rnd(i * 7.3) * W, y0: rnd(i * 3.1) * SKY_H,
    spd: 55 + rnd(i * 5.7) * 90, len: 7 + rnd(i * 2.2) * 10,
  }));

  const BLDGS = [
    [0.00,0.70,0.056],[0.04,0.53,0.038],[0.08,0.76,0.044],[0.12,0.65,0.033],
    [0.15,0.73,0.046],[0.19,0.56,0.036],[0.23,0.80,0.038],
    [0.63,0.73,0.046],[0.67,0.56,0.038],[0.71,0.74,0.046],[0.75,0.52,0.036],
    [0.79,0.70,0.053],[0.83,0.63,0.040],[0.88,0.77,0.050],[0.93,0.68,0.07],
  ].map(([xf, yf, wf]) => {
    const bx = xf * W, by = yf * SKY_H;
    const bw = Math.max(10, wf * W), bh = (1 - yf) * SKY_H;
    const cols = Math.max(1, Math.round(bw / 12)), rows = Math.max(1, Math.round(bh / 10));
    const wins: { x: number; y: number; w: number; h: number; col: string }[] = [];
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

  function rrect(x: number, y: number, w: number, h: number, r: number): void {
    c.beginPath();
    c.moveTo(x + r, y); c.lineTo(x + w - r, y);
    c.arcTo(x + w, y, x + w, y + r, r); c.lineTo(x + w, y + h - r);
    c.arcTo(x + w, y + h, x + w - r, y + h, r); c.lineTo(x + r, y + h);
    c.arcTo(x, y + h, x, y + h - r, r); c.lineTo(x, y + r);
    c.arcTo(x, y, x + r, y, r); c.closePath();
  }

  function frame(now: number): void {
    const ms = now - t0, t = ms / 1000;

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

    if (ms > 400) {
      c.strokeStyle = 'rgba(100,150,220,0.45)'; c.lineWidth = 0.8;
      for (const dr of RAIN) {
        const y = (dr.y0 + dr.spd * t) % SKY_H;
        c.beginPath(); c.moveTo(dr.x, y); c.lineTo(dr.x - 1, y + dr.len); c.stroke();
      }
    }

    const dashGrd = c.createLinearGradient(0, DASH_Y, 0, DASH_Y + DASH_H);
    dashGrd.addColorStop(0, '#181c27'); dashGrd.addColorStop(1, '#0e1118');
    c.fillStyle = dashGrd; c.fillRect(0, DASH_Y, W, H - DASH_Y);
    c.strokeStyle = '#252c3c'; c.lineWidth = 1;
    c.beginPath(); c.moveTo(0, DASH_Y); c.lineTo(W, DASH_Y); c.stroke();

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
      c.fillStyle = 'rgba(0,0,0,0.18)';
      for (let sy = SC_Y; sy < SC_Y + SC_H; sy += 3) c.fillRect(SC_X, sy, SC_W, 1);
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
      c.shadowColor = 'rgba(0,200,60,0.4)'; c.shadowBlur = 20;
      c.strokeStyle = 'rgba(0,150,50,0.15)'; c.lineWidth = 2;
      rrect(SC_X, SC_Y, SC_W, SC_H, SC_R); c.stroke();
      c.shadowBlur = 0;
    }

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
