/*
 * render.ts — the Renderer component (Component 3).
 *
 * Pure drawing: every function takes `ctx` + explicit args and reads NO physics/game globals.
 * Pose / Point / VehicleSpec / Polygon arguments are plain readable values — fields are read
 * directly (pose.x/.y/.h, spec.wb/.wid/…, polygon[i].x). `steeringRadians` is the explicit
 * "turn direction" render slot.
 *
 * Kinematics stays in the kernel: drawArcGuides takes the caller's advancePose to roll the
 * swept arc, and drive limits are passed in precomputed. (The steering overlay derives its
 * turn centre from the wheelbase — plain drawing trig, not a simulation.) drawCarBody
 * carries the full per-vehicle art (bus/miata/tractor detail, hubs, treads).
 *
 * Depends on the `Physics` namespace (physics-kernel.ts) only for the rad() helper.
 */
import { Physics } from './physics-kernel.js';
import type { Pose, VehicleSpec } from './physics-kernel.js';
import type { Point } from './geometry2d.js';

interface CarOpts {
  vehicle?: string;
  wheels?: boolean;
  steer?: number;
  fill?: string;
  stroke?: string | false;
  detail?: boolean;
}

// drawPolygon(ctx, polygon) — stroke/fill a wall, border, or goal outline.
export function drawPolygon(ctx: CanvasRenderingContext2D, polygon: Point[]): void {
  const n = polygon.length;
  if (!n) return;
  ctx.beginPath();
  ctx.moveTo(polygon[0].x, polygon[0].y);
  for (let i = 1; i < n; i++) ctx.lineTo(polygon[i].x, polygon[i].y);
  ctx.closePath();
}

// drawPath(ctx, pts, color, dashed, lw, worldPerPixel) — styled world-space polyline.
// Line width is capped at 3·worldPerPixel so it never gets thinner than a few pixels.
export function drawPath(ctx: CanvasRenderingContext2D, pts: Point[], color: string, dashed?: boolean, lw = 0.09, worldPerPixel = Infinity): void {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.lineWidth = Math.min(lw, 3 * worldPerPixel);
  ctx.strokeStyle = color;
  if (dashed) ctx.setLineDash([0.35, 0.25]);
  ctx.stroke();
  ctx.setLineDash([]);
}

// drawArrow(ctx, x, y, ang, len, color) — a line with an arrowhead (direction indicator).
export function drawArrow(ctx: CanvasRenderingContext2D, x: number, y: number, ang: number, len: number, color: string): void {
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

// Local-frame footprint corners from the spec (render legitimately needs the dims).
function footprintLocal(spec: VehicleSpec) {
  const x0 = -spec.rOver, x1 = spec.wb + spec.fOver;
  const wy = spec.wid / 2;
  return { x0, x1, y0: -wy, y1: wy };
}

// Rounded-rect path helper (caller fills/strokes).
function roundRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number): void {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

// drawCarBody(ctx, pose, opts, spec) — full per-vehicle car art, drawn in the car's
// local frame (the caller sets the world/camera transform). opts = { vehicle, wheels,
// steer, fill, stroke, detail }. spec supplies the dimensions.
export function drawCarBody(ctx: CanvasRenderingContext2D, pose: Pose, opts: CarOpts, spec: VehicleSpec): void {
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
    // front axle beam (rotates with steer) — thin pipe
    ctx.save(); ctx.translate(spec.wb, 0); ctx.rotate(opts.steer || 0);
    ctx.strokeStyle = '#3a4050'; ctx.lineWidth = 0.04;
    ctx.beginPath(); ctx.moveTo(0, -fCy); ctx.lineTo(0, fCy); ctx.stroke();
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
    const tFill = opts.fill || '#d46020', tStroke = opts.fill ? (opts.stroke || '#7a3500') : '#7a3500';
    ctx.fillStyle = tFill; ctx.lineWidth = 0.07; ctx.strokeStyle = tStroke;
    roundRect(ctx, x0, -cabW / 2, jx - x0, cabW, 0.12); ctx.fill(); if (tStroke) ctx.stroke();
    roundRect(ctx, jx, -hoodW / 2, x0 + len - jx, hoodW, 0.10); ctx.fill(); if (tStroke) ctx.stroke();
  } else {
    const fill = vtype === 'miata' ? '#d23b3b' : opts.fill;
    const corner = vtype === 'bus' ? Math.min(0.18, w * 0.08) : Math.min(0.3, w * 0.17);
    roundRect(ctx, x0, -w / 2, len, w, corner);
    ctx.fillStyle = fill; ctx.fill();
    if (opts.stroke) {
      ctx.lineWidth = 0.07;
      ctx.strokeStyle = vtype === 'miata' ? '#7d1f1f' : opts.stroke;
      ctx.stroke();
    }
  }

  if (opts.detail) {
    if (vtype === 'bus')        drawBusDetail(ctx, x0, len, w);
    else if (vtype === 'miata') drawConvertibleDetail(ctx, x0, len, w);
    else if (vtype === 'tractor') drawTractorDetail(ctx, x0, len, w);
    else                        drawSedanDetail(ctx, x0, len, w);
  }
  ctx.restore();
}

// Top-down Lamborghini R480: open cab with ROPS arch, large rear wheels flanking.
function drawTractorDetail(ctx: CanvasRenderingContext2D, x0: number, len: number, w: number): void {
  const front = x0 + len;
  const jx = x0 + len * 0.54;
  const cabW = w * 0.80, hoodW = w * 0.44;
  // ROPS arch — thick bar across cab just behind the hood junction
  ctx.strokeStyle = '#9aab8a'; ctx.lineWidth = 0.13; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(jx - 0.12, -cabW * 0.44); ctx.lineTo(jx - 0.12, cabW * 0.44); ctx.stroke();
  // Operator platform floor
  ctx.fillStyle = 'rgba(10,8,5,0.58)';
  roundRect(ctx, x0 + len * 0.06, -cabW / 2 + 0.18, len * 0.44, cabW - 0.36, 0.10); ctx.fill();
  // Seat
  ctx.fillStyle = '#1a1210';
  roundRect(ctx, x0 + len * 0.10, -0.20, len * 0.14, 0.40, 0.07); ctx.fill();
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

function drawSedanDetail(ctx: CanvasRenderingContext2D, x0: number, len: number, w: number): void {
  const wsX = x0 + len * 0.30, rwX = x0 + len * 0.09, glH = w - 0.44;
  ctx.fillStyle = 'rgba(8,12,18,0.45)';
  roundRect(ctx, wsX, -w / 2 + 0.22, Math.min(0.85, len * 0.20), glH, 0.15); ctx.fill();
  roundRect(ctx, rwX, -w / 2 + 0.25, Math.min(0.6, len * 0.13), glH, 0.15); ctx.fill();
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(x0 + len - 0.18, -w / 2 + 0.15, 0.12, Math.min(0.3, w * 0.17));
  ctx.fillRect(x0 + len - 0.18,  w / 2 - 0.45, 0.12, Math.min(0.3, w * 0.17));
}

// Top-down convertible: open cockpit (no roof), small raked windshield,
// two seats and a roll hoop behind them.
function drawConvertibleDetail(ctx: CanvasRenderingContext2D, x0: number, len: number, w: number): void {
  const cockpitX = x0 + len * 0.16, cockpitLen = len * 0.46;
  // open interior tub
  ctx.fillStyle = '#2a1010';
  roundRect(ctx, cockpitX, -w / 2 + 0.20, cockpitLen, w - 0.40, 0.12); ctx.fill();
  // two seats
  ctx.fillStyle = '#3a2424';
  const seatW = cockpitLen * 0.42, seatH = (w - 0.40) / 2 - 0.12;
  roundRect(ctx, cockpitX + cockpitLen * 0.12, -w / 2 + 0.30, seatW, seatH, 0.08); ctx.fill();
  roundRect(ctx, cockpitX + cockpitLen * 0.12,  0.06,          seatW, seatH, 0.08); ctx.fill();
  // raked windshield at the front of the cockpit
  ctx.fillStyle = 'rgba(150,200,230,0.55)';
  roundRect(ctx, cockpitX + cockpitLen - 0.04, -w / 2 + 0.24, 0.14, w - 0.48, 0.06); ctx.fill();
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
function drawBusDetail(ctx: CanvasRenderingContext2D, x0: number, len: number, w: number): void {
  const front = x0 + len;
  // wraparound windscreen
  ctx.fillStyle = 'rgba(120,170,210,0.55)';
  roundRect(ctx, front - 0.5, -w / 2 + 0.18, 0.34, w - 0.36, 0.1); ctx.fill();
  // side window strips
  const winX = x0 + len * 0.12, winLen = len * 0.66, strip = 0.26;
  ctx.fillStyle = 'rgba(120,170,210,0.45)';
  roundRect(ctx, winX, -w / 2 + 0.14, winLen, strip, 0.08); ctx.fill();
  roundRect(ctx, winX,  w / 2 - 0.14 - strip, winLen, strip, 0.08); ctx.fill();
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
  roundRect(ctx, front - 1.2, w / 2 - 0.16 - strip - 0.02, 0.5, strip + 0.04, 0.05); ctx.fill();
  // headlights
  ctx.fillStyle = '#ffe9a8';
  ctx.fillRect(front - 0.12, -w / 2 + 0.16, 0.10, 0.3);
  ctx.fillRect(front - 0.12,  w / 2 - 0.46, 0.10, 0.3);
}

// drawGhost(ctx, pose, spec, color, steer, worldPerPixel) — dashed footprint outline
// with dashed wheel boxes and a filled heading notch at the nose.
export function drawGhost(ctx: CanvasRenderingContext2D, pose: Pose, spec: VehicleSpec, color: string, steer = 0, worldPerPixel = Infinity): void {
  const wy = spec.wid / 2 - 0.13;
  ctx.save();
  ctx.translate(pose.x, pose.y);
  ctx.rotate(pose.h);
  ctx.lineWidth = Math.min(0.07, 2 * worldPerPixel);
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  for (const [wx, wya, a] of [
    [0, -wy, 0], [0, wy, 0],
    [spec.wb, -wy, steer], [spec.wb, wy, steer],
  ]) {
    ctx.save();
    ctx.translate(wx, wya);
    ctx.rotate(a);
    ctx.strokeRect(-0.33, -0.13, 0.66, 0.26);
    ctx.restore();
  }
  ctx.restore();
  ctx.setLineDash([]);

  // footprint outline (world frame) — same corners as kernel.carPolygon
  const f = footprintLocal(spec), c = Math.cos(pose.h), s = Math.sin(pose.h);
  const pt = (lx, ly) => ({ x: pose.x + c * lx - s * ly, y: pose.y + s * lx + c * ly });
  drawPolygon(ctx, [pt(f.x0, f.y0), pt(f.x1, f.y0), pt(f.x1, f.y1), pt(f.x0, f.y1)]);
  ctx.lineWidth = Math.min(0.07, 2 * worldPerPixel);
  ctx.strokeStyle = color;
  ctx.setLineDash([0.25, 0.18]);
  ctx.stroke();
  ctx.setLineDash([]);
  // heading notch at the nose
  const nx = pose.x + c * (spec.wb + spec.fOver), ny = pose.y + s * (spec.wb + spec.fOver);
  ctx.beginPath();
  ctx.moveTo(nx + c * 0.45, ny + s * 0.45);
  ctx.lineTo(nx - s * 0.3, ny + c * 0.3);
  ctx.lineTo(nx + s * 0.3, ny - c * 0.3);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
}

// drawSteeringWheels(ctx, pose, spec, steerRad, worldPerPixel) — the two front wheels at
// `pose`, rotated to `steerRad`, with a cyan outline (part of the steering overlay).
export function drawSteeringWheels(ctx: CanvasRenderingContext2D, pose: Pose, spec: VehicleSpec, steerRad: number, worldPerPixel = Infinity): void {
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const wy = spec.wid / 2 - 0.16;
  const wl = Math.min(0.9, spec.len * 0.075), wt = Math.min(0.18, spec.wid * 0.10);
  for (const ly of [wy, -wy]) {
    const wx = pose.x + spec.wb * c - ly * s;
    const wyp = pose.y + spec.wb * s + ly * c;
    ctx.save();
    ctx.translate(wx, wyp);
    ctx.rotate(pose.h + steerRad);
    ctx.fillStyle = '#10131a';
    ctx.fillRect(-wl / 2, -wt, wl, wt * 2);
    ctx.lineWidth = Math.min(0.03, 1.5 * worldPerPixel);
    ctx.strokeStyle = 'rgba(120,220,255,0.9)';
    ctx.strokeRect(-wl / 2, -wt, wl, wt * 2);
    ctx.restore();
  }
}

// drawSteeringGeometry(ctx, pose, spec, steerRad, previewPose, worldPerPixel) — the rear-axle
// axis, the instantaneous turn centre on it, and the radius lines from the rear axle and
// both front wheels to that centre (classic Ackermann), plus the steered front wheels.
export function drawSteeringGeometry(ctx: CanvasRenderingContext2D, pose: Pose, spec: VehicleSpec, steerRad: number, previewPose: Pose | null, worldPerPixel = Infinity): void {
  if (Math.abs(steerRad) < Physics.rad(0.5)) return;   // ~straight: centre at infinity
  const R = spec.wb / Math.tan(steerRad);
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  const ux = -s, uy = c;                         // rear-axle axis direction (toward centre)
  const O = { x: pose.x + R * ux, y: pose.y + R * uy };
  const sgn = Math.sign(R);
  const at = t => ({ x: pose.x + t * ux, y: pose.y + t * uy });
  const half = spec.wid / 2 - 0.16;              // matches drawn wheel inset
  drawPath(ctx, [at(-sgn * 1.0), at(R + sgn * 1.0)], 'rgba(255,255,255,0.28)', false, 0.035, worldPerPixel);
  drawPath(ctx, [{ x: pose.x, y: pose.y }, O], 'rgba(120,220,255,0.8)', false, 0.045, worldPerPixel);
  drawSteeringWheels(ctx, pose, spec, steerRad, worldPerPixel);
  const fp = previewPose || pose;
  const fc = Math.cos(fp.h), fs = Math.sin(fp.h);
  const fw = ly => ({ x: fp.x + spec.wb * fc - ly * fs, y: fp.y + spec.wb * fs + ly * fc });
  drawPath(ctx, [fw(half), O], 'rgba(255,255,255,0.28)', false, 0.03, worldPerPixel);
  drawPath(ctx, [fw(-half), O], 'rgba(255,255,255,0.28)', false, 0.03, worldPerPixel);
  ctx.beginPath();
  ctx.arc(O.x, O.y, 0.12, 0, 2 * Math.PI);
  ctx.fillStyle = 'rgba(120,220,255,0.9)';
  ctx.fill();
}

// drawArcGuides(ctx, pose, spec, steerRad, forwardLimit, backwardLimit, advancePose, wpp) —
// swept tracks of the 4 footprint corners (solid) and 4 wheels (dashed) over the drivable
// arc. The caller supplies its kernel's advancePose and the precomputed drive limits.
export function drawArcGuides(ctx: CanvasRenderingContext2D, pose: Pose, spec: VehicleSpec, steerRad: number, forwardLimit: number, backwardLimit: number, advancePose: (p: Pose, steer: number, dist: number) => Pose, worldPerPixel = Infinity): void {
  const N = 60;
  const fLen = spec.wb + spec.fOver, half = spec.wid / 2;
  const sample = (limit, dir) => {
    const cFL = [], cFR = [], cRL = [], cRR = [], wFL = [], wFR = [], wRL = [], wRR = [];
    for (let i = 0; i <= N; i++) {
      const p = advancePose(pose, steerRad, dir * limit * i / N);
      const cs = Math.cos(p.h), sn = Math.sin(p.h);
      const w = (lx, ly) => ({ x: p.x + cs * lx - sn * ly, y: p.y + sn * lx + cs * ly });
      cFL.push(w(fLen, half)); cFR.push(w(fLen, -half));
      cRL.push(w(-spec.rOver, half)); cRR.push(w(-spec.rOver, -half));
      wFL.push(w(spec.wb, half)); wFR.push(w(spec.wb, -half));
      wRL.push(w(0, half)); wRR.push(w(0, -half));
    }
    return { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR };
  };
  for (const [limit, dir] of [[forwardLimit, 1], [backwardLimit, -1]]) {
    if (limit < 0.2) continue;
    const { cFL, cFR, cRL, cRR, wFL, wFR, wRL, wRR } = sample(limit, dir);
    const col  = dir > 0 ? 'rgba(69,196,255,0.28)' : 'rgba(255,159,67,0.28)';
    const wCol = dir > 0 ? 'rgba(69,196,255,0.50)' : 'rgba(255,159,67,0.50)';
    drawPath(ctx, cFL, col, false, 0.06, worldPerPixel);
    drawPath(ctx, cFR, col, false, 0.06, worldPerPixel);
    drawPath(ctx, cRL, col, false, 0.06, worldPerPixel);
    drawPath(ctx, cRR, col, false, 0.06, worldPerPixel);
    drawPath(ctx, wFL, wCol, true, 0.05, worldPerPixel);
    drawPath(ctx, wFR, wCol, true, 0.05, worldPerPixel);
    drawPath(ctx, wRL, wCol, true, 0.05, worldPerPixel);
    drawPath(ctx, wRR, wCol, true, 0.05, worldPerPixel);
  }
}
