'use strict';
/*
 * render.js — the Renderer component (Component 3).
 *
 * Pure drawing: every function takes `ctx` + explicit args and reads NO physics/game globals.
 * Pose / Point / VehicleSpec / Polygon arguments are plain readable values — fields are read
 * directly (pose.x/.y/.h, spec.wb/.wid/…, polygon[i].x). `steeringRadians` is the explicit
 * "turn direction" render slot.
 *
 * Kinematics stays in the kernel: the arc/steering overlays take the caller's
 * advancePose / arcCenter / turnRadius rather than re-deriving motion here. drawCarBody
 * carries the full per-vehicle art (bus/miata/tractor detail, hubs, treads).
 *
 * Depends on the `Physics` namespace (physics-kernel.js) only for the rad() helper.
 */
import { Physics } from './physics-kernel.js';

export const Renderer = (function (P) {

  // drawPolygon(ctx, polygon) — stroke/fill a wall, border, or goal outline.
  function drawPolygon(ctx, polygon) {
    const n = polygon.length;
    if (!n) return;
    ctx.beginPath();
    ctx.moveTo(polygon[0].x, polygon[0].y);
    for (let i = 1; i < n; i++) ctx.lineTo(polygon[i].x, polygon[i].y);
    ctx.closePath();
  }

  // drawPath(ctx, points, worldPerPixel, color) — the rear-axle trajectory polyline.
  function drawPath(ctx, points, worldPerPixel, color) {
    if (points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * worldPerPixel;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
    ctx.stroke();
    ctx.restore();
  }

  // drawArrow(ctx, x, y, heading, size, color) — forward/reverse direction indicator.
  function drawArrow(ctx, x, y, heading, size, color) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.6, size * 0.6);
    ctx.lineTo(-size * 0.6, -size * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // Local-frame footprint corners from the spec (render legitimately needs the dims).
  function footprintLocal(spec) {
    const x0 = -spec.rOver, x1 = spec.wb + spec.fOver;
    const wy = spec.wid / 2;
    return { x0, x1, y0: -wy, y1: wy };
  }

  // Rounded-rect path helper (caller fills/strokes).
  function roundRect(ctx, x, y, w, h, r) {
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
  function drawCarBody(ctx, pose, opts, spec) {
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
      const tFill = opts.fill || '#d46020', tStroke = opts.fill ? opts.stroke : '#7a3500';
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
  function drawTractorDetail(ctx, x0, len, w) {
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

  function drawSedanDetail(ctx, x0, len, w) {
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
  function drawConvertibleDetail(ctx, x0, len, w) {
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
  function drawBusDetail(ctx, x0, len, w) {
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

  // drawGhost(ctx, pose, spec, color, steeringRadians?) — dashed footprint outline.
  function drawGhost(ctx, pose, spec, color, steeringRadians = 0) {
    const f = footprintLocal(spec);
    ctx.save();
    ctx.translate(pose.x, pose.y);
    ctx.rotate(pose.h);
    ctx.setLineDash([0.25, 0.18]);
    ctx.lineWidth = 0.05;
    ctx.strokeStyle = color;
    ctx.strokeRect(f.x0, f.y0, f.x1 - f.x0, f.y1 - f.y0);
    ctx.setLineDash([]);
    drawSteeringWheels(ctx, pose, spec, steeringRadians, 0, true);
    ctx.restore();
  }

  // drawSteeringWheels(ctx, pose, spec, steeringRadians, worldPerPixel[, _localFrame])
  function drawSteeringWheels(ctx, pose, spec, steeringRadians, worldPerPixel, _localFrame) {
    const wb = spec.wb, wy = spec.wid / 2 - 0.16;
    if (!_localFrame) { ctx.save(); ctx.translate(pose.x, pose.y); ctx.rotate(pose.h); }
    ctx.fillStyle = '#2bd';
    for (const side of [-wy, wy]) {
      ctx.save(); ctx.translate(wb, side); ctx.rotate(steeringRadians || 0);
      ctx.fillRect(-0.16, -0.08, 0.32, 0.16);
      ctx.restore();
    }
    if (!_localFrame) ctx.restore();
    void worldPerPixel;
  }

  // Styled world-space polyline (solid/dashed, explicit width) for the guide overlays.
  function strokeGuide(ctx, pts, color, dashed, lineWidth) {
    if (pts.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    if (dashed) ctx.setLineDash([0.35, 0.25]);
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }

  // drawSteeringGeometry(ctx, pose, spec, steeringRadians, turnCenter, turnRadius, previewPose, wpp)
  // The instantaneous turn centre + radius lines (rear axle and both front wheels → centre),
  // plus the steered front wheels. turnCenter (= kernel.arcCenter) / turnRadius (signed, =
  // kernel.turnRadius) come from the caller's kernel so the renderer does no kinematics.
  function drawSteeringGeometry(ctx, pose, spec, steeringRadians, turnCenter, turnRadius, previewPose, worldPerPixel) {
    if (!turnCenter || Math.abs(steeringRadians) < P.rad(0.5)) return;   // ~straight: centre at infinity
    const R = turnRadius;                                  // signed: wheelbase / tan(steer)
    const px = pose.x, py = pose.y, h = pose.h;
    const ux = -Math.sin(h), uy = Math.cos(h);             // rear-axle axis, toward the centre
    const O = { x: turnCenter.x, y: turnCenter.y };
    const sgn = Math.sign(R);
    const at = t => ({ x: px + t * ux, y: py + t * uy });
    const half = spec.wid / 2 - 0.16;                      // matches drawn wheel inset
    strokeGuide(ctx, [at(-sgn * 1.0), at(R + sgn * 1.0)], 'rgba(255,255,255,0.28)', false, 0.035);
    strokeGuide(ctx, [{ x: px, y: py }, O], 'rgba(120,220,255,0.8)', false, 0.045);
    drawSteeringWheels(ctx, pose, spec, steeringRadians, worldPerPixel);
    const fp = previewPose || pose;
    const fc = Math.cos(fp.h), fs = Math.sin(fp.h);
    const wb = spec.wb, fpx = fp.x, fpy = fp.y;
    const fw = ly => ({ x: fpx + wb * fc - ly * fs, y: fpy + wb * fs + ly * fc });
    strokeGuide(ctx, [fw(half), O], 'rgba(255,255,255,0.28)', false, 0.03);
    strokeGuide(ctx, [fw(-half), O], 'rgba(255,255,255,0.28)', false, 0.03);
    ctx.save();
    ctx.beginPath();
    ctx.arc(O.x, O.y, 0.12, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(120,220,255,0.9)';
    ctx.fill();
    ctx.restore();
    void worldPerPixel;
  }

  // drawArcGuides(ctx, pose, spec, steeringRadians, forwardLimit, backwardLimit, advancePose, wpp)
  // Swept tracks of the 4 footprint corners (solid) and 4 wheels (dashed) over the drivable
  // arc. The caller supplies its kernel's advancePose so the rollout uses the one kinematics
  // implementation; forward/backward limits are precomputed by the caller via the kernel.
  function drawArcGuides(ctx, pose, spec, steeringRadians, forwardLimit, backwardLimit, advancePose, worldPerPixel) {
    const N = 60;
    const fLen = spec.wb + spec.fOver;
    const rOver = spec.rOver, wb = spec.wb, half = spec.wid / 2;
    const sample = (limit, dir) => {
      const t = { cFL: [], cFR: [], cRL: [], cRR: [], wFL: [], wFR: [], wRL: [], wRR: [] };
      for (let i = 0; i <= N; i++) {
        const p = advancePose(pose, steeringRadians, dir * limit * i / N);
        const cs = Math.cos(p.h), sn = Math.sin(p.h);
        const px = p.x, py = p.y;
        const w = (lx, ly) => ({ x: px + cs * lx - sn * ly, y: py + sn * lx + cs * ly });
        t.cFL.push(w(fLen, half)); t.cFR.push(w(fLen, -half));
        t.cRL.push(w(-rOver, half)); t.cRR.push(w(-rOver, -half));
        t.wFL.push(w(wb, half)); t.wFR.push(w(wb, -half));
        t.wRL.push(w(0, half)); t.wRR.push(w(0, -half));
      }
      return t;
    };
    for (const [limit, dir] of [[forwardLimit, 1], [backwardLimit, -1]]) {
      if (!(limit > 0.2)) continue;
      const t = sample(limit, dir);
      const col  = dir > 0 ? 'rgba(69,196,255,0.28)' : 'rgba(255,159,67,0.28)';
      const wCol = dir > 0 ? 'rgba(69,196,255,0.50)' : 'rgba(255,159,67,0.50)';
      for (const k of ['cFL', 'cFR', 'cRL', 'cRR']) strokeGuide(ctx, t[k], col, false, 0.06);
      for (const k of ['wFL', 'wFR', 'wRL', 'wRR']) strokeGuide(ctx, t[k], wCol, true, 0.05);
    }
    void worldPerPixel;
  }

  return {
    drawPolygon, drawPath, drawArrow,
    drawCarBody, drawGhost,
    drawSteeringWheels, drawSteeringGeometry, drawArcGuides,
  };
})(Physics);
