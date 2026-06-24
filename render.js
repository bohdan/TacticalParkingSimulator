'use strict';
/*
 * render.js — the Renderer component (Component 3).
 *
 * Pure drawing: every function takes `ctx` + explicit args and reads NO physics/game globals.
 * Pose / VehicleSpec / Polygon arguments are opaque physics handles, read ONLY through the
 * `Physics` accessors (poseX/Y/Heading, spec*, polygonVertex). `steeringRadians` is the
 * explicit "turn direction" render slot.
 *
 * Kinematics stays in the kernel: the arc/steering overlays take the caller's
 * advancePose / arcCenter / turnRadius rather than re-deriving motion here. The one
 * remaining simplification is drawCarBody's per-vehicle art (bus/miata/tractor detail,
 * hubs, treads), which draws a plain body box + wheels until that styling is ported.
 *
 * Depends on the `Physics` namespace (physics-kernel.js).
 */
const Renderer = (function (P) {

  // drawPolygon(ctx, polygon) — stroke/fill a wall, border, or goal outline.
  function drawPolygon(ctx, polygon) {
    const n = P.polygonCount(polygon);
    if (!n) return;
    ctx.beginPath();
    const v0 = P.polygonVertex(polygon, 0);
    ctx.moveTo(P.pointX(v0), P.pointY(v0));
    for (let i = 1; i < n; i++) {
      const v = P.polygonVertex(polygon, i);
      ctx.lineTo(P.pointX(v), P.pointY(v));
    }
    ctx.closePath();
  }

  // drawPath(ctx, points, worldPerPixel, color) — the rear-axle trajectory polyline.
  function drawPath(ctx, points, worldPerPixel, color) {
    if (points.length < 2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * worldPerPixel;
    ctx.beginPath();
    ctx.moveTo(P.poseX(points[0]), P.poseY(points[0]));
    for (let i = 1; i < points.length; i++)
      ctx.lineTo(P.poseX(points[i]), P.poseY(points[i]));
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

  // Local-frame footprint corners from the opaque spec (render legitimately needs dims).
  function footprintLocal(spec) {
    const x0 = -P.specRearOverhang(spec), x1 = P.specWheelbase(spec) + P.specFrontOverhang(spec);
    const wy = P.specWidth(spec) / 2;
    return { x0, x1, y0: -wy, y1: wy };
  }

  // Minimal car footprint + steered wheels. The per-vehicle styling (bus/miata/tractor
  // detail, hubs, treads) is the deferred port; this draws the body box and 4 wheels with
  // the front pair rotated by the turn-direction slot.
  function drawCarBody(ctx, car, worldPerPixel) {
    const { pose, spec, steeringRadians, fill = '#5b7', stroke = '#284', wheels = true } = car;
    const f = footprintLocal(spec);
    ctx.save();
    ctx.translate(P.poseX(pose), P.poseY(pose));
    ctx.rotate(P.poseHeading(pose));
    // body
    ctx.fillStyle = fill;
    ctx.fillRect(f.x0, f.y0, f.x1 - f.x0, f.y1 - f.y0);
    if (stroke) { ctx.lineWidth = 0.07; ctx.strokeStyle = stroke; ctx.strokeRect(f.x0, f.y0, f.x1 - f.x0, f.y1 - f.y0); }
    if (wheels) {
      const wb = P.specWheelbase(spec), wy = f.y1 - 0.16;
      const wl = Math.min(0.9, (f.x1 - f.x0) * 0.075), wt = 0.16;
      ctx.fillStyle = '#10131a';
      for (const [wx, a] of [[0, 0], [wb, steeringRadians || 0]])
        for (const side of [-wy, wy]) {
          ctx.save(); ctx.translate(wx, side); ctx.rotate(a);
          ctx.fillRect(-wl / 2, -wt, wl, wt * 2);
          ctx.restore();
        }
    }
    ctx.restore();
    void worldPerPixel; // reserved for cosmetic line widths in the full port
  }

  // drawGhost(ctx, pose, spec, color, steeringRadians?) — dashed footprint outline.
  function drawGhost(ctx, pose, spec, color, steeringRadians = 0) {
    const f = footprintLocal(spec);
    ctx.save();
    ctx.translate(P.poseX(pose), P.poseY(pose));
    ctx.rotate(P.poseHeading(pose));
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
    const wb = P.specWheelbase(spec), wy = P.specWidth(spec) / 2 - 0.16;
    if (!_localFrame) { ctx.save(); ctx.translate(P.poseX(pose), P.poseY(pose)); ctx.rotate(P.poseHeading(pose)); }
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
    ctx.moveTo(P.pointX(pts[0]), P.pointY(pts[0]));
    for (let i = 1; i < pts.length; i++) ctx.lineTo(P.pointX(pts[i]), P.pointY(pts[i]));
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
    const px = P.poseX(pose), py = P.poseY(pose), h = P.poseHeading(pose);
    const ux = -Math.sin(h), uy = Math.cos(h);             // rear-axle axis, toward the centre
    const O = { x: P.pointX(turnCenter), y: P.pointY(turnCenter) };
    const sgn = Math.sign(R);
    const at = t => ({ x: px + t * ux, y: py + t * uy });
    const half = P.specWidth(spec) / 2 - 0.16;             // matches drawn wheel inset
    strokeGuide(ctx, [at(-sgn * 1.0), at(R + sgn * 1.0)], 'rgba(255,255,255,0.28)', false, 0.035);
    strokeGuide(ctx, [{ x: px, y: py }, O], 'rgba(120,220,255,0.8)', false, 0.045);
    drawSteeringWheels(ctx, pose, spec, steeringRadians, worldPerPixel);
    const fp = previewPose || pose;
    const fc = Math.cos(P.poseHeading(fp)), fs = Math.sin(P.poseHeading(fp));
    const wb = P.specWheelbase(spec), fpx = P.poseX(fp), fpy = P.poseY(fp);
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
    const fLen = P.specWheelbase(spec) + P.specFrontOverhang(spec);
    const rOver = P.specRearOverhang(spec), wb = P.specWheelbase(spec), half = P.specWidth(spec) / 2;
    const sample = (limit, dir) => {
      const t = { cFL: [], cFR: [], cRL: [], cRR: [], wFL: [], wFR: [], wRL: [], wRR: [] };
      for (let i = 0; i <= N; i++) {
        const p = advancePose(pose, steeringRadians, dir * limit * i / N);
        const cs = Math.cos(P.poseHeading(p)), sn = Math.sin(P.poseHeading(p));
        const px = P.poseX(p), py = P.poseY(p);
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
})(typeof Physics !== 'undefined' ? Physics : require('./physics-kernel.js'));

if (typeof module !== 'undefined' && module.exports) module.exports = Renderer;
