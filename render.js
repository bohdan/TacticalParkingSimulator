'use strict';
/*
 * render.js — the Renderer component (Component 3).
 *
 * Pure drawing: every function takes `ctx` + explicit args and reads NO physics/game globals.
 * Pose / VehicleSpec / Polygon arguments are opaque physics handles, read ONLY through the
 * `Physics` accessors (poseX/Y/Heading, spec*, polygonVertex). `steeringRadians` is the
 * explicit "turn direction" render slot.
 *
 * The geometrically-simple drawers are implemented. The elaborate per-vehicle art
 * (drawCarBody detail, arc guides, steering geometry) is the larger extraction from game.js
 * and is left as documented stubs — this file establishes the INTERFACE.
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

  // drawSteeringGeometry(ctx, pose, spec, steeringRadians, isPreview, worldPerPixel) —
  // turn centre + radius circle. STUB: needs the turn centre/radius from kernel.arcCenter /
  // kernel.moveTurnRadius passed in by the caller; wire in during the game.js rewire step.
  function drawSteeringGeometry(/* ctx, pose, spec, steeringRadians, isPreview, worldPerPixel */) {
    /* TODO(port): draw arcCenter dot + radius circle + steered wheels (game.js:862). */
  }

  // drawArcGuides(ctx, pose, spec, steeringRadians, forwardLimit, backwardLimit, worldPerPixel)
  // STUB: the swept corner/wheel tracks. Limits are precomputed by the caller via the kernel.
  function drawArcGuides(/* ctx, pose, spec, steeringRadians, fwd, bwd, worldPerPixel */) {
    /* TODO(port): sweep corner/wheel tracks over [backwardLimit, forwardLimit] (game.js:815). */
  }

  return {
    drawPolygon, drawPath, drawArrow,
    drawCarBody, drawGhost,
    drawSteeringWheels, drawSteeringGeometry, drawArcGuides,
  };
})(typeof Physics !== 'undefined' ? Physics : require('./physics-kernel.js'));

if (typeof module !== 'undefined' && module.exports) module.exports = Renderer;
