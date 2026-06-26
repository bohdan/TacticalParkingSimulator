'use strict';
/*
 * geometry2d.js — generic 2D geometry utilities (`Geom2D`).
 *
 * Standard, physics-agnostic geometry: polygon builders, point-in-polygon, SAT overlap,
 * convex hull, bounding circle, segment distance / intersection. Nothing here knows about
 * vehicles, steering, or the game. The physics layer (physics-kernel.js) builds on this.
 *
 * Operates on plain points {x, y} and polygons (arrays of points, convex & CCW for the SAT
 * and hull routines). Browser global + Node module.
 */
export const Geom2D = (function () {

  // Axis-aligned rectangle → 4-point polygon (CCW).
  function rectanglePolygon(x, y, w, h) {
    return [{ x, y }, { x: x + w, y }, { x: x + w, y: y + h }, { x, y: y + h }];
  }

  // Oriented box centred at (cx,cy), size w×h, rotated by `ang` → 4-point polygon.
  function orientedBoxPolygon(cx, cy, w, h, ang) {
    const c = Math.cos(ang), s = Math.sin(ang);
    const pt = (x, y) => ({ x: cx + c * x - s * y, y: cy + s * x + c * y });
    return [pt(-w / 2, -h / 2), pt(w / 2, -h / 2), pt(w / 2, h / 2), pt(-w / 2, h / 2)];
  }

  // Ray-cast point-in-polygon.
  function pointInPolygon(pt, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const a = poly[i], b = poly[j];
      if ((a.y > pt.y) !== (b.y > pt.y) &&
          pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }

  // Separating-axis test for two convex polygons → true if they overlap.
  function polygonsCollide(A, B) {
    for (const [P, Q] of [[A, B], [B, A]]) {
      for (let i = 0; i < P.length; i++) {
        const a = P[i], b = P[(i + 1) % P.length];
        const nx = b.y - a.y, ny = a.x - b.x;
        let minP = Infinity, maxP = -Infinity, minQ = Infinity, maxQ = -Infinity;
        for (const v of P) { const d = v.x * nx + v.y * ny; if (d < minP) minP = d; if (d > maxP) maxP = d; }
        for (const v of Q) { const d = v.x * nx + v.y * ny; if (d < minQ) minQ = d; if (d > maxQ) maxQ = d; }
        if (maxP < minQ || maxQ < minP) return false;
      }
    }
    return true;
  }

  // Distance from point (px,py) to segment (ax,ay)-(bx,by).
  function pointToSegmentDistance(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    if (!l2) return Math.hypot(px - ax, py - ay);
    const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  }

  // Intersection point of segments p1-p2 and p3-p4, or null if they don't cross.
  function segmentIntersection(p1, p2, p3, p4) {
    const d = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
    if (Math.abs(d) < 1e-12) return null;               // parallel / collinear
    const t = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / d;
    const u = ((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / d;
    if (t < 0 || t > 1 || u < 0 || u > 1) return null;  // cross is off one of the segments
    return { x: p1.x + t * (p2.x - p1.x), y: p1.y + t * (p2.y - p1.y) };
  }

  // Convex hull (Andrew's monotone chain), CCW.
  function convexHull(pts) {
    const p = pts.slice().sort((a, b) => a.x - b.x || a.y - b.y);
    const n = p.length;
    if (n < 3) return p;
    const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    const lo = [];
    for (const pt of p) {
      while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], pt) <= 0) lo.pop();
      lo.push(pt);
    }
    const hi = [];
    for (let i = n - 1; i >= 0; i--) {
      const pt = p[i];
      while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], pt) <= 0) hi.pop();
      hi.push(pt);
    }
    lo.pop(); hi.pop();
    return lo.concat(hi);
  }

  // Bounding circle of a polygon (centroid + farthest-vertex radius).
  function polygonBoundingCircle(poly) {
    let cx = 0, cy = 0;
    for (const v of poly) { cx += v.x; cy += v.y; }
    cx /= poly.length; cy /= poly.length;
    let r = 0;
    for (const v of poly) { const d = Math.hypot(v.x - cx, v.y - cy); if (d > r) r = d; }
    return { x: cx, y: cy, r };
  }

  // Average of a polygon's vertices.
  function centroid(poly) {
    let x = 0, y = 0;
    for (const v of poly) { x += v.x; y += v.y; }
    return { x: x / poly.length, y: y / poly.length };
  }

  // A representative contact point for two overlapping polygons (a vertex of one inside the
  // other, else the vertex of A nearest B's centroid). Used to mark where a collision is.
  function contactPoint(A, B) {
    for (const v of A) if (pointInPolygon(v, B)) return v;
    for (const v of B) if (pointInPolygon(v, A)) return v;
    const c = centroid(B);
    let best = A[0], bd = Infinity;
    for (const v of A) {
      const d = (v.x - c.x) ** 2 + (v.y - c.y) ** 2;
      if (d < bd) { bd = d; best = v; }
    }
    return best;
  }

  return {
    rectanglePolygon, orientedBoxPolygon, pointInPolygon, polygonsCollide,
    pointToSegmentDistance, segmentIntersection, convexHull, polygonBoundingCircle,
    centroid, contactPoint,
  };
})();
