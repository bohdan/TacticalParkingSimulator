'use strict';
/*
 * geometry2d.ts — generic 2D geometry utilities (`Geom2D`).
 *
 * Standard, physics-agnostic geometry: polygon builders, point-in-polygon, SAT overlap,
 * convex hull, bounding circle, segment distance / intersection. Nothing here knows about
 * vehicles, steering, or the game. The physics layer (physics-kernel.ts) builds on this.
 *
 * Operates on plain points {x, y} and polygons (arrays of points, convex & CCW for the SAT
 * and hull routines). Browser global + Node module.
 */

export interface Point {
  x: number;
  y: number;
}

export interface BoundingCircle {
  x: number;
  y: number;
  r: number;
}

export interface Shape {
  poly: Point[];
  bc: BoundingCircle;
}

export class Geom2D {
  static rectanglePolygon(x: number, y: number, w: number, h: number): Point[] {
    return [{ x, y }, { x: x + w, y }, { x: x + w, y: y + h }, { x, y: y + h }];
  }

  static orientedBoxPolygon(cx: number, cy: number, w: number, h: number, ang: number): Point[] {
    const c = Math.cos(ang), s = Math.sin(ang);
    const pt = (x: number, y: number): Point => ({ x: cx + c * x - s * y, y: cy + s * x + c * y });
    return [pt(-w / 2, -h / 2), pt(w / 2, -h / 2), pt(w / 2, h / 2), pt(-w / 2, h / 2)];
  }

  static pointInPolygon(pt: Point, poly: ReadonlyArray<Point>): boolean {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const a = poly[i], b = poly[j];
      if ((a.y > pt.y) !== (b.y > pt.y) &&
          pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }

  static polygonsCollide(A: ReadonlyArray<Point>, B: ReadonlyArray<Point>): boolean {
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

  static pointToSegmentDistance(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    if (!l2) return Math.hypot(px - ax, py - ay);
    const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  }

  static segmentIntersection(p1: Point, p2: Point, p3: Point, p4: Point): Point | null {
    const d = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
    if (Math.abs(d) < 1e-12) return null;
    const t = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / d;
    const u = ((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / d;
    if (t < 0 || t > 1 || u < 0 || u > 1) return null;
    return { x: p1.x + t * (p2.x - p1.x), y: p1.y + t * (p2.y - p1.y) };
  }

  static convexHull(pts: ReadonlyArray<Point>): Point[] {
    const p = pts.slice().sort((a, b) => a.x - b.x || a.y - b.y);
    const n = p.length;
    if (n < 3) return p;
    const cross = (o: Point, a: Point, b: Point) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    const lo: Point[] = [];
    for (const pt of p) {
      while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], pt) <= 0) lo.pop();
      lo.push(pt);
    }
    const hi: Point[] = [];
    for (let i = n - 1; i >= 0; i--) {
      const pt = p[i];
      while (hi.length >= 2 && cross(hi[hi.length - 2], hi[hi.length - 1], pt) <= 0) hi.pop();
      hi.push(pt);
    }
    lo.pop(); hi.pop();
    return lo.concat(hi);
  }

  static polygonBoundingCircle(poly: ReadonlyArray<Point>): BoundingCircle {
    let cx = 0, cy = 0;
    for (const v of poly) { cx += v.x; cy += v.y; }
    cx /= poly.length; cy /= poly.length;
    let r = 0;
    for (const v of poly) { const d = Math.hypot(v.x - cx, v.y - cy); if (d > r) r = d; }
    return { x: cx, y: cy, r };
  }

  static centroid(poly: ReadonlyArray<Point>): Point {
    let x = 0, y = 0;
    for (const v of poly) { x += v.x; y += v.y; }
    return { x: x / poly.length, y: y / poly.length };
  }

  static contactPoint(A: ReadonlyArray<Point>, B: ReadonlyArray<Point>): Point {
    for (const v of A) if (Geom2D.pointInPolygon(v, B)) return v;
    for (const v of B) if (Geom2D.pointInPolygon(v, A)) return v;
    const c = Geom2D.centroid(B);
    let best = A[0], bd = Infinity;
    for (const v of A) {
      const d = (v.x - c.x) ** 2 + (v.y - c.y) ** 2;
      if (d < bd) { bd = d; best = v; }
    }
    return best;
  }
}
