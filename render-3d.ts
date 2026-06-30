// render-3d.ts — Three.js 3D visualisation, extracted from game.ts.
// THREE is loaded as a global script tag (three.min.js); declared below.
// Consumers import show3DView and pass game state via View3DParams.
import { inGoal, clamp } from './physics-compat.js';
import type { VehicleSpec, Pose, Goal } from './physics-kernel.js';

declare const THREE: any;

interface ObstacleRect {
  x?: number; y?: number;
  cx?: number; cy?: number;
  w: number; h: number;
  ang?: number;
}

interface ObstaclePose { cx: number; cy: number; h: number; type?: string; }

interface TrafficDef {
  x: number; y: number; h: number;
  speed: number; offset: number; loop: number;
  color?: string;
}

interface SceneObstacle3D {
  kind: string;
  rect?: ObstacleRect;
  pose?: ObstaclePose;
  carSpec?: VehicleSpec;
}

export interface Level3D {
  w: number; h: number;
  goal: Goal;
  start: Pose;
  vehicle?: string;
  obstacles: SceneObstacle3D[];
  traffic?: TrafficDef[];
}

export interface View3DParams {
  level: Level3D;
  sedan: VehicleSpec;
  car: VehicleSpec;
  viewScale: number;
  moves: Array<{ dist: number }>;
  planSims: Array<{ pts: Pose[] }>;
  cv: HTMLCanvasElement;
  v3d: HTMLElement;
  cv3d: HTMLCanvasElement;
  v3dClose: HTMLElement;
  planEnd: () => Pose;
  finishRun: () => void;
  toast: (msg: string) => void;
  onClose: () => void;
}

/* Build a compound car mesh matching the 2D silhouettes.
   Group origin = body centre, ground at y=0, car faces +X at rotation.y=0. */
export function buildCar3D(spec: VehicleSpec, bodyColorHex: number, vehicleType: string): any {
  const grp = new THREE.Group();
  const { len, wid, rOver, wb } = spec;
  const fOver   = spec.fOver != null ? spec.fOver : len - wb - rOver;
  const halfLen = len / 2;
  const isBus     = vehicleType === 'bus';
  const isTractor = vehicleType === 'tractor';
  const isMiata   = vehicleType === 'miata';

  const wheelR   = Math.min(0.36, wid * 0.20);
  const wheelThk = Math.min(0.24, wid * 0.14);
  const bodyH    = 0.40;
  const Y0       = wheelR;   // bottom-of-body sits on top of wheel radius

  const bodyMat  = new THREE.MeshLambertMaterial({ color: bodyColorHex });
  const glassMat = new THREE.MeshLambertMaterial({ color: 0x0e1d2c, transparent: true, opacity: 0.88 });
  const wheelMat = new THREE.MeshLambertMaterial({ color: 0x141414 });
  const rimMat   = new THREE.MeshLambertMaterial({ color: 0x8899bb });
  const hlMat    = new THREE.MeshBasicMaterial({ color: 0xffffcc });
  const tlMat    = new THREE.MeshBasicMaterial({ color: 0xdd1f1f });

  const add = (m: any) => grp.add(m);
  const box = (w: number, h: number, d: number, mat: any) => new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);

  // ── tractor — T-shaped body ───────────────────────────────────────────────
  if (isTractor) {
    const jx    = -halfLen + len * 0.54;   // cab/hood junction in local X
    const cabW  = wid * 0.82, hoodW = wid * 0.46;
    const cabH  = bodyH + len * 0.10;
    const hoodH = bodyH * 0.55;
    const cabLen  = jx + halfLen;
    const hoodLen = halfLen - jx;

    const cab  = box(cabLen,  cabH,  cabW,  bodyMat);
    cab.position.set(-halfLen + cabLen / 2, Y0 + cabH / 2, 0);  add(cab);

    const hood = box(hoodLen, hoodH, hoodW, bodyMat);
    hood.position.set(jx + hoodLen / 2, Y0 + hoodH / 2, 0);     add(hood);

    // ROPS arch (rollbar)
    const archMat = new THREE.MeshLambertMaterial({ color: 0x9aab8a });
    const arch = box(0.13, 0.14, cabW * 0.85, archMat);
    arch.position.set(jx - 0.10, Y0 + cabH + 0.07, 0);           add(arch);

    // Exhaust stack
    const exMat = new THREE.MeshLambertMaterial({ color: 0x1c1c1c });
    const exGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.38, 7);
    const ex = new THREE.Mesh(exGeo, exMat);
    ex.position.set(jx + hoodLen * 0.52, Y0 + hoodH + 0.19, hoodW * 0.38); add(ex);

    // Tractor big rear wheels + small front wheels
    const rR = wid * 0.24, rThk = 0.65;
    const fR = wid * 0.10, fThk = 0.28;
    const rAxX = -halfLen + rOver, fAxX = rAxX + wb;
    const rCy   = wid / 2 - rR;   // wheel centre in Z (inset from body edge)
    const fCy   = wid / 2 - fR;
    const rGeo = new THREE.CylinderGeometry(rR, rR, rThk, 14);
    const fGeo = new THREE.CylinderGeometry(fR, fR, fThk, 12);
    for (const side of [1, -1]) {
      for (const [geo, axX, cy, thk] of [[rGeo, rAxX, rCy, rThk], [fGeo, fAxX, fCy, fThk]]) {
        const wm = new THREE.Mesh(geo, wheelMat); wm.rotation.x = Math.PI / 2;
        wm.position.set(axX, geo === rGeo ? rR : fR, side * (cy + thk / 2)); add(wm);
        const rim = new THREE.Mesh(new THREE.CircleGeometry((geo === rGeo ? rR : fR) * 0.55, 8), rimMat);
        rim.position.set(axX, geo === rGeo ? rR : fR, side * (cy + thk / 2 + 0.01));
        if (side === -1) rim.rotation.y = Math.PI; add(rim);
      }
    }
    // headlights
    for (const z of [hoodW * 0.3, -hoodW * 0.3]) {
      const hl = box(0.07, 0.14, 0.18, hlMat); hl.position.set(halfLen - 0.04, Y0 + hoodH * 0.55, z); add(hl);
    }
    // taillights
    for (const z of [cabW * 0.35, -cabW * 0.35]) {
      const tl = box(0.07, 0.13, 0.18, tlMat); tl.position.set(-halfLen + 0.04, Y0 + cabH * 0.55, z); add(tl);
    }
    return grp;
  }

  // ── sedan / bus / miata — body slab ──────────────────────────────────────
  const body = box(len, bodyH, wid, bodyMat);
  body.position.y = Y0 + bodyH / 2; add(body);

  const cabinH  = isBus ? len * 0.145 : len * 0.112;
  const cabinRX = isBus ? -halfLen + len * 0.04 : -halfLen + len * 0.13;
  const cabinFX = isBus ?  halfLen - len * 0.04 : -halfLen + len * 0.73;
  const cabinLen = cabinFX - cabinRX;
  const cabinCX  = (cabinRX + cabinFX) / 2;
  const cabinWid = isBus ? wid * 0.96 : wid * 0.85;
  const gT = 0.045;

  if (isMiata) {
    // Convertible roadster: no roof — windshield + A-pillar rail + rollover hoop
    const wsH = cabinH * 0.62;
    const ws = box(gT * 2, wsH, wid * 0.82, glassMat);
    ws.position.set(cabinFX + gT, Y0 + bodyH + wsH * 0.55, 0); add(ws);

    const aRail = box(0.06, 0.05, wid * 0.85, bodyMat);
    aRail.position.set(cabinFX + gT, Y0 + bodyH + wsH, 0); add(aRail);

    // Twin roll-hoop posts + crossbar
    const hoopX = -halfLen + len * 0.44;
    const hoopH = cabinH * 0.72;
    for (const side of [1, -1]) {
      const post = box(0.065, hoopH, 0.07, bodyMat);
      post.position.set(hoopX, Y0 + bodyH + hoopH / 2, side * wid * 0.30); add(post);
    }
    const xbar = box(0.065, 0.065, wid * 0.62, bodyMat);
    xbar.position.set(hoopX, Y0 + bodyH + hoopH, 0); add(xbar);
  } else {
    // Sedan / bus: full cabin box + glass
    const cabin = box(cabinLen, cabinH, cabinWid, bodyMat);
    cabin.position.set(cabinCX, Y0 + bodyH + cabinH / 2, 0); add(cabin);

    const gH = cabinH * 0.65;
    const gY = Y0 + bodyH + cabinH * 0.35 + gH / 2;
    const gW = cabinWid * 0.92;

    const ws = box(gT, gH, gW, glassMat);
    ws.position.set(cabinFX + gT / 2, gY, 0); add(ws);

    const rw = box(gT, gH * 0.78, gW * 0.90, glassMat);
    rw.position.set(cabinRX - gT / 2, gY - gH * 0.10, 0); add(rw);

    if (isBus) {
      const swH = cabinH * 0.42, swLen = len * 0.062;
      const swY = Y0 + bodyH + cabinH * 0.52;
      for (const wz of [cabinWid / 2 + 0.01, -cabinWid / 2 - 0.01]) {
        for (let i = 0; i < 8; i++) {
          const sw = box(swLen, swH, gT, glassMat);
          sw.position.set(-halfLen + len * 0.12 + i * len * 0.096, swY + swH / 2, wz); add(sw);
        }
      }
    } else {
      const swLen = cabinLen * 0.68, swH = cabinH * 0.60;
      const swY = Y0 + bodyH + cabinH * 0.40 + swH / 2;
      for (const wz of [cabinWid / 2 + 0.01, -cabinWid / 2 - 0.01]) {
        const sw = box(swLen, swH, gT, glassMat);
        sw.position.set(cabinCX - cabinLen * 0.05, swY, wz); add(sw);
      }
    }
  }

  // ── wheels ────────────────────────────────────────────────────────────────
  const rearAxleX  = -halfLen + rOver;
  const frontAxleX = rearAxleX + wb;
  const axles: Array<[number, boolean]> = isBus
    ? [[rearAxleX - wb * 0.08, false], [rearAxleX + wb * 0.08, false], [frontAxleX, true]]
    : [[rearAxleX, false], [frontAxleX, true]];

  const wGeo = new THREE.CylinderGeometry(wheelR, wheelR, wheelThk, 14);
  for (const [axX] of axles) {
    for (const side of [1, -1]) {
      const wz = side * (wid / 2 - wheelThk / 2);
      const wm = new THREE.Mesh(wGeo, wheelMat); wm.rotation.x = Math.PI / 2;
      wm.position.set(axX, wheelR, wz); add(wm);
      const rim = new THREE.Mesh(new THREE.CircleGeometry(wheelR * 0.58, 8), rimMat);
      rim.position.set(axX, wheelR, wz + side * (wheelThk / 2 + 0.005));
      if (side === -1) rim.rotation.y = Math.PI; add(rim);
    }
  }

  // ── headlights + taillights ───────────────────────────────────────────────
  if (isMiata) {
    // Round headlights — Miata's signature circular lenses
    const hlGeo = new THREE.CylinderGeometry(0.11, 0.11, 0.06, 14);
    for (const z of [wid * 0.27, -wid * 0.27]) {
      const hl = new THREE.Mesh(hlGeo, hlMat);
      hl.rotation.z = Math.PI / 2;
      hl.position.set(halfLen - 0.04, Y0 + bodyH * 0.58, z); add(hl);
    }
    // Round taillights
    const tlGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.05, 12);
    for (const z of [wid * 0.28, -wid * 0.28]) {
      const tl = new THREE.Mesh(tlGeo, tlMat);
      tl.rotation.z = Math.PI / 2;
      tl.position.set(-halfLen + 0.04, Y0 + bodyH * 0.55, z); add(tl);
    }
  } else {
    for (const z of [wid * 0.28, -wid * 0.28]) {
      const hl = box(0.07, 0.14, 0.22, hlMat);
      hl.position.set(halfLen - 0.04, Y0 + bodyH * 0.55, z); add(hl);
    }
    for (const z of [wid * 0.28, -wid * 0.28]) {
      const tl = box(0.07, 0.14, 0.22, tlMat);
      tl.position.set(-halfLen + 0.04, Y0 + bodyH * 0.55, z); add(tl);
    }
  }

  // suppress unused-variable warning for fOver (computed for symmetry with game.ts)
  void fOver;

  return grp;
}

export function show3DView(params: View3DParams): void {
  const { level, sedan, car, viewScale, moves, planSims,
          cv, v3d, cv3d: cv3, v3dClose,
          planEnd, finishRun, toast, onClose } = params;

  if (!level) { onClose(); return; }
  if (typeof THREE === 'undefined') {
    toast('3D library not loaded'); onClose(); return;
  }

  // Position the 3-D overlay exactly over the 2-D game canvas so the
  // transition is seamless (same on-screen rectangle, same world scale).
  const rect = cv.getBoundingClientRect();
  v3d.style.left   = rect.left + 'px';
  v3d.style.top    = rect.top + 'px';
  v3d.style.width  = rect.width + 'px';
  v3d.style.height = rect.height + 'px';
  (v3d.style as any).right  = 'auto';
  (v3d.style as any).bottom = 'auto';
  v3d.style.background = 'transparent';   // let the live 2-D show through during crossfade
  v3d.classList.remove('hidden');

  const W = rect.width, H = rect.height;

  // ── renderer ──────────────────────────────────────────────────────────────
  const renderer = new THREE.WebGLRenderer({ canvas: cv3, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(W, H);
  renderer.setClearColor(0x171a21);
  cv3.style.opacity = '0';   // fade in over the 2-D view

  // ── scene + lights ────────────────────────────────────────────────────────
  const scene = new THREE.Scene();
  const diag = Math.hypot(level.w, level.h);
  scene.fog = new THREE.Fog(0x171a21, diag * 2.5, diag * 6.0);

  scene.add(new THREE.AmbientLight(0xcce0ff, 0.60));
  const sun = new THREE.DirectionalLight(0xfff8ee, 1.0);
  sun.position.set(level.w * 0.3, level.h * 1.2, -level.h * 0.5);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0x8899cc, 0.35);
  fill.position.set(-level.w, level.h * 0.4, level.h);
  scene.add(fill);

  // ── floor ─────────────────────────────────────────────────────────────────
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(level.w, level.h),
    new THREE.MeshLambertMaterial({ color: 0x23272f })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(level.w / 2, 0, level.h / 2);
  scene.add(floor);

  const gSz = Math.max(Math.ceil(level.w), Math.ceil(level.h));
  const grid = new THREE.GridHelper(gSz, gSz, 0x2a2f3a, 0x2a2f3a);
  grid.position.set(level.w / 2, 0.001, level.h / 2);
  scene.add(grid);

  // ── goal zone ─────────────────────────────────────────────────────────────
  const g = level.goal;
  const goalGrp = new THREE.Group();
  goalGrp.position.set(g.cx, 0, g.cy);
  if (g.ang) goalGrp.rotation.y = -g.ang;

  const goalPl = new THREE.Mesh(
    new THREE.PlaneGeometry(g.w, g.h),
    new THREE.MeshBasicMaterial({ color: 0xf2c84b, transparent: true, opacity: 0.18,
                                  depthWrite: false, side: THREE.DoubleSide })
  );
  goalPl.rotation.x = -Math.PI / 2; goalPl.position.y = 0.005;
  goalGrp.add(goalPl);

  // dashed border approximated with 4 line segments
  const bPts = [[-g.w/2,-g.h/2],[g.w/2,-g.h/2],[g.w/2,g.h/2],[-g.w/2,g.h/2],[-g.w/2,-g.h/2]]
    .map(([x,z]) => new THREE.Vector3(x, 0.008, z));
  goalGrp.add(new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(bPts),
    new THREE.LineBasicMaterial({ color: 0xf2c84b, opacity: 0.80, transparent: true })
  ));
  scene.add(goalGrp);

  // ── obstacles ─────────────────────────────────────────────────────────────
  const WALL_H = 1.4, CURB_H = 0.22;

  for (const o of level.obstacles) {
    if (o.kind === 'border') continue;

    if (o.kind === 'wall' || o.kind === 'curb') {
      const r = o.rect; if (!r) continue;
      const hgt = o.kind === 'curb' ? CURB_H : WALL_H;
      const clr = o.kind === 'curb' ? 0x4a5266 : 0x38404f;
      const mat = new THREE.MeshLambertMaterial({ color: clr });
      let mesh: any;
      if (r.ang != null) {
        mesh = new THREE.Mesh(new THREE.BoxGeometry(r.w, hgt, r.h), mat);
        mesh.position.set(r.cx, hgt / 2, r.cy); mesh.rotation.y = -r.ang;
      } else {
        mesh = new THREE.Mesh(new THREE.BoxGeometry(r.w, hgt, r.h), mat);
        mesh.position.set(r.x! + r.w / 2, hgt / 2, r.y! + r.h / 2);
      }
      scene.add(mesh);
      // Lighter cap on top of walls
      if (o.kind === 'wall') {
        const capMat = new THREE.MeshLambertMaterial({ color: 0x4e5870 });
        const cap = new THREE.Mesh(new THREE.BoxGeometry(r.ang != null ? r.w : r.w, 0.05,
                                                          r.ang != null ? r.h : r.h), capMat);
        cap.position.copy(mesh.position); cap.position.y = hgt + 0.025;
        cap.rotation.copy(mesh.rotation); scene.add(cap);
      }
    } else if (o.kind === 'car') {
      const sp = o.carSpec || sedan;
      const vt = o.pose!.type || 'default';
      const obstColor = vt === 'miata' ? 0xd23b3b : 0x737d8c;
      const carGrp = buildCar3D(sp, obstColor, vt);
      carGrp.position.set(o.pose!.cx, 0, o.pose!.cy);
      carGrp.rotation.y = -o.pose!.h;
      scene.add(carGrp);
    }
  }

  // ── traffic ───────────────────────────────────────────────────────────────
  const trafficObjs: Array<{ grp: any; tr: TrafficDef }> = [];
  if (level.traffic) {
    for (const tr of level.traffic) {
      const clrHex = parseInt((tr.color || '#4e5a6e').replace('#', ''), 16);
      const tGrp = buildCar3D(sedan, clrHex, 'default');
      tGrp.position.set(tr.x, 0, tr.y);
      tGrp.rotation.y = -tr.h;
      scene.add(tGrp);
      trafficObjs.push({ grp: tGrp, tr });
    }
  }

  // ── player car ────────────────────────────────────────────────────────────
  const vType   = level.vehicle || 'default';
  const pColor  = vType === 'miata' ? 0xd23b3b : vType === 'tractor' ? 0xe8760a : 0x45c4ff;
  const pGrp    = buildCar3D(car, pColor, vType);
  const pBodFwd = car.len / 2 - car.rOver;   // rear-axle → body-centre offset

  function posePcar(pose: Pose): void {
    pGrp.position.set(
      pose.x + Math.cos(pose.h) * pBodFwd, 0,
      pose.y + Math.sin(pose.h) * pBodFwd
    );
    pGrp.rotation.y = -pose.h;
  }
  posePcar(level.start);
  scene.add(pGrp);

  // ── replay samples from planSims ──────────────────────────────────────────
  let rSamples: Pose[] | null = null, rCum: number[] | null = null;
  let rTotal = 0, rSpeed = 0;
  if (moves.length && planSims.length === moves.length) {
    const samp: Pose[] = [], cum: number[] = [];
    let tot = 0;
    for (let i = 0; i < planSims.length; i++) {
      const sim = planSims[i];
      const n = Math.max(1, sim.pts.length - 1);
      const step = Math.abs(moves[i].dist) / n;
      for (let j = (i === 0 ? 0 : 1); j < sim.pts.length; j++) {
        if (j > 0) tot += step;
        samp.push(sim.pts[j]); cum.push(tot);
      }
    }
    rSamples = samp; rCum = cum; rTotal = tot;
    rSpeed = clamp(rTotal / 3, 2.5, 7);
  }

  // ── camera ────────────────────────────────────────────────────────────────
  const camFocus = new THREE.Vector3(level.w / 2, 0, level.h / 2);
  const FOV_Y  = 55 * Math.PI / 180;
  const tanHV  = Math.tan(FOV_Y / 2);

  // camA: top-down height matching 2D canvas scale exactly.
  const camAH  = H / (viewScale * 2 * tanHV);
  const camA   = new THREE.Vector3(level.w / 2, camAH, level.h / 2);

  // camB: raised camera that keeps the level framed with a small border.
  // Compute slant distance R so the level just fits (with ~10% border each side)
  // in both axes given the actual canvas aspect ratio and FOV.
  const BORDER  = 0.80;                                    // 80 % of frame → 10 % border each side
  const R_forH  = level.h / (2 * BORDER * tanHV);         // height-driven min slant distance
  const R_forW  = (level.w / W * H) / (2 * BORDER * tanHV); // width-driven (accounts for aspect ratio)
  const R       = Math.max(R_forH, R_forW) * 1.15;        // +15 % for near/far perspective spread
  const elev    = 42 * Math.PI / 180;                      // elevation angle above horizontal
  const camB    = new THREE.Vector3(level.w / 2, R * Math.sin(elev), level.h / 2 + R * Math.cos(elev));

  const camera = new THREE.PerspectiveCamera(55, W / H, 0.1, diag * 15);
  camera.position.copy(camA);
  camera.lookAt(camFocus);

  // ── close / exit ──────────────────────────────────────────────────────────
  let _alive = true, _replayT0: number | null = null;
  let _exitT0: number | null = null, _exitCamFrom: any = null;
  const CROSS_MS = 450, MORPH_MS = 1400;
  const HOLD_MS = 1500, EXIT_MORPH_MS = 900, EXIT_FADE_MS = 400;

  function startExit(skipHold: boolean): void {
    if (_exitT0) return;
    _exitCamFrom = camera.position.clone();
    _exitT0 = performance.now() - (skipHold ? HOLD_MS : 0);
  }
  function closeView(): void {
    if (!_alive) return;
    _alive = false;
    v3d.classList.add('hidden'); renderer.dispose();
    if (inGoal(planEnd(), level.goal)) finishRun();
    onClose();
  }
  v3dClose.onclick = () => startExit(true);
  cv3.addEventListener('pointerdown', (e: PointerEvent) => { if (e.isPrimary) startExit(true); }, { once: true });

  // ── animation loop ────────────────────────────────────────────────────────
  // Entry:  crossfade (2D→3D opacity) → camera top-down→raised → replay
  // Exit:   hold → camera raised→top-down + fade out → close
  const t0 = performance.now();
  const easeIO = (t: number) => t < 0.5 ? 2*t*t : -1 + (4 - 2*t)*t;

  function frame(now: number): void {
    if (!_alive) return;
    const el = now - t0;

    if (!_exitT0) {
      // entry phases
      if (el < CROSS_MS) {
        cv3.style.opacity = (el / CROSS_MS).toFixed(3);
        camera.position.copy(camA);
        camera.lookAt(camFocus);
      } else {
        cv3.style.opacity = '1';
        const mt = Math.min(1, (el - CROSS_MS) / MORPH_MS);
        camera.position.lerpVectors(camA, camB, easeIO(mt));
        camera.lookAt(camFocus);
        if (mt >= 1 && rSamples && !_replayT0) _replayT0 = now;
        if (mt >= 1 && !rSamples) startExit(false);
      }
      if (_replayT0 && rSamples && rCum) {
        const trav = Math.min(rTotal, (now - _replayT0) / 1000 * rSpeed);
        let lo = 0, hi = rCum.length - 1;
        while (lo < hi) { const m = (lo + hi) >> 1; if (rCum[m] < trav) lo = m + 1; else hi = m; }
        posePcar(rSamples[lo]);
        if (trav >= rTotal) startExit(false);
      }
    } else {
      // exit phases: hold → reverse morph + fade out
      const et = now - _exitT0;
      if (et >= HOLD_MS) {
        const p = et - HOLD_MS;
        const mt = Math.min(1, p / EXIT_MORPH_MS);
        camera.position.lerpVectors(_exitCamFrom, camA, easeIO(mt));
        camera.lookAt(camFocus);
        const fadeEl = Math.max(0, p - EXIT_MORPH_MS * 0.4);
        cv3.style.opacity = (1 - Math.min(1, fadeEl / EXIT_FADE_MS)).toFixed(3);
        if (mt >= 1 && fadeEl >= EXIT_FADE_MS) { closeView(); return; }
      }
    }

    const tSec = now / 1000;
    for (const { grp, tr } of trafficObjs) {
      const d = (tSec * tr.speed + tr.offset) % tr.loop;
      grp.position.set(tr.x + Math.cos(tr.h) * d, 0, tr.y + Math.sin(tr.h) * d);
    }

    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
}
