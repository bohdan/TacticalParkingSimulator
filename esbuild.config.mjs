// Build script: bundles each entry point with esbuild.
//
//   node esbuild.config.mjs            one-shot build, fixed filenames (local `npm run build`)
//   node esbuild.config.mjs --watch    same, rebuilds on change (`npm run dev`)
//   node esbuild.config.mjs --hash     content-hashed filenames + HTML rewrite (CI/deploy only)
//
// solver.ts spawns solver-worker.ts as a Web Worker via a runtime string
// (`new Worker(new URL(url, import.meta.url))`), which esbuild can't rewrite the way it
// rewrites static `import` specifiers. So the worker's URL is a compile-time constant,
// `__SOLVER_WORKER_URL__`, injected via `define`. In hashed mode that means building the
// worker first to learn its hashed filename, then defining it for the second pass.
import * as esbuild from 'esbuild';
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';

const HASH = process.argv.includes('--hash');
const WATCH = process.argv.includes('--watch');

const ENTRIES = {
  game: 'game.ts',
  editor: 'editor.ts',
  'truck-physics-demo': 'truck-physics-demo.ts',
  'touch-controls-demo': 'touch-controls-demo.ts',
  'solver-worker': 'solver-worker.ts',
};

const common = {
  bundle: true,
  format: 'esm',
  target: 'es2020',
  outdir: 'build',
  sourcemap: true,
  logLevel: 'info',
  // render-3d.ts (and the Three.js library it pulls in) is only reached via the
  // dynamic `import('./render-3d.js')` in game.ts, so splitting gives it its own
  // chunk file instead of bloating every entry point that doesn't use 3D.
  splitting: true,
};

async function buildHashed() {
  // Pass 1: the worker alone, so its final (hashed) filename is known.
  const workerResult = await esbuild.build({
    ...common,
    entryPoints: { 'solver-worker': ENTRIES['solver-worker'] },
    entryNames: '[name]-[hash]',
    metafile: true,
  });
  const workerOut = Object.keys(workerResult.metafile.outputs).find(f => f.endsWith('.js'));
  const workerUrl = './' + path.basename(workerOut);

  // Pass 2: everything else, with the worker's URL baked in.
  const { 'solver-worker': _skip, ...rest } = ENTRIES;
  const mainResult = await esbuild.build({
    ...common,
    entryPoints: rest,
    entryNames: '[name]-[hash]',
    metafile: true,
    define: { __SOLVER_WORKER_URL__: JSON.stringify(workerUrl) },
  });

  rewriteHtml({ ...workerResult.metafile.outputs, ...mainResult.metafile.outputs });
}

function rewriteHtml(outputs) {
  const nameFor = {};
  for (const [outPath, info] of Object.entries(outputs)) {
    if (!outPath.endsWith('.js')) continue;
    const entry = info.entryPoint && path.basename(info.entryPoint, '.ts');
    if (entry) nameFor[entry] = path.basename(outPath);
  }
  const cssHash = crypto.createHash('sha256').update(fs.readFileSync('style.css')).digest('hex').slice(0, 10);

  for (const html of ['index.html', 'editor.html', 'truck-physics-demo.html', 'touch-controls-demo.html']) {
    if (!fs.existsSync(html)) continue;
    let src = fs.readFileSync(html, 'utf8');
    src = src.replace(/(src=["'])build\/([\w-]+)\.js(["'])/g, (m, pre, name, post) =>
      nameFor[name] ? `${pre}build/${nameFor[name]}${post}` : m);
    src = src.replace(/(href=["'])style\.css(["'])/, `$1style.css?v=${cssHash}$2`);
    fs.writeFileSync(html, src);
  }
  console.log('Rewrote HTML references to hashed build output:', nameFor);
}

async function buildFixed() {
  const opts = {
    ...common,
    entryPoints: ENTRIES,
    entryNames: '[name]',
    define: { __SOLVER_WORKER_URL__: JSON.stringify('./solver-worker.js') },
  };
  if (WATCH) {
    const ctx = await esbuild.context(opts);
    await ctx.watch();
    console.log('esbuild watching for changes...');
  } else {
    await esbuild.build(opts);
  }
}

if (HASH) await buildHashed();
else await buildFixed();
