// Soak test: a random command (or a stop) every 3 s for 120 s, several seeds. Transitions are the fragile part.
//   node test/soak.mjs [seeds=8] [stopProb=0.25] [pushN=0] [hipBoost=2] [paramsJSON]
import { runTrial } from './harness.mjs';

const nSeeds = parseInt(process.argv[2] ?? '8'), stopProb = parseFloat(process.argv[3] ?? '0.25');
const pushN = parseFloat(process.argv[4] ?? '0'), hipBoost = parseFloat(process.argv[5] ?? '2');
const params = JSON.parse(process.argv[6] || '{}');
const firstSeed = parseInt(process.env.SEED0 ?? '1');
let falls = 0;
for (let seed = firstSeed; seed < firstSeed + nSeeds; seed++) {
  let s = seed * 7919; const rnd = () => (s = (s * 16807) % 2147483647) / 2147483647;
  const segs = [];
  for (let k = 0; k < 40; k++) segs.push(rnd() < stopProb ? { walk: false } : { walk: true, vx: -0.3 + 1.0 * rnd(), vy: -0.2 + 0.4 * rnd(), wz: -0.6 + 1.2 * rnd() });
  const pushes = [];
  if (pushN > 0) for (let t = 5; t < 118; t += 4) { const a = 2 * Math.PI * rnd(); pushes.push({ t: t + rnd(), dur: 0.1, F: [pushN * Math.cos(a), pushN * Math.sin(a), 0] }); }
  let stops = 0, last = 'stand';
  const r = await runTrial({ hipBoost, params, T: 120, pushes, cmd: (t) => t < 1 ? { walk: false } : segs[Math.min(39, Math.floor((t - 1) / 3))],
    log: { every: 10, fn: (c) => { if (last === 'walk' && c.mode === 'stand') stops++; last = c.mode; } } });
  if (r.fell) falls++;
  const seg = r.fell ? segs[Math.min(39, Math.floor((r.fallT - 1) / 3))] : null;
  console.log(`seed ${seed}: ${r.fell ? 'FELL @' + r.fallT.toFixed(1) + 's during ' + JSON.stringify(seg, (k, v) => typeof v === 'number' ? +v.toFixed(2) : v) : 'ok'}`
    + `  steps=${r.steps} stops=${stops} captureSteps=${r.captureSteps} sat%=${r.satPct.toFixed(0)}`);
}
console.log(`hip x${hipBoost}, stopProb ${stopProb}, pushes ${pushN} N: ${nSeeds - falls}/${nSeeds} seeds survived 120 s`);
process.exit(falls ? 1 : 0);
