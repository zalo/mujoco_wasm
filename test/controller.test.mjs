// Acceptance tests for the analytic humanoid controller (headless, deterministic).
//   node test/controller.test.mjs            run everything
//   node test/controller.test.mjs 3 6        run only groups 3 and 6
import { runTrial } from './harness.mjs';
import { BEHAVIOURS, BehaviourRunner } from '../src/controllers/behaviours.js';

const only = process.argv.slice(2).map(Number);
const want = (g) => only.length === 0 || only.includes(g);
let pass = 0, fail = 0;
const check = (name, ok, detail = '') => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}  ${detail}`); ok ? pass++ : fail++; };
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const walkCmd = (c) => (t) => ({ walk: t > 1.0, vx: t > 2 ? (c.vx || 0) : 0, vy: t > 2 ? (c.vy || 0) : 0, wz: t > 2 ? (c.wz || 0) : 0 });
const user0 = { walk: false, vx: 0, vy: 0, wz: 0 };

if (want(1)) {
  const r = await runTrial({ T: 10, cmd: () => ({ walk: false }) });
  const drift = Math.hypot(r.dx, r.dy);
  check('1 stand 10 s', !r.fell && drift < 0.05 && r.satPct < 2, `drift=${drift.toFixed(3)} sat%=${r.satPct.toFixed(1)}`);
}
if (want(2)) for (const [n, F] of [['60 fwd', [60, 0, 0]], ['60 back', [-60, 0, 0]], ['100 left', [0, 100, 0]], ['100 right', [0, -100, 0]]]) {
  const r = await runTrial({ T: 9, pushes: [{ t: 3, dur: 0.1, F }], cmd: () => ({ walk: false }) });
  check(`2 stand push ${n} N x 0.1 s`, !r.fell, `captureSteps=${r.captureSteps}`);
}
if (want(3)) for (const vx of [0, 0.3, 0.6, -0.3]) {
  const r = await runTrial({ T: 20, statsFrom: 5, cmd: walkCmd({ vx }) });
  check(`3 walk vx=${vx}`, !r.fell && Math.abs(r.vxMean - vx) <= 0.08 && Math.abs(r.vyMean) <= 0.05 && Math.abs(wrap(r.yawEnd)) <= 0.3,
    `v=(${r.vxMean.toFixed(2)},${r.vyMean.toFixed(2)}) yaw=${r.yawEnd.toFixed(2)} sat%=${r.satPct.toFixed(0)}`);
}
if (want(4)) {
  let r = await runTrial({ T: 20, statsFrom: 5, cmd: walkCmd({ vy: 0.2 }) });
  check('4 strafe vy=0.2', !r.fell && Math.abs(r.vyMean - 0.2) <= 0.05, `vy=${r.vyMean.toFixed(2)}`);
  r = await runTrial({ T: 20, statsFrom: 5, cmd: walkCmd({ wz: 0.5 }) });
  const dyaw = wrap(r.yawEnd - r.yaw0 - 7.5);   // 0.5 rad/s for 15 s
  check('4 turn in place wz=0.5', !r.fell && Math.abs(dyaw) <= 1.5, `yaw error over 15 s = ${dyaw.toFixed(2)} rad`);
  r = await runTrial({ T: 20, statsFrom: 5, cmd: walkCmd({ vx: 0.5, wz: 0.5 }) });
  check('4 arc vx=0.5 wz=0.5', !r.fell);
  r = await runTrial({ T: 20, statsFrom: 5, cmd: walkCmd({ vx: 0.4, vy: -0.15 }) });
  check('4 diagonal (0.4,-0.15)', !r.fell && Math.abs(r.vxMean - 0.4) <= 0.08 && Math.abs(r.vyMean + 0.15) <= 0.05, `v=(${r.vxMean.toFixed(2)},${r.vyMean.toFixed(2)})`);
}
if (want(5)) {
  let ok = true, worst = '';
  for (const F of [[80, 0, 0], [-80, 0, 0], [0, 80, 0], [0, -80, 0]]) for (const t0 of [6.0, 6.1, 6.2, 6.3]) {
    const r = await runTrial({ T: 12, pushes: [{ t: t0, dur: 0.1, F }], cmd: walkCmd({}) });
    if (r.fell) { ok = false; worst += ` F=${F} t0=${t0}`; }
  }
  check('5 stepping in place + 80 N x 0.1 s pushes (4 dirs x 4 phases)', ok, worst);
}
if (want(6)) {
  const modes = []; let z1 = 0, z2 = 0;
  const r = await runTrial({ T: 30,
    cmd: (t) => t < 2 ? { walk: false } : t < 8 ? { walk: true, vx: 0.4 } : t < 13 ? { walk: false } : t < 18 ? { walk: true, vx: 0.3, wz: 0.6 }
      : t < 22 ? { walk: false } : t < 25 ? { walk: false, height: 0.62 } : { walk: false, height: 0.80 },
    log: { every: 20, fn: (c, d, t) => { const md = c.mode; if (!modes.length || modes[modes.length - 1] !== md) modes.push(md);
      if (Math.abs(t - 24.9) < 0.06) z1 = c.core.com[2]; if (Math.abs(t - 29.8) < 0.06) z2 = c.core.com[2]; } } });
  // must stop in both stop windows; a capture (shuffle) step while rising from the squat is allowed
  const seq = modes.join('>');
  check('6 stand/walk/stop/turn/stop/squat scenario', !r.fell && seq.startsWith('stand>walk>stand>walk>stand') && modes[modes.length - 1] === 'stand' && Math.abs(z1 - 0.62) < 0.03 && Math.abs(z2 - 0.80) < 0.03,
    `modes=${modes.join('>')} z=${z1.toFixed(3)},${z2.toFixed(3)}`);
}
// run a behaviour exactly as the browser UI does (BehaviourRunner holds standing behaviours until the robot is settled)
const viaRunner = (name, pre = null) => { let C = null; const runner = new BehaviourRunner(name);
  return { log: { every: 1, fn: (c) => { C = c; } }, cmd: (t) => pre && t < pre.T ? pre.cmd(t) : runner.command(t - (pre ? pre.T : 0), user0, C) }; };
if (want(7)) {
  for (const name of ['Squats', 'Sway', 'Wave', 'Dance']) {
    const r = await runTrial({ T: 20, ...viaRunner(name) });
    check(`7 behaviour: ${name}`, !r.fell, `mode=${r.mode}`);
  }
  const r = await runTrial({ T: 20, cmd: (t) => BEHAVIOURS['Wave'](t, { walk: t > 1, vx: 0.4, vy: 0, wz: 0 }) });
  check('7 behaviour: Wave while walking', !r.fell);
}
if (want(11)) {
  // the UI path that used to fall: walk (incl. turning), stop, then pick a standing behaviour straight away
  let falls = [], n = 0;
  for (const name of ['Squats', 'Sway', 'Dance']) for (const [vx, wz] of [[0.4, 0], [0.3, 0.5], [-0.2, 0], [0.5, -0.4], [0, 0.6]]) {
    const r = await runTrial({ T: 25, ...viaRunner(name, { T: 9, cmd: (t) => t < 6 ? { walk: t > 1, vx, wz } : { walk: false } }) });
    n++; if (r.fell) falls.push(`${name}(${vx},${wz})@${r.fallT.toFixed(1)}`);
  }
  check(`11 walk -> stop -> standing behaviour (${n} runs)`, falls.length === 0, falls.join(' '));
}
if (want(8)) for (const vx of [0, 0.3, -0.3]) {   // stock gears are chaotic at the saturation edge above ~0.4 m/s (see README)
  // stock actuators (hip x1) with arms forward: the hard-mode toggle. Saturation is expected; only falls are checked.
  const r = await runTrial({ hipBoost: 1, params: { armPose: [0, 0, -0.6], armSwing: 0 }, T: 20, statsFrom: 5, cmd: walkCmd({ vx }) });
  check(`8 STOCK hip gears walk vx=${vx}`, !r.fell, `v=${r.vxMean.toFixed(2)} sat%=${r.satPct.toFixed(0)}`);
}
if (want(9)) {
  const r = await runTrial({ T: 20, cmd: walkCmd({ vx: 0.4 }) });
  check('9 controller cost < 0.2 ms/step', r.ctrlMsPerStep < 0.2, `${(r.ctrlMsPerStep * 1000).toFixed(0)} us ctrl, ${(r.simMsPerStep * 1000).toFixed(0)} us physics`);
}
if (want(10)) for (const vx of [0, 0.3, 0.6]) {
  const r = await runTrial({ timestep: 0.002, T: 15, statsFrom: 5, cmd: walkCmd({ vx }) });
  check(`10 dt=0.002 walk vx=${vx}`, !r.fell && Math.abs(r.vxMean - vx) <= 0.1, `v=${r.vxMean.toFixed(2)}`);
}
console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
