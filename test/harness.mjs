// Headless Node harness for the analytic humanoid controller (runs the exact shipped src/controllers code).
//   const r = await runTrial({ T: 10, cmd: t => ({ walk: true, vx: 0.3 }), pushes: [{ t: 3, dur: 0.1, F: [100, 0, 0] }] });
import load_mujoco from '../node_modules/@mujoco/mujoco/mujoco.js';
import { readFileSync, readdirSync, statSync } from 'fs';
import { join, relative, dirname } from 'path';
import { fileURLToPath } from 'url';
import { AnalyticHumanoidController } from '../src/controllers/analyticHumanoid.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const SCENES = join(ROOT, 'assets', 'scenes');
let _mujoco = null;

export async function getMujoco() {
  if (_mujoco) return _mujoco;
  const mujoco = await load_mujoco();
  mujoco.FS.mkdir('/working');
  mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
  const walk = (dir) => {
    for (const entry of readdirSync(dir)) {
      const full = join(dir, entry), rel = relative(SCENES, full);
      if (statSync(full).isDirectory()) { mujoco.FS.mkdir('/working/' + rel); walk(full); }
      else mujoco.FS.writeFile('/working/' + rel, readFileSync(full));
    }
  };
  walk(SCENES);
  return (_mujoco = mujoco);
}

/**
 * opts: { scene, T, cmd(t), pushes: [{t, dur, F, body}], hipBoost (default 2), params, timestep, statsFrom, log: {every, fn} }
 * Returns { fell, fallT, vxMean, vyMean, yaw0, yawEnd, satPct, ctrlMsPerStep, simMsPerStep, steps, captureSteps, mode, dx, dy }
 */
export async function runTrial(opts = {}) {
  const mujoco = await getMujoco();
  const m = mujoco.MjModel.mj_loadXML('/working/' + (opts.scene || 'humanoid.xml'));
  const d = new mujoco.MjData(m);
  if (opts.timestep) m.opt.timestep = opts.timestep;
  const ctrl = new AnalyticHumanoidController(mujoco, m, d, { hipBoost: opts.hipBoost ?? 2, params: opts.params });
  if (opts.init) opts.init(mujoco, m, d);
  mujoco.mj_forward(m, d);
  const dt = m.opt.timestep, nsteps = Math.round((opts.T ?? 10) / dt);
  const pushes = opts.pushes || [];
  const head = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY.value, 'head');
  const warm = Math.round((opts.statsFrom ?? 0) / dt);
  const x0 = d.qpos[0], y0 = d.qpos[1];
  let fallT = null, satSteps = 0, ctrlMs = 0, simMs = 0, sx0 = x0, sy0 = y0, st0 = 0, yaw0 = 0, n = 0;
  const trace = [];
  for (let i = 0; i < nsteps; i++) {
    const t = d.time;
    d.xfrc_applied.fill(0);
    for (const p of pushes) {
      if (t >= p.t && t < p.t + p.dur) {
        const b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY.value, p.body || 'torso');
        for (let k = 0; k < 3; k++) d.xfrc_applied[6 * b + k] += p.F[k];
      }
    }
    const cmd = opts.cmd ? opts.cmd(t) : {};
    const t0 = performance.now();
    mujoco.mj_step1(m, d);
    const t1 = performance.now();
    ctrl.step(cmd);
    const t2 = performance.now();
    mujoco.mj_step2(m, d);
    const t3 = performance.now();
    ctrlMs += t2 - t1; simMs += (t1 - t0) + (t3 - t2); n++;
    let anySat = false;
    for (let a = 0; a < m.nu; a++) if (Math.abs(d.ctrl[a]) >= 0.999) anySat = true;
    if (anySat) satSteps++;
    if (i === warm) { sx0 = d.qpos[0]; sy0 = d.qpos[1]; st0 = d.time; yaw0 = ctrl.yaw(); }
    if (opts.log && i % opts.log.every === 0) trace.push(opts.log.fn(ctrl, d, t));
    if (!(d.xpos[3 * head + 2] > 0.9) || !isFinite(d.qpos[2])) { fallT = d.time; break; }
  }
  const dur = d.time - st0;
  const res = {
    fell: fallT !== null, fallT, T: d.time, dx: d.qpos[0] - x0, dy: d.qpos[1] - y0,
    vxMean: dur > 0 ? (d.qpos[0] - sx0) / dur : 0, vyMean: dur > 0 ? (d.qpos[1] - sy0) / dur : 0,
    yaw0, yawEnd: ctrl.yaw(), satPct: 100 * satSteps / Math.max(1, n),
    ctrlMsPerStep: ctrlMs / Math.max(1, n), simMsPerStep: simMs / Math.max(1, n),
    steps: ctrl.S.steps, captureSteps: ctrl.S.captureSteps, mode: ctrl.mode, comZ: ctrl.core.com[2], trace,
  };
  ctrl.dispose(); d.delete(); m.delete();
  return res;
}
