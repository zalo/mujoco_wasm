import load_mujoco from './node_modules/@mujoco/mujoco/mujoco.js';
import { DiffIK } from './src/utils/DiffIK.js';
import { readFileSync, readdirSync, statSync } from 'fs';
import { join, relative } from 'path';
const mujoco = await load_mujoco();
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
function copyDir(dir) {
  for (const entry of readdirSync(dir)) {
    const full = join(dir, entry);
    const rel = relative('assets/scenes', full);
    if (statSync(full).isDirectory()) { mujoco.FS.mkdir('/working/' + rel); copyDir(full); }
    else { mujoco.FS.writeFile('/working/' + rel, readFileSync(full)); }
  }
}
copyDir('assets/scenes');
const model = mujoco.MjModel.mj_loadXML('/working/ufactory_xarm7/scene_teleop.xml');
const data = new mujoco.MjData(model);
mujoco.mj_forward(model, data);
const ik = new DiffIK(mujoco, model, { siteId: 0, reachCenter: [0, 0, 1.3], reachRadius: 0.95 });
console.log('arm actuators:', ik.armActIds.join(','), '| guarded geoms:', ik.guardGeoms.length);

const FRAME_DT = 1 / 60, STEPS = Math.round(FRAME_DT / model.opt.timestep);
const down = [0, 1, 0, 0];
let minSphereBottom = Infinity, maxJointVel = 0, groundLimitedFrames = 0;
const prevQ = new Float64Array(7);
const prevCtrl = new Float64Array(7);
function runFrames(target, quat, frames) {
  for (let f = 0; f < frames; f++) {
    const t = [...target];
    ik.clampTarget(t);
    prevCtrl.set(ik.lastCommand);
    ik.step(data, t, quat, FRAME_DT);
    /* joint speed measured from sim below */
    if (ik.status.groundLimited) groundLimitedFrames++;
    for (let k = 0; k < 7; k++) prevQ[k] = data.qpos[k];
    for (let s = 0; s < STEPS; s++) mujoco.mj_step(model, data);
    for (let k = 0; k < 7; k++) maxJointVel = Math.max(maxJointVel, Math.abs(data.qpos[k] - prevQ[k]) / FRAME_DT);
    // measure REAL (simulated) clearance of guarded geoms
    for (const g of ik.guardGeoms) {
      minSphereBottom = Math.min(minSphereBottom, data.geom_xpos[g*3+2] - model.geom_rbound[g]);
    }
  }
}
const tcp = () => [...data.site_xpos.slice(0, 3)];
const perr = (t) => { const s = tcp(); return Math.hypot(t[0]-s[0], t[1]-s[1], t[2]-s[2]) * 1000; };

// seed lastCommand for step-size measurement
for (let k = 0; k < 7; k++) prevCtrl[k] = 0;

// 1. reachable target from the folded qpos0 (worst case; slow but must converge)
let t1 = [0.3, -0.35, 1.1];
runFrames(t1, down, 720);
const e1 = perr(t1);
console.log('T1 from qpos0 : tcp', tcp().map(v=>v.toFixed(3)).join(','), '| err', e1.toFixed(1)+'mm');

// 2. target far below the floor: arm must stop above ground
let t2 = [0.3, -0.35, -0.30];
runFrames(t2, down, 360);
console.log('T2 below floor: tcp', tcp().map(v=>v.toFixed(3)).join(','), '| clamped target z', ik.clampTarget([...t2])[2]);

// 3. lateral sweep while pressing down
for (let i = 0; i <= 60; i++) { runFrames([0.3 - i * 0.01, -0.35, -0.2], down, 4); }
console.log('T3 low sweep  : tcp', tcp().map(v=>v.toFixed(3)).join(','));

// 4. back up high and to the other side
let t4 = [-0.2, -0.35, 1.2];
runFrames(t4, down, 480);
const e4 = perr(t4);
console.log('T4 recover    : tcp', tcp().map(v=>v.toFixed(3)).join(','), '| err', e4.toFixed(1)+'mm');

// 5. from the home keyframe (how the scene actually starts), hand-scale move
mujoco.mj_resetDataKeyframe(model, data, 0);
mujoco.mj_forward(model, data);
let t5 = [0.09, -0.40, 1.1];
runFrames(t5, down, 120);
const e5 = perr(t5);
console.log('T5 from home  : tcp', tcp().map(v=>v.toFixed(3)).join(','), '| err', e5.toFixed(1)+'mm  (2s budget)');

console.log('--- safety stats over all frames ---');
console.log('min real bounding-sphere bottom:', (minSphereBottom*1000).toFixed(1)+'mm  (>=0 means meshes can never have touched ground)');
console.log('max real joint velocity:', maxJointVel.toFixed(2), 'rad/s (command limit 2.0)');
console.log('ground-gate active on', groundLimitedFrames, 'frames');
const pass = minSphereBottom > 0 && maxJointVel < 3.0 && e1 < 20 && e4 < 20 && e5 < 20;
console.log(pass ? 'PASS' : 'FAIL');
ik.dispose(); data.delete(); model.delete();
process.exit(pass ? 0 : 1);
