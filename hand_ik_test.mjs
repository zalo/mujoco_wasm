// Regression test for the xArm7 + Shadow Hand teleop stack: 9-dof arm IK
// (7 arm joints + 2 wrist joints) tracking the palm site, fingertip
// retargeting through the 18 finger actuators, and the ground-clearance
// safeties with the hand mounted.
import load_mujoco from './node_modules/@mujoco/mujoco/mujoco.js';
import { DiffIK } from './src/utils/DiffIK.js';
import { HandRetarget } from './src/utils/HandRetarget.js';
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

const model = mujoco.MjModel.mj_loadXML('/working/ufactory_xarm7/scene_hand_teleop.xml');
const data = new mujoco.MjData(model);

// --- replicate the demo's teleop detection ---
const names = new Uint8Array(model.names);
const dec = new TextDecoder();
const nm = (adr) => { let e = adr; while (names[e] !== 0) e++; return dec.decode(names.subarray(adr, e)); };
let tcpSiteId = -1;
for (let s = 0; s < model.nsite; s++) { if (nm(model.name_siteadr[s]).endsWith('grasp_site')) tcpSiteId = s; }
const chainJoints = new Set();
for (let p = model.site_bodyid[tcpSiteId]; p != 0; p = model.body_parentid[p]) {
  for (let j = model.body_jntadr[p]; j < model.body_jntadr[p] + model.body_jntnum[p]; j++) chainJoints.add(j);
}
const armActIds = [];
for (let a = 0; a < model.nu; a++) {
  if (model.actuator_trntype[a] == 0 && chainJoints.has(model.actuator_trnid[a * 2])) armActIds.push(a);
}
const tipSuffixes = ['thdistal', 'ffdistal', 'mfdistal', 'rfdistal', 'lfdistal'];
const tipBodyIds = tipSuffixes.map((s) => {
  for (let b = 0; b < model.nbody; b++) { if (nm(model.name_bodyadr[b]).endsWith(s)) return b; }
  return -1;
});
const handActIds = [];
for (let a = 0; a < model.nu; a++) { if (!armActIds.includes(a)) handActIds.push(a); }
console.log('arm actuators:', armActIds.length, '(expect 9)  hand actuators:', handActIds.length,
  '(expect 18)  tips found:', tipBodyIds.every((b) => b >= 0));

// start at home like the demo
for (let a = 0; a < model.nu; a++) {
  if (model.actuator_trntype[a] == 0) {
    const adr = model.jnt_qposadr[model.actuator_trnid[2 * a]];
    data.qpos[adr] = model.key_qpos[adr];
    data.ctrl[a] = model.key_qpos[adr];
  }
}
mujoco.mj_forward(model, data);

// mirror the demo: stiffen weak wrist servos on the chain
for (const a of armActIds) {
  if (model.actuator_gainprm[a * 10] < 50) {
    model.actuator_gainprm[(a * 10) + 0] = 300;
    model.actuator_biasprm[(a * 10) + 1] = -300;
    model.actuator_biasprm[(a * 10) + 2] = -30;
    model.actuator_forcerange[(a * 2) + 0] = -30;
    model.actuator_forcerange[(a * 2) + 1] = 30;
  }
}
const ik = new DiffIK(mujoco, model, { siteId: tcpSiteId, armActIds: armActIds,
  reachCenter: [0, 0, 1.3], reachRadius: 0.95 });
const hr = new HandRetarget(mujoco, model, { tipBodyIds: tipBodyIds, actIds: handActIds });

const FRAME_DT = 1 / 60, STEPS = Math.round(FRAME_DT / model.opt.timestep);
let minSphereBottom = Infinity;
const trackClearance = () => {
  for (const g of ik.guardGeoms) {
    minSphereBottom = Math.min(minSphereBottom, data.geom_xpos[g * 3 + 2] - model.geom_rbound[g]);
  }
};
const palm = () => [...data.site_xpos.slice(tcpSiteId * 3, tcpSiteId * 3 + 3)];
const tip = (f) => [...data.xpos.slice(tipBodyIds[f] * 3, tipBodyIds[f] * 3 + 3)];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
// Like the demo, fingertip targets live in the palm-site frame, so arm
// drift/sag cancels out of the finger tasks.
const toSiteFrame = (w) => {
  const p = palm(), R = [...data.site_xmat.slice(tcpSiteId * 9, tcpSiteId * 9 + 9)];
  const d = [w[0] - p[0], w[1] - p[1], w[2] - p[2]];
  return [R[0]*d[0]+R[3]*d[1]+R[6]*d[2], R[1]*d[0]+R[4]*d[1]+R[7]*d[2], R[2]*d[0]+R[5]*d[1]+R[8]*d[2]];
};
const toWorld = (l) => {
  const p = palm(), R = [...data.site_xmat.slice(tcpSiteId * 9, tcpSiteId * 9 + 9)];
  return [p[0]+R[0]*l[0]+R[1]*l[1]+R[2]*l[2], p[1]+R[3]*l[0]+R[4]*l[1]+R[5]*l[2], p[2]+R[6]*l[0]+R[7]*l[1]+R[8]*l[2]];
};

// T1: palm tracks a reachable target, hand pointing down
const down = [0, 1, 0, 0];
let target = [0.09, -0.4, 1.2];
let fingerTargets = null;
const runFrames = (frames) => {
  for (let f = 0; f < frames; f++) {
    const t = [...target];
    ik.clampTarget(t);
    ik.step(data, t, down, FRAME_DT);
    if (fingerTargets) hr.step(data, fingerTargets.map(toWorld), FRAME_DT);
    for (let s = 0; s < STEPS; s++) mujoco.mj_step(model, data);
    trackClearance();
  }
};
runFrames(300);
const e1 = dist(palm(), target) * 1000;
console.log('T1 palm tracking  : palm', palm().map(v => v.toFixed(3)).join(','), '| err', e1.toFixed(1) + 'mm');

// T2: fingertip retargeting toward a known-feasible half-curl pose: drive
// the actuators there directly, record the tips, reset open, then ask the
// retargeter to reach the recorded tips.
const openTips = tipBodyIds.map((_, f) => toSiteFrame(tip(f)));
for (let k = 0; k < handActIds.length; k++) {
  const a = handActIds[k], name = nm(model.name_actuatoradr[a]);
  if (name.endsWith('J3')) { data.ctrl[a] = 0.8; }
  if (name.endsWith('J0')) { data.ctrl[a] = 1.2; }
  if (name.endsWith('THJ4')) { data.ctrl[a] = 0.9; }
  if (name.endsWith('THJ2')) { data.ctrl[a] = 0.5; }
}
runFrames(150);
const curledTips = tipBodyIds.map((_, f) => toSiteFrame(tip(f)));
for (let k = 0; k < handActIds.length; k++) { data.ctrl[handActIds[k]] = 0; } // reopen
runFrames(150);

fingerTargets = curledTips;
const before = tipBodyIds.map((_, f) => dist(tip(f), toWorld(fingerTargets[f])) * 1000);
runFrames(240);
const after = tipBodyIds.map((_, f) => dist(tip(f), toWorld(fingerTargets[f])) * 1000);
console.log('T2 finger curl    : tip errors', before.map(v => v.toFixed(0)).join('/'), '->', after.map(v => v.toFixed(0)).join('/'), 'mm');
const e2 = Math.max(...after);

// T3: reopen — retarget back to the recorded open tips
fingerTargets = openTips;
runFrames(240);
const reopenErr = tipBodyIds.map((_, f) => dist(tip(f), toWorld(fingerTargets[f])) * 1000);
console.log('T3 reopened       : tip errors', reopenErr.map(v => v.toFixed(0)).join('/'), 'mm | thumb-index gap', (dist(tip(0), tip(1)) * 1000).toFixed(0) + 'mm');

// T4: command the palm below the floor — ground safeties must hold
target = [0.09, -0.4, -0.3];
fingerTargets = null;
runFrames(300);
console.log('T4 below floor    : palm z', palm()[2].toFixed(3), '(target clamped to', ik.clampTarget([0.09, -0.4, -0.3])[2] + ')');

// T5: fingers commanded straight down while the palm is at the floor limit
fingerTargets = tipBodyIds.map((_, f) => { const p = tip(f); return [p[0], p[1], -0.2]; });
runFrames(240);
const tipZs = tipBodyIds.map((_, f) => tip(f)[2]);
console.log('T5 fingers to floor: min commanded-tip z', Math.min(...tipZs).toFixed(3), '| hand gate active:', hr.status.groundLimited);

console.log('--- min real bounding-sphere bottom:', (minSphereBottom * 1000).toFixed(1) + 'mm ---');
const pass = e1 < 25 && e2 < 25 && minSphereBottom > 0;
console.log(pass ? 'PASS' : 'FAIL');
ik.dispose(); hr.dispose(); data.delete(); model.delete();
process.exit(pass ? 0 : 1);
