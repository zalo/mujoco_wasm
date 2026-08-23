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
const gravcomp = process.argv[2] == 'gravcomp';
const model = mujoco.MjModel.mj_loadXML('/working/ufactory_xarm7/scene_hand_teleop.xml');
const data = new mujoco.MjData(model);
console.log('body_gravcomp writable:', !!model.body_gravcomp, model.body_gravcomp?.length);
const chainJoints = new Set();
for (let p = model.site_bodyid[0]; p != 0; p = model.body_parentid[p]) {
  for (let j = model.body_jntadr[p]; j < model.body_jntadr[p] + model.body_jntnum[p]; j++) chainJoints.add(j);
}
const armActIds = [];
for (let a = 0; a < model.nu; a++) if (model.actuator_trntype[a] == 0 && chainJoints.has(model.actuator_trnid[a*2])) armActIds.push(a);
for (let a = 0; a < model.nu; a++) if (model.actuator_trntype[a] == 0) {
  const adr = model.jnt_qposadr[model.actuator_trnid[2*a]];
  data.qpos[adr] = model.key_qpos[adr]; data.ctrl[a] = model.key_qpos[adr];
}
for (const a of armActIds) {
  if (model.actuator_gainprm[a*10] < 50) {
    model.actuator_gainprm[a*10] = 300; model.actuator_biasprm[a*10+1] = -300; model.actuator_biasprm[a*10+2] = 0;
    model.actuator_forcerange[a*2] = -30; model.actuator_forcerange[a*2+1] = 30;
    model.dof_damping[model.jnt_dofadr[model.actuator_trnid[a*2]]] = 8;
  }
}
const root = model.body_rootid[model.site_bodyid[0]];
const robotDofs = [];
for (let d = 0; d < model.nv; d++) if (model.body_rootid[model.dof_bodyid[d]] == root) robotDofs.push(d);
const stepOnce = () => {
  if (gravcomp) {
    for (let i = 0; i < data.qfrc_applied.length; i++) data.qfrc_applied[i] = 0;
    for (const d of robotDofs) data.qfrc_applied[d] += data.qfrc_bias[d];
  }
  mujoco.mj_step(model, data);
};
mujoco.mj_forward(model, data);
// settle 1s
for (let i = 0; i < 500; i++) stepOnce();
// what's touching at settle?
{
  const names = new Uint8Array(model.names); const dec = new TextDecoder();
  const nm = (adr) => { let e=adr; while(names[e]!==0)e++; return dec.decode(names.subarray(adr,e)); };
  const pairs = new Set();
  const cvec = data.contact;
  for (let i = 0; i < data.ncon; i++) {
    const c = cvec.get(i);
    if (c.dist < 0.001) pairs.add(nm(model.name_bodyadr[model.geom_bodyid[c.geom1]]) + '|' + nm(model.name_bodyadr[model.geom_bodyid[c.geom2]]));
    c.delete();
  }
  cvec.delete();
  console.log('settled contacts:', [...pairs].join('  '));
  // home tcp
  console.log('home palm site:', [...data.site_xpos.slice(0,3)].map(v=>v.toFixed(3)).join(','));
  console.log('settled actuator forces:', armActIds.map(a => +data.actuator_force[a].toFixed(1)).join(', '));
}
// per-joint step response
for (const a of armActIds) {
  const snapshot = new Float64Array(data.qpos), snapvel = new Float64Array(data.qvel), snapctrl = new Float64Array(data.ctrl);
  const adr = model.jnt_qposadr[model.actuator_trnid[2*a]];
  const q0 = data.qpos[adr];
  data.ctrl[a] = q0 + 0.15;
  let t90 = -1;
  const traj = [];
  for (let i = 0; i < 1000; i++) {
    stepOnce();
    if (i % 200 == 0) traj.push(+(data.qpos[adr] - q0).toFixed(3));
    if (t90 < 0 && (data.qpos[adr] - q0) > 0.135) { t90 = (i+1) * model.opt.timestep; break; }
  }
  const frc = data.actuator_force[a];
  console.log('act', a, 'step t90:', t90 < 0 ? '>2s' : (t90*1000).toFixed(0)+'ms',
    t90 < 0 ? '| traj ' + traj.join(',') + ' | actfrc ' + frc.toFixed(1) + ' | ncon ' + data.ncon : '');
  data.qpos.set(snapshot); data.qvel.set(snapvel); data.ctrl.set(snapctrl);
  mujoco.mj_forward(model, data);
}
