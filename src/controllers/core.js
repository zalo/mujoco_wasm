// State estimation + contact-consistent feedforward ("dynamic balance force control") for humanoid.xml.
// Per physics step:  mj_step1 -> core.update() -> controller math (adds into core.tau) -> core.writeCtrl() -> mj_step2
import { clamp } from './humanoidModel.js';

export class Core {
  constructor(mujoco, m, d, idx) {
    this.mj = mujoco; this.m = m; this.d = d; this.idx = idx;
    this.nv = m.nv; this.nu = m.nu;
    this.mass = m.body_subtreemass[1];
    this.g = 9.81;
    // wasm-side scratch buffers (must be .delete()d in dispose)
    this.jp = [new mujoco.DoubleBuffer(3 * m.nv), new mujoco.DoubleBuffer(3 * m.nv)];
    this.jr = [new mujoco.DoubleBuffer(3 * m.nv), new mujoco.DoubleBuffer(3 * m.nv)];
    this.jcom = new mujoco.DoubleBuffer(3 * m.nv);
    this.tau = new Float64Array(m.nv);       // generalized torque accumulator (dof indexed)
    this.com = [0, 0, 0]; this.vcom = [0, 0, 0]; this.Lcom = [0, 0, 0];
    // feet: index 0 = right, 1 = left
    this.footBody = [idx.B.foot_right, idx.B.foot_left];
    this.footGeoms = [[idx.G.foot1_right, idx.G.foot2_right], [idx.G.foot1_left, idx.G.foot2_left]];
    this.footCenterLocal = [0.035, 0, -0.027]; // sole centre in the foot-body frame
    this.sole = [[0, 0, 0], [0, 0, 0]];      // world points where the foot wrenches act
    this.footYaw = [0, 0];
    this.contact = [false, false];
    this.footMinZ = [0, 0];
    this.yawPelvis = 0; this.yawTorso = 0;
    this.baseResidual = [0, 0, 0, 0, 0, 0];
  }

  dispose() {
    for (const b of [...this.jp, ...this.jr, this.jcom]) b.delete();
    this.jp = this.jr = this.jcom = null;
  }

  update() {
    const { mj, m, d, idx } = this;
    // COM position (fresh after mj_step1) and velocity via the COM Jacobian
    for (let k = 0; k < 3; k++) this.com[k] = d.subtree_com[3 + k];
    mj.mj_jacSubtreeCom(m, d, this.jcom, 1);
    const Jc = this.jcom.GetView(), qv = d.qvel, nv = this.nv;
    for (let k = 0; k < 3; k++) { let s = 0; for (let j = 0; j < nv; j++) s += Jc[k * nv + j] * qv[j]; this.vcom[k] = s; }
    // centroidal angular momentum (about the COM, world frame)
    mj.mj_subtreeVel(m, d);
    this.Lcom[0] = d.subtree_angmom[3]; this.Lcom[1] = d.subtree_angmom[4]; this.Lcom[2] = d.subtree_angmom[5];
    // feet
    for (let f = 0; f < 2; f++) {
      const b = this.footBody[f], o = 9 * b, c = this.footCenterLocal;
      for (let k = 0; k < 3; k++) this.sole[f][k] = d.xpos[3 * b + k] + d.xmat[o + 3 * k] * c[0] + d.xmat[o + 3 * k + 1] * c[1] + d.xmat[o + 3 * k + 2] * c[2];
      this.footYaw[f] = Math.atan2(d.xmat[o + 3], d.xmat[o]);
      // lowest point of the two foot capsules (flat-floor contact test; allocation-free, no contact list needed)
      let minz = 1e9;
      for (const g of this.footGeoms[f]) {
        const hz = m.geom_size[3 * g + 1], r = m.geom_size[3 * g];
        const zc = d.geom_xpos[3 * g + 2], az = d.geom_xmat[9 * g + 8];
        minz = Math.min(minz, zc - Math.abs(az) * hz - r);
      }
      this.footMinZ[f] = minz;
      this.contact[f] = minz < 0.005;
    }
    const pb = idx.B.pelvis, tb = idx.B.torso;
    this.yawPelvis = Math.atan2(d.xmat[9 * pb + 3], d.xmat[9 * pb]);
    this.yawTorso = Math.atan2(d.xmat[9 * tb + 3], d.xmat[9 * tb]);
    this.tau.fill(0);
  }

  // tau += bias - spring - sum_i J_i^T W_i for feet with a wrench W_i = { f: [3] at this.sole[i], n: [3] }.
  addContactFeedforward(wrenches) {
    const { mj, m, d, nv } = this;
    for (let j = 6; j < nv; j++) this.tau[j] += d.qfrc_bias[j] - d.qfrc_spring[j];
    for (let j = 0; j < 6; j++) this.baseResidual[j] = d.qfrc_bias[j];
    for (let f = 0; f < 2; f++) {
      const W = wrenches[f]; if (!W) continue;
      mj.mj_jac(m, d, this.jp[f], this.jr[f], this.sole[f], this.footBody[f]);
      const Jp = this.jp[f].GetView(), Jr = this.jr[f].GetView();
      for (let j = 0; j < nv; j++) {
        const s = Jp[j] * W.f[0] + Jp[nv + j] * W.f[1] + Jp[2 * nv + j] * W.f[2] +
                  Jr[j] * W.n[0] + Jr[nv + j] * W.n[1] + Jr[2 * nv + j] * W.n[2];
        if (j >= 6) this.tau[j] -= s; else this.baseResidual[j] -= s;
      }
    }
  }

  pd(name, qref, kp, kd, qdref = 0) {
    const J = this.idx.J[name], d = this.d;
    this.tau[J.d] += kp * (qref - d.qpos[J.q]) + kd * (qdref - d.qvel[J.d]);
  }

  addTau(name, t) { this.tau[this.idx.J[name].d] += t; }

  // Write torques to the actuators (ctrl = tau / gear, clamped to [-1, 1]); returns the number saturated.
  writeCtrl() {
    const { d, idx } = this; let sat = 0;
    for (const n of idx.names) {
      const J = idx.J[n];
      let u = this.tau[J.d] / J.gear;
      if (u > 1) { u = 1; sat++; } else if (u < -1) { u = -1; sat++; }
      d.ctrl[J.a] = u;
    }
    return sat;
  }

  // Zero torque except joint damping (used when the robot has fallen).
  writeLimp(kd = 2) {
    const { d, idx } = this;
    for (const n of idx.names) {
      const J = idx.J[n];
      d.ctrl[J.a] = clamp(-kd * d.qvel[J.d] / J.gear, -1, 1);
    }
  }
}
