// Analytic (non-learned) stand + walk controller for the stock humanoid.xml (torque motors). No neural network, no QP.
//   STAND: DCM (capture point) feedback -> desired CoP -> two-foot contact wrench -> tau = bias - spring - sum J^T W + posture PD
//   WALK : gait clock; stance leg = J^T of a LIPM force through the COM (+height PD, +pelvis moment, +in-foot CoP modulation);
//          swing leg = DCM-offset foot placement on the ALIP state (angular momentum about the stance point),
//          analytic leg IK, joint PD with velocity feed-forward; velocity integrator; slew-limited vx/vy/wz command.
//   Transitions: rate-limited weight shift before the first step, stop only from a good foot configuration,
//          capture step from stand when the DCM leaves the support box, limp FALLEN state.
// Overview in README.md ("Analytic locomotion controller"); acceptance tests in test/controller.test.mjs, soak in test/soak.mjs.
import { Core } from './core.js';
import { makeIndex, clamp, wrap, cross, crouchPose, bendForComZ, legIK } from './humanoidModel.js';

export const DEFAULTS = {
  T: 0.33,              // step duration (s)
  swingH: 0.07,         // swing apex height (m)
  halfWidth: 0.10,      // nominal lateral foot offset from the COM path (m)
  comZ: 0.80,           // desired COM height above the stance sole while walking (m)
  kpZ: 150, kdZ: 25,    // COM height PD (1/s^2, 1/s), scaled by mass
  kpR: 400, kdR: 40,    // pelvis roll/pitch moment gains (N m/rad, N m s/rad)
  kpYaw: 120, kdYaw: 12,  // pelvis yaw gains (tuned up from 80/8: 24-seed stop/start soak 1 fall -> 0)
  pitchRef: 0.0,        // desired pelvis forward pitch (rad)
  kpSw: [300, 100, 400, 300], kdSw: [20, 8, 30, 12],   // swing hip_x, hip_z, hip_y, knee
  qdFF: 1.0, qdMax: 12, qdTau: -0.005 / Math.log(0.5),              // swing joint-velocity feed-forward from IK finite differences (filter tau, s)
  swingArrive: 0.85, swingDown: 0.95, pressV: 0.4,
  kpAnkSw: 15, kdAnkSw: 0.6,
  kpFlat: 20, flatMax: 6,                               // loaded-foot flattening PD (N m/rad) and its torque cap
  kdSt: [3, 3, 3, 3, 0.5, 0.5],                         // stance joint damping hip_x, hip_z, hip_y, knee, ankle_y, ankle_x
  couple: 1.0, coupleZ: 1.0,                            // SIMBICON-style cancellation of swing-hip torque through the stance hip (x/y, z)
  hipZMax: 0.4,                                         // swing hip_z (foot yaw) correction limit (rad)
  kpAbd: 200, kdAbd: 10, kpArm: 30, kdArm: 2,
  armSwing: 0.8, armPose: [0.65, -1.05, -0.6],          // [shoulder1, shoulder2, elbow]; arms hanging at the sides
  vGain: 0.05,                                          // extra Raibert gain on (v - v_cmd)
  accMax: 0.5, wAccMax: 1.5,                            // command slew limits (m/s^2, rad/s^2)
  vTau: -0.005 / Math.log(0.98), vKi: 1.0, vIntMax: 1.0, vIntLeak: 0.4, vFF: 1.0,   // velocity filter tau (s), integrator gain/clamp/bleed, feed-forward scale
  maxStep: 0.45, minSep: 0.13, maxSide: 0.40,
  copX: 0.01,                                           // nominal single-support CoP ahead of the ankle (m)
  ankleK: 1.5,                                          // single-support CoP modulation gain (DCM feedback), 0 = off
  copLimX: [0.04, 0.05], copLimY: 0.012,                // capsule-pair foot: only +-1.2 cm of lateral CoP authority
  kneeBend: 0.3,
  standKxi: 2.0, kpLegStand: 150, kdLegStand: 8, comAboveHip: 0.045, zRate: 0.4,
  autoStep: true, autoMargin: -0.01, autoHold: 1.2, Tcapture: 0.24,
  stopSpeed: 0.15, stopMaxStagger: 0.08, stopWidth: [0.16, 0.30],
  stopDcmX: [-0.045, 0.06],   // allowed DCM offset (back, fwd) from the stand target when stopping: the heel side has ~5 cm of CoP room, the toe side ~9 cm
  stopLead: 0.03,       // while stopping, land each foot this far behind the predicted DCM so the stand starts with the DCM over the toes
  standEntryTau: 0,     // >0: on entering stand, slide the DCM target from the current DCM to the nominal one (s). Off: it hurt the random stop/start soak (1 -> 5 falls in 40)
  shiftTime: 0.6, shiftSpeed: 0.25, shiftRest: 0.045, Tfirst: 0.30,   // weight shift before the first step
  tdEarly: 0.55,                                        // earliest phase at which swing contact ends the step
  fallHeadZ: 0.9,                                       // head below this (m) = fallen
  ANKLE_H: 0.107, L1: 0.38, L2: 0.33, EPS: 0.01,        // leg geometry (m)
};

const SIDE = ['right', 'left'];
const jn = (base, s) => base + '_' + SIDE[s];

/**
 * Usage (per physics step):  mj_step1(m, d);  ctrl.step(cmd);  mj_step2(m, d);
 * cmd = { walk, vx, vy, wz, height?, comOffset?: [x, y], arms?: [right|null, left|null] }   heading frame, SI units.
 */
export class AnalyticHumanoidController {
  constructor(mujoco, model, data, opts = {}) {
    this.mujoco = mujoco; this.m = model; this.d = data;
    this.P = { ...DEFAULTS, ...(opts.params || {}) };
    this.idx = makeIndex(mujoco, model);
    this.stockGear = {};
    for (const n of ['hip_x_right', 'hip_x_left']) this.stockGear[n] = model.actuator_gear[6 * this.idx.J[n].a];
    this.hipBoost = 1;
    this.setHipBoost(opts.hipBoost ?? 2);
    this.core = new Core(mujoco, model, data, this.idx);
    this.reset();
  }

  /** Scale the hip-abduction motors (1 = stock 40 N m). Restored by dispose(). */
  setHipBoost(k) {
    this.hipBoost = k;
    for (const n of ['hip_x_right', 'hip_x_left']) {
      const J = this.idx.J[n];
      this.m.actuator_gear[6 * J.a] = this.stockGear[n] * k;
      J.gear = this.m.actuator_gear[6 * J.a];
    }
  }

  /** Call after mj_resetData / keyframe load / teleport. */
  reset() {
    const P = this.P;
    const cp = crouchPose(P.kneeBend);
    const cp0 = [0, 0, cp.hip_y, cp.knee];
    this.S = {
      mode: 'stand', t: 0, tau: 0, stance: 1, yawDes: null, liftoff: [0, 0, 0], steps: 0, captureSteps: 0,
      vf: [0, 0], vint: [0, 0], vcmd: [0, 0], wz: 0, standRef: [cp0.slice(), cp0.slice()], standBend: P.kneeBend,
      zcmd: P.comZ, armOverride: null, standT: 0, autoUntil: -1, Tstep: P.T, target: [0, 0], standOff: [0, 0],
      qdf: [0, 0, 0, 0], ikq: [0, 0, 0, 0], prevIK: null, shiftT: 0, pendingWalk: false, fallen: false,
      sat: 0, wL: 0.5,
    };
    this.dbg = { xi: [0, 0], p: [0, 0], tgt: null };
  }

  dispose() {
    if (!this.core) return;
    this.core.dispose(); this.core = null;
    // leave the model as we found it (the model may outlive the controller if the user toggles it off)
    try { this.setHipBoost(1); } catch (e) { /* model already deleted */ }
  }

  get mode() { return this.S.fallen ? 'fallen' : this.S.mode; }
  yaw() { return this.core.yawPelvis; }

  /** Snapshot for HUD / markers (world coordinates, MuJoCo axes). */
  get info() {
    const S = this.S, c = this.core;
    return {
      mode: this.mode, stance: S.mode === 'walk' ? SIDE[S.stance] : 'both', phase: S.mode === 'walk' ? S.tau / S.Tstep : 0,
      steps: S.steps, captureSteps: S.captureSteps, fallen: S.fallen, com: c.com.slice(), vcom: c.vcom.slice(),
      dcm: this.dbg.xi ? this.dbg.xi.slice() : null, cop: this.dbg.p ? this.dbg.p.slice() : null,
      target: S.mode === 'walk' && this.dbg.tgt ? this.dbg.tgt.slice() : null,
      vFiltered: S.vf.slice(), satCount: S.sat, hipBoost: this.hipBoost,
    };
  }

  step(cmd = {}) {
    const { m, d, S, P, core } = this;
    core.update();
    const headZ = d.xpos[3 * this.idx.B.head + 2];
    if (S.fallen || !(headZ > P.fallHeadZ) || !isFinite(d.qpos[2])) {
      S.fallen = true;
      core.writeLimp();
      return;
    }
    S.armOverride = cmd.arms || null;
    if (S.yawDes === null) S.yawDes = core.yawPelvis;
    S.t += m.opt.timestep;
    if (S.mode === 'stand') {
      if (cmd.walk && !S.pendingWalk) { S.pendingWalk = true; S.shiftT = 0; S.stance = S.wL >= 0.5 ? 1 : 0; }   // first stance = more loaded foot
      if (!cmd.walk) S.pendingWalk = false;
      this._standStep(cmd);
    } else {
      this._walkStep(cmd.walk ? cmd : { walk: false, vx: 0, vy: 0, wz: 0 });
    }
    S.sat = core.writeCtrl();
  }

  // ---------------------------------------------------------------- helpers
  _anklePos(s) { const j = this.idx.J[jn('ankle_y', s)].j, x = this.d.xanchor; return [x[3 * j], x[3 * j + 1], x[3 * j + 2]]; }

  // stance point: ankle projected onto the sole, shifted forward by copX along the foot x-axis
  _stancePoint(s) {
    const a = this._anklePos(s), o = 9 * this.core.footBody[s], xm = this.d.xmat, z = this.core.footMinZ[s];
    const fx = xm[o], fy = xm[o + 3], n = Math.hypot(fx, fy) || 1;
    return [a[0] + this.P.copX * fx / n, a[1] + this.P.copX * fy / n, z > 0.02 ? 0 : Math.max(0, z)];
  }

  _upperBody() {
    const { core, d, idx, P, S } = this;
    for (const n of ['abdomen_z', 'abdomen_y', 'abdomen_x']) core.pd(n, 0, P.kpAbd, P.kdAbd);
    // arms: pose + counter-swing with the opposite leg
    const hyR = d.qpos[idx.J.hip_y_right.q], hyL = d.qpos[idx.J.hip_y_left.q];
    for (let s = 0; s < 2; s++) {
      const A = (S.armOverride && S.armOverride[s]) || P.armPose;
      const sw = S.mode === 'walk' ? P.armSwing * 0.5 * (s === 0 ? (hyR - hyL) : (hyL - hyR)) : 0;
      core.pd(jn('shoulder1', s), A[0], P.kpArm, P.kdArm);
      core.pd(jn('shoulder2', s), A[1] + sw, P.kpArm, P.kdArm);
      core.pd(jn('elbow', s), A[2], P.kpArm, P.kdArm);
    }
  }

  // weak restoring torque that keeps a loaded foot flat (zero when flat, so it does not disturb the CoP law)
  _footFlat(s) {
    const { P, d, core, idx } = this;
    if (P.kpFlat <= 0) return;
    const fo = 9 * core.footBody[s];
    const pitch = -Math.asin(clamp(d.xmat[fo + 6], -1, 1)), roll = Math.asin(clamp(d.xmat[fo + 7], -1, 1));
    core.tau[idx.J[jn('ankle_y', s)].d] += clamp(-P.kpFlat * pitch, -P.flatMax, P.flatMax);
    core.tau[idx.J[jn('ankle_x', s)].d] += clamp((s === 0 ? -1 : 1) * P.kpFlat * roll, -P.flatMax, P.flatMax);
  }

  // world-frame restoring moment on the pelvis: roll/pitch to upright (+pitchRef), yaw to yawDes
  _pelvisMoment(yawDes) {
    const { d, P, core } = this, pelvis = this.idx.B.pelvis, o = 9 * pelvis;
    const z = [d.xmat[o + 2], d.xmat[o + 5], d.xmat[o + 8]];
    const yaw = core.yawPelvis, cy = Math.cos(yaw), sy = Math.sin(yaw);
    const sp = Math.sin(P.pitchRef);
    const zd = [sp * cy, sp * sy, Math.sqrt(Math.max(0.1, 1 - sp * sp))];
    const e = cross(z, zd);
    const w = [d.cvel[6 * pelvis], d.cvel[6 * pelvis + 1], d.cvel[6 * pelvis + 2]];
    return [P.kpR * e[0] - P.kdR * w[0], P.kpR * e[1] - P.kdR * w[1], P.kpYaw * wrap(yawDes - yaw) - P.kdYaw * w[2]];
  }

  _enterStand() {
    const { S, d, idx, core, P } = this;
    S.mode = 'stand'; S.pendingWalk = false; S.standT = 0; S.vint[0] = S.vint[1] = 0;
    for (let s = 0; s < 2; s++) S.standRef[s] = ['hip_x', 'hip_z', 'hip_y', 'knee'].map(n => d.qpos[idx.J[jn(n, s)].q]);
    S.standBend = bendForComZ(core.com[2], P);
    S.zcmd = core.com[2];
    // soft entry: DCM target starts at the current DCM (clamped to the box) and slides to the nominal target
    const c = core.com, v = core.vcom, w0 = Math.sqrt(core.g / Math.max(0.3, c[2]));
    const sp0 = this._stancePoint(0), sp1 = this._stancePoint(1), yaw = core.yawPelvis, cy = Math.cos(yaw), sy = Math.sin(yaw);
    const ox = c[0] + v[0] / w0 - (0.5 * (sp0[0] + sp1[0]) + 0.02 * cy), oy = c[1] + v[1] / w0 - (0.5 * (sp0[1] + sp1[1]) + 0.02 * sy);
    const hx = clamp(ox * cy + oy * sy, -0.04, 0.06), hy = clamp(-ox * sy + oy * cy, -0.05, 0.05);
    S.standOff = P.standEntryTau > 0 ? [hx * cy - hy * sy, hx * sy + hy * cy] : [0, 0];
  }

  _beginStep(stance, Tstep = this.P.T) {
    const S = this.S;
    S.mode = 'walk'; S.stance = stance; S.tau = 0; S.steps++; S.Tstep = Tstep;
    const a = this._anklePos(1 - stance);
    S.liftoff = [a[0], a[1], a[2]];
  }

  // ---------------------------------------------------------------- STAND
  _standStep(cmd) {
    const { m, P, S, core, dbg } = this, dt = m.opt.timestep;
    const c = core.com, v = core.vcom, M = core.mass, g = core.g;
    const sp = [this._stancePoint(0), this._stancePoint(1)];
    const mid = [0.5 * (sp[0][0] + sp[1][0]), 0.5 * (sp[0][1] + sp[1][1])];
    const w0 = Math.sqrt(g / Math.max(0.3, c[2]));
    const xi = [c[0] + v[0] / w0, c[1] + v[1] / w0];          // capture point / DCM
    const yaw = core.yawPelvis, cy = Math.cos(yaw), sy = Math.sin(yaw);
    let xid = [mid[0] + 0.02 * cy, mid[1] + 0.02 * sy];
    if (cmd.comOffset) {   // behaviour hook: desired DCM offset (heading frame) for sway / lean / dance
      // keep the requested offset inside the support the feet actually provide (a narrow stance after walking would
      // otherwise push the DCM out of the box and trigger capture steps)
      const latLim = Math.max(0, 0.5 * Math.hypot(sp[1][0] - sp[0][0], sp[1][1] - sp[0][1]) - 0.035);
      const ox = clamp(cmd.comOffset[0], -0.04, 0.05), oy = clamp(cmd.comOffset[1], -latLim, latLim);
      xid = [xid[0] + ox * cy - oy * sy, xid[1] + ox * sy + oy * cy];
    }
    if (P.standEntryTau > 0 && (S.standOff[0] || S.standOff[1])) {
      const k = Math.exp(-dt / P.standEntryTau);
      S.standOff[0] *= k; S.standOff[1] *= k;
      xid = [xid[0] + S.standOff[0], xid[1] + S.standOff[1]];
    }
    if (S.pendingWalk) {
      // weight shift: slide the DCM target toward the first stance foot at a limited speed, stopping shiftRest short of it
      const st = sp[S.stance];
      const full = Math.hypot(mid[0] - st[0], mid[1] - st[1]);
      const frac = clamp(Math.min(S.shiftT * P.shiftSpeed, Math.max(0, full - P.shiftRest)) / Math.max(full, 1e-6), 0, 1);
      xid = [mid[0] + frac * (st[0] - mid[0]), mid[1] + frac * (st[1] - mid[1])];
    }
    let p = [xi[0] + P.standKxi * (xi[0] - xid[0]), xi[1] + P.standKxi * (xi[1] - xid[1])];
    const hy = Math.hypot(sp[1][0] - sp[0][0], sp[1][1] - sp[0][1]) / 2 + 0.012, hx = 0.07;
    let ex = (p[0] - mid[0]) * cy + (p[1] - mid[1]) * sy, ey = -(p[0] - mid[0]) * sy + (p[1] - mid[1]) * cy;
    // DCM relative to the support box (heading frame): > 0 means the ankle strategy cannot hold it
    const dxi = [(xi[0] - mid[0]) * cy + (xi[1] - mid[1]) * sy - 0.02, -(xi[0] - mid[0]) * sy + (xi[1] - mid[1]) * cy];
    const over = Math.max(Math.abs(dxi[0]) - hx, Math.abs(dxi[1]) - hy);
    dbg.dxi = dxi; dbg.box = [hx, hy]; dbg.over = over;
    const eyRaw = ey;
    ex = clamp(ex, -hx + 0.02, hx + 0.02); ey = clamp(ey, -hy, hy);
    p = [mid[0] + ex * cy - ey * sy, mid[1] + ex * sy + ey * cy];
    S.standT += dt;
    if (P.autoStep && S.standT > 0.3 && over > P.autoMargin && core.contact[0] && core.contact[1]) {
      // capture step: swing the foot on the side the DCM is escaping to (lateral), else the less loaded foot
      S.autoUntil = S.t + P.autoHold;
      const stance = Math.abs(eyRaw) > 0.5 * hy ? (eyRaw > 0 ? 0 : 1) : (S.wL > 0.5 ? 1 : 0);
      S.vint[0] = S.vint[1] = 0; S.vcmd[0] = S.vcmd[1] = 0;
      this._beginStep(stance, P.Tcapture); S.captureSteps++;
      return this._walkStep({ vx: 0, vy: 0, wz: 0 });
    }
    S.zcmd += clamp((cmd.height ?? P.comZ) - S.zcmd, -P.zRate * dt, P.zRate * dt);
    const zdes = S.zcmd;
    const Fz = M * (g + P.kpZ * (zdes - c[2]) - P.kdZ * v[2]);
    const F = [M * w0 * w0 * (c[0] - p[0]), M * w0 * w0 * (c[1] - p[1]), Fz];
    const dxy = [sp[1][0] - sp[0][0], sp[1][1] - sp[0][1]], L2 = dxy[0] * dxy[0] + dxy[1] * dxy[1];
    const wL = clamp(((p[0] - sp[0][0]) * dxy[0] + (p[1] - sp[0][1]) * dxy[1]) / Math.max(L2, 1e-6), 0.02, 0.98);
    S.wL = wL;
    const w = [1 - wL, wL], W = [null, null];
    // stand: posture PD holds roll/pitch; only yaw goes through the feet. n_total = T - sum r_i x f_i
    const nt = [0, 0, this._pelvisMoment(S.yawDes)[2]];
    const fs = [];
    for (let s = 0; s < 2; s++) {
      const f = [w[s] * F[0], w[s] * F[1], w[s] * F[2]]; fs.push(f);
      const mm = cross([sp[s][0] - c[0], sp[s][1] - c[1], sp[s][2] - c[2]], f);
      nt[0] -= mm[0]; nt[1] -= mm[1]; nt[2] -= mm[2];
    }
    for (let s = 0; s < 2; s++) {
      const fz = fs[s][2];
      // foot moment limits about the stance point: lateral +-1.2 cm, CoP forward <= 9 cm, backward <= 6 cm
      W[s] = { f: fs[s], n: [clamp(w[s] * nt[0], -0.012 * fz, 0.012 * fz), clamp(w[s] * nt[1], -0.09 * fz, 0.06 * fz),
                             clamp(w[s] * nt[2], -0.04 * fz, 0.04 * fz)] };
      core.sole[s][0] = sp[s][0]; core.sole[s][1] = sp[s][1]; core.sole[s][2] = sp[s][2];
    }
    core.addContactFeedforward(W);
    // leg posture: reference captured at stand entry, shifted by the crouch change needed for the height command
    const cNow = crouchPose(bendForComZ(zdes, P)), c0 = crouchPose(S.standBend);
    for (let s = 0; s < 2; s++) {
      const R = S.standRef[s];
      core.pd(jn('hip_x', s), R[0], P.kpLegStand, P.kdLegStand);
      core.pd(jn('hip_z', s), R[1], P.kpLegStand, P.kdLegStand);
      core.pd(jn('hip_y', s), R[2] + (cNow.hip_y - c0.hip_y), P.kpLegStand, P.kdLegStand);
      core.pd(jn('knee', s), R[3] + (cNow.knee - c0.knee), P.kpLegStand, P.kdLegStand);
      core.pd(jn('ankle_y', s), 0, 0, 1.0);   // ankles: damping only, the ankle torque is the CoP actuator
      core.pd(jn('ankle_x', s), 0, 0, 1.0);
      this._footFlat(s);
    }
    this._upperBody();
    dbg.xi = xi; dbg.p = p; dbg.tgt = null;
    if (S.pendingWalk) {
      S.shiftT += dt;
      const st = sp[S.stance];
      // lift off once the DCM is near the stance point (or on timeout)
      if (S.shiftT > P.shiftTime || Math.hypot(xi[0] - st[0], xi[1] - st[1]) < P.shiftRest + 0.015) this._beginStep(S.stance, P.Tfirst);
    }
  }

  // ---------------------------------------------------------------- WALK
  _walkStep(cmd) {
    const { m, d, P, S, core, idx, dbg } = this, dt = m.opt.timestep;
    const c = core.com, v = core.vcom, M = core.mass, g = core.g;
    const st = S.stance, sw = 1 - st;
    S.tau += dt;
    const phi = S.tau / S.Tstep;
    // heading command
    S.wz += clamp((cmd.wz || 0) - S.wz, -P.wAccMax * dt, P.wAccMax * dt);
    S.yawDes = wrap(S.yawDes + S.wz * dt);
    const cy = Math.cos(S.yawDes), sy = Math.sin(S.yawDes);
    const toH = (x, y) => [x * cy + y * sy, -x * sy + y * cy];
    const toW = (x, y) => [x * cy - y * sy, x * sy + y * cy];

    // ---------- stance leg: LIPM force through the COM + height PD + pelvis moment, realised by J^T
    const sp = this._stancePoint(st);
    const h = Math.max(0.4, c[2] - sp[2]);
    const w0 = Math.sqrt(g / h);
    const Fz = M * (g + P.kpZ * (P.comZ - h) - P.kdZ * v[2]);
    const eT = Math.exp(w0 * S.Tstep);
    // ALIP: angular momentum about the stance point, L = Lcom + M r x v  ->  v_eff = (L_y, -L_x) / (M h)
    const r3 = [c[0] - sp[0], c[1] - sp[1], h];
    const Lx = core.Lcom[0] + M * (r3[1] * v[2] - r3[2] * v[1]), Ly = core.Lcom[1] + M * (r3[2] * v[0] - r3[0] * v[2]);
    // in-foot CoP modulation (ankle strategy): CoP = sp + clamp(ankleK (xi - xi_nom(tau)))
    let px = sp[0], py = sp[1];
    if (P.ankleK > 0) {
      const xw = [r3[0] + Ly / (M * h) / w0, r3[1] - Lx / (M * h) / w0];
      const [xhx, xhy] = toH(xw[0], xw[1]);
      const grow = Math.exp(w0 * Math.min(S.tau, S.Tstep));
      const sgnS = st === 1 ? 1 : -1;
      // the nominal orbit uses the SAME offset as the foot-placement law (command + integrator), else they fight
      const bnx = (P.vFF * S.vcmd[0] + S.vint[0]) * S.Tstep / (eT - 1) + (cmd.walk ? 0 : P.stopLead);
      const bny = -sgnS * (2 * P.halfWidth) / (1 + eT) + (P.vFF * S.vcmd[1] + S.vint[1]) * S.Tstep / (eT - 1);
      const dpx = clamp(P.ankleK * (xhx - bnx * grow), -P.copLimX[0], P.copLimX[1]);
      const dpy = clamp(P.ankleK * (xhy - bny * grow), -P.copLimY, P.copLimY);
      const dw = toW(dpx, dpy); px += dw[0]; py += dw[1];
    }
    const Tm = this._pelvisMoment(S.yawDes);
    let fx = Fz * (c[0] - px) / h, fy = Fz * (c[1] - py) / h;
    fx += -Tm[1] / h; fy += Tm[0] / h;   // pelvis moment realised by shifting the force's line of action
    const fmax = 0.7 * Fz, fn = Math.hypot(fx, fy); if (fn > fmax) { fx *= fmax / fn; fy *= fmax / fn; }
    const W = [null, null];
    W[st] = { f: [fx, fy, Math.max(0, Fz)], n: [0, 0, clamp(Tm[2], -10, 10)] };
    core.sole[st][0] = px; core.sole[st][1] = py; core.sole[st][2] = sp[2];
    core.addContactFeedforward(W);
    const stN = ['hip_x', 'hip_z', 'hip_y', 'knee', 'ankle_y', 'ankle_x'];
    for (let k = 0; k < 6; k++) core.pd(jn(stN[k], st), 0, 0, P.kdSt[k]);
    this._footFlat(st);

    // ---------- foot placement: DCM-offset law in the heading frame
    //   xi_td = (xi - p_s) e^{w0 t_rem};  target = xi_td - b,  b = [ v T/(e^{w0 T}-1),  -sgnNew 2a/(1+e^{w0 T}) + vy T/(e^{w0 T}-1) ]
    const trem = Math.max(0, S.Tstep * P.swingDown - S.tau);
    const [cxh, cyh] = toH(c[0] - sp[0], c[1] - sp[1]);
    const [vxh, vyh] = toH(v[0], v[1]);
    const er = Math.exp(w0 * trem);
    const [vex, vey] = toH(Ly / (M * h), -Lx / (M * h));
    const xih = [cxh + vex / w0, cyh + vey / w0];
    const xtd = xih[0] * er, ytd = xih[1] * er;
    S.vcmd[0] += clamp((cmd.vx || 0) - S.vcmd[0], -P.accMax * dt, P.accMax * dt);
    S.vcmd[1] += clamp((cmd.vy || 0) - S.vcmd[1], -P.accMax * dt, P.accMax * dt);
    const vxd = S.vcmd[0], vyd = S.vcmd[1];
    const sgnNew = sw === 1 ? 1 : -1;   // new stance foot side (+1 = left = +y)
    const aV = 1 - Math.exp(-dt / P.vTau);
    S.vf[0] += aV * (vxh - S.vf[0]); S.vf[1] += aV * (vyh - S.vf[1]);
    S.vint[0] = clamp(S.vint[0] + P.vKi * (vxd - S.vf[0]) * dt, -P.vIntMax, P.vIntMax);
    S.vint[1] = clamp(S.vint[1] + P.vKi * (vyd - S.vf[1]) * dt, -P.vIntMax, P.vIntMax);
    if (!cmd.walk) { const k = Math.exp(-dt / P.vIntLeak); S.vint[0] *= k; S.vint[1] *= k; }   // bleed while stopping
    const bx = (P.vFF * vxd + S.vint[0]) * S.Tstep / (eT - 1) + (cmd.walk ? 0 : P.stopLead);
    const by = -sgnNew * (2 * P.halfWidth) / (1 + eT) + (P.vFF * vyd + S.vint[1]) * S.Tstep / (eT - 1);
    let tx = xtd - bx + P.vGain * (S.vf[0] - vxd), ty = ytd - by + P.vGain * (S.vf[1] - vyd);
    tx = clamp(tx, -P.maxStep, P.maxStep);
    ty = sgnNew > 0 ? clamp(ty, P.minSep, P.maxSide) : clamp(ty, -P.maxSide, -P.minSep);
    let tgt = toW(tx, ty); tgt = [sp[0] + tgt[0], sp[1] + tgt[1]];
    S.target = tgt;

    // ---------- swing foot trajectory (ankle pivot) -> analytic IK in the pelvis frame -> joint PD
    const s = clamp(phi / P.swingArrive, 0, 1), sm = s * s * (3 - 2 * s);
    const zoff = phi > P.swingDown ? -P.pressV * (S.tau - P.swingDown * S.Tstep)   // press down until contact
                                   : P.swingH * Math.sin(Math.PI * clamp(phi / P.swingDown, 0, 1));
    const zbase = (1 - sm) * Math.max(S.liftoff[2], P.ANKLE_H) + sm * P.ANKLE_H;
    const ades = [S.liftoff[0] + (tgt[0] - S.liftoff[0]) * sm, S.liftoff[1] + (tgt[1] - S.liftoff[1]) * sm, zbase + zoff];
    const hb = sw === 0 ? idx.B.thigh_right : idx.B.thigh_left, o = 9 * idx.B.pelvis;
    const dv = [ades[0] - d.xpos[3 * hb], ades[1] - d.xpos[3 * hb + 1], ades[2] - d.xpos[3 * hb + 2]];
    const vp = [d.xmat[o] * dv[0] + d.xmat[o + 3] * dv[1] + d.xmat[o + 6] * dv[2],
                d.xmat[o + 1] * dv[0] + d.xmat[o + 4] * dv[1] + d.xmat[o + 7] * dv[2],
                d.xmat[o + 2] * dv[0] + d.xmat[o + 5] * dv[1] + d.xmat[o + 8] * dv[2]];
    const qzW = clamp(wrap(S.yawDes - core.yawPelvis), -P.hipZMax, P.hipZMax);
    const ik = legIK(vp, sw, sw === 0 ? qzW : -qzW, P);
    const swN = ['hip_x', 'hip_z', 'hip_y', 'knee'];
    const tauSw = [0, 0, 0, 0];
    const aQd = 1 - Math.exp(-dt / P.qdTau);
    const sameLeg = S.prevIK && S.prevIK.sw === sw;
    for (let k = 0; k < 4; k++) {
      const J = idx.J[jn(swN[k], sw)];
      const qref = clamp(ik[swN[k]], J.lo + 0.02, J.hi - 0.02);
      const qd = sameLeg ? clamp((qref - S.prevIK.q[k]) / dt, -P.qdMax, P.qdMax) : 0;
      S.qdf[k] = sameLeg ? S.qdf[k] + aQd * (qd - S.qdf[k]) : 0;
      S.ikq[k] = qref;
      const t = P.kpSw[k] * (qref - d.qpos[J.q]) + P.kdSw[k] * (P.qdFF * S.qdf[k] - d.qvel[J.d]);
      tauSw[k] = t; core.tau[J.d] += t;
    }
    S.prevIK = { sw, q: S.ikq.slice() };
    if (P.couple > 0) {   // SIMBICON coupling: cancel the swing-hip reaction on the pelvis through the stance hip
      core.addTau(jn('hip_x', st), P.couple * tauSw[0]);
      core.addTau(jn('hip_z', st), P.coupleZ * tauSw[1]);
      core.addTau(jn('hip_y', st), -P.couple * tauSw[2]);
    }
    { // swing ankle: keep the foot level in the world so it lands flat
      const fo = 9 * core.footBody[sw];
      const pitch = -Math.asin(clamp(d.xmat[fo + 6], -1, 1)), roll = Math.asin(clamp(d.xmat[fo + 7], -1, 1));
      const Jy = idx.J[jn('ankle_y', sw)], Jx = idx.J[jn('ankle_x', sw)];
      core.tau[Jy.d] += -P.kpAnkSw * pitch - P.kdAnkSw * d.qvel[Jy.d];
      core.tau[Jx.d] += -(sw === 0 ? 1 : -1) * P.kpAnkSw * roll - P.kdAnkSw * d.qvel[Jx.d];
    }
    this._upperBody();
    dbg.xi = [sp[0] + toW(xih[0], xih[1])[0], sp[1] + toW(xih[0], xih[1])[1]];
    dbg.p = [px, py]; dbg.tgt = tgt;

    // ---------- step transition
    if ((phi >= P.tdEarly && core.contact[sw]) || phi > 1.6) {
      const a = this._anklePos(sw);
      const [sx, sy2] = toH(a[0] - sp[0], a[1] - sp[1]);
      const wantStop = !cmd.walk && S.t > S.autoUntil;
      const slow = Math.hypot(S.vf[0], S.vf[1]) < P.stopSpeed && Math.hypot(S.vcmd[0], S.vcmd[1]) < 0.05 && Math.abs(S.wz) < 0.1;
      // settle into stand only with the feet side by side, 16-30 cm apart ...
      const goodFeet = Math.abs(sx) < P.stopMaxStagger && Math.abs(sy2) > P.stopWidth[0] && Math.abs(sy2) < P.stopWidth[1];
      // ... and with the DCM already inside the stand-mode CoP box of the two feet
      // (measured with the COM-velocity DCM, i.e. the estimate the stand controller will use from the next tick on)
      const xiV = [cxh + vxh / w0, cyh + vyh / w0];
      const dmx = xiV[0] - 0.5 * sx - 0.02;
      const goodDcm = dmx > P.stopDcmX[0] && dmx < P.stopDcmX[1] && Math.abs(xiV[1] - 0.5 * sy2) < 0.5 * Math.abs(sy2);
      if (wantStop && slow && goodFeet && goodDcm) this._enterStand();
      else this._beginStep(sw);
    }
  }
}
