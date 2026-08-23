import { solveLinear } from './DiffIK.js';

/** Fingertip-retargeting differential IK for dexterous hands (the
 *  AnyTeleop / dex-retargeting formulation): five 3D fingertip-position
 *  tasks are stacked into one Jacobian over the hand's actuators and solved
 *  with damped least squares each frame, driving the hand's position
 *  actuators toward tracked-fingertip targets.
 *
 *  The solve runs in actuator space, so tendon-coupled joints (e.g. the
 *  Shadow Hand's J0 actuators, which drive the middle+distal joints
 *  together) are handled by summing the coupled joints' Jacobian columns.
 *
 *  Safeties mirror DiffIK: per-actuator velocity limit, a command leash
 *  around the measured actuator lengths (no windup when fingers are
 *  blocked), ctrlrange clipping, and a predictive ground-clearance gate on
 *  a shadow MjData so fingers can't be commanded into the floor even when
 *  the palm hovers just above it. */
export class HandRetarget {
  constructor(mujoco, model, opts) {
    this.mujoco = mujoco;
    this.model  = model;

    this.tipBodyIds  = [...opts.tipBodyIds]; // thumb, index, middle, ring, pinky
    this.actIds      = [...opts.actIds];
    this.damping     = opts.damping     ?? 1e-3;
    this.maxVel      = opts.maxVel      ?? 3.5;   // actuator-space rad/s
    // Finger position servos are weak (kp ~0.4-1.5), so a large command lead
    // winds up past the target before the measured tips catch up; keep the
    // leash short to bound the overshoot.
    this.leash       = opts.leash       ?? 0.3;   // rad, command lead over measured
    this.clearance   = opts.clearance   ?? 0.015; // m, bounding-sphere height
    this.floorZ      = opts.floorZ      ?? 0.0;

    // Per-actuator Jacobian columns: joint transmissions map to one dof,
    // fixed-tendon transmissions to the coupled joints' dofs with their
    // tendon coefficients.
    this.columns = this.actIds.map((a) => {
      if (model.actuator_trntype[a] == 0) { // mjTRN_JOINT
        return [{ dof: model.jnt_dofadr[model.actuator_trnid[2 * a]], coef: 1.0,
                  qposAdr: model.jnt_qposadr[model.actuator_trnid[2 * a]] }];
      }
      if (model.actuator_trntype[a] == 3) { // mjTRN_TENDON
        const t = model.actuator_trnid[2 * a];
        const out = [];
        for (let w = model.tendon_adr[t]; w < model.tendon_adr[t] + model.tendon_num[t]; w++) {
          if (model.wrap_type[w] == 1) { // mjWRAP_JOINT
            const j = model.wrap_objid[w];
            out.push({ dof: model.jnt_dofadr[j], coef: model.wrap_prm[w], qposAdr: model.jnt_qposadr[j] });
          }
        }
        return out;
      }
      return [];
    });

    // Ground-clearance guard: geoms on the hand's moving bodies.
    this.guardGeoms = [];
    const tipRoot = model.body_rootid[this.tipBodyIds[0]];
    for (let g = 0; g < model.ngeom; g++) {
      let b = model.geom_bodyid[g], joints = 0;
      if (model.body_rootid[b] != tipRoot) { continue; }
      for (let p = b; p != 0; p = model.body_parentid[p]) { joints += model.body_jntnum[p]; }
      if (joints > 0) { this.guardGeoms.push(g); }
    }

    this.shadow = new mujoco.MjData(model);
    const nv = model.nv;
    this.jacp = new mujoco.DoubleBuffer(3 * nv);
    this.jacr = new mujoco.DoubleBuffer(3 * nv);

    this.lastCommand = new Float64Array(this.actIds.length);
    this.commandInitialized = false;
    this.status = { groundLimited: false };
  }

  /** Re-seed the command trajectory from the current actuator lengths (call
   *  after externally resetting the simulation state). */
  reset() { this.commandInitialized = false; }

  /** Minimum bounding-sphere height above the floor with the candidate
   *  actuator commands applied kinematically (coupled joints split evenly). */
  minClearance(data, cmd) {
    const m = this.model, shadow = this.shadow;
    shadow.qpos.set(data.qpos);
    for (let k = 0; k < this.actIds.length; k++) {
      const cols = this.columns[k];
      let coefSum = 0;
      for (const c of cols) { coefSum += Math.abs(c.coef); }
      for (const c of cols) { shadow.qpos[c.qposAdr] = cmd[k] * (c.coef / Math.max(coefSum, 1e-9)); }
    }
    this.mujoco.mj_kinematics(m, shadow);
    let min = Infinity;
    for (const g of this.guardGeoms) {
      min = Math.min(min, shadow.geom_xpos[(g * 3) + 2] - m.geom_rbound[g] - this.floorZ);
    }
    return min;
  }

  /** One retarget cycle toward the five fingertip targets (MuJoCo world
   *  coordinates, ordered thumb/index/middle/ring/pinky). */
  step(data, targetsWorld, dt) {
    const m = this.model, mujoco = this.mujoco, n = this.actIds.length, nv = m.nv;
    const nTask = this.tipBodyIds.length * 3;

    // Stacked fingertip-position error and Jacobian (task rows x actuators).
    const err = new Array(nTask);
    const J = Array.from({ length: nTask }, () => new Array(n).fill(0));
    for (let f = 0; f < this.tipBodyIds.length; f++) {
      const b = this.tipBodyIds[f];
      for (let r = 0; r < 3; r++) { err[(f * 3) + r] = targetsWorld[f][r] - data.xpos[(b * 3) + r]; }
      mujoco.mj_jacBody(m, data, this.jacp, this.jacr, b);
      const jp = this.jacp.GetView();
      for (let k = 0; k < n; k++) {
        for (const c of this.columns[k]) {
          for (let r = 0; r < 3; r++) { J[(f * 3) + r][k] += c.coef * jp[(r * nv) + c.dof]; }
        }
      }
    }

    // Damped least squares in task space.
    const A = Array.from({ length: nTask }, (_, i) => Array.from({ length: nTask }, (_, j) => {
      let s = (i == j) ? this.damping : 0.0;
      for (let k = 0; k < n; k++) { s += J[i][k] * J[j][k]; }
      return s;
    }));
    const y = solveLinear(A, err);
    const dv = new Array(n).fill(0);
    for (let k = 0; k < n; k++) {
      for (let i = 0; i < nTask; i++) { dv[k] += J[i][k] * y[i]; }
    }

    // Velocity limit, then integrate the persistent command with a leash
    // around the measured actuator lengths.
    const maxStep = this.maxVel * Math.min(Math.max(dt, 1e-3), 0.1);
    const worst = Math.max(...dv.map(Math.abs));
    if (worst > maxStep) { for (let k = 0; k < n; k++) { dv[k] *= maxStep / worst; } }

    if (!this.commandInitialized) {
      for (let k = 0; k < n; k++) { this.lastCommand[k] = data.actuator_length[this.actIds[k]]; }
      this.commandInitialized = true;
    }
    const cmd = new Array(n);
    const buildCmd = (scale) => {
      for (let k = 0; k < n; k++) {
        const a = this.actIds[k], meas = data.actuator_length[a];
        let q = this.lastCommand[k] + dv[k] * scale;
        q = Math.min(Math.max(q, meas - this.leash), meas + this.leash);
        if (m.actuator_ctrlrange[2 * a] < m.actuator_ctrlrange[2 * a + 1]) {
          q = Math.min(Math.max(q, m.actuator_ctrlrange[2 * a]), m.actuator_ctrlrange[2 * a + 1]);
        }
        cmd[k] = q;
      }
    };

    this.status.groundLimited = false;
    const nowClearance = this.minClearance(data, [...this.lastCommand]);
    let accepted = false;
    for (const scale of [1.0, 0.5, 0.25]) {
      buildCmd(scale);
      const c = this.minClearance(data, cmd);
      if (c >= this.clearance || c >= nowClearance - 1e-6) { accepted = true; break; }
      this.status.groundLimited = true;
    }
    if (accepted) {
      for (let k = 0; k < n; k++) {
        data.ctrl[this.actIds[k]] = cmd[k];
        this.lastCommand[k] = cmd[k];
      }
    }
    return this.status;
  }

  dispose() {
    this.shadow.delete();
    this.jacp.delete();
    this.jacr.delete();
  }
}
