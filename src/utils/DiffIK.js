/** Differential inverse kinematics (damped least squares) that drives a
 *  MuJoCo arm through its position actuators, following the approach of
 *  dm_control / mjctrl: per cycle, solve J dq = error for the end-effector
 *  site, integrate one bounded step from the measured joint positions, and
 *  hand the result to the actuators as position targets.
 *
 *  Because the intent is to eventually stream these commands to a real arm,
 *  safety is enforced on the COMMANDS before they reach the actuators, not
 *  by relying on simulated contacts:
 *   1. Workspace clamp: the task-space target is clamped above the floor and
 *      inside a reach sphere.
 *   2. Joint-velocity limit: each command step is bounded to maxJointVel.
 *   3. Joint-range / ctrl-range clipping.
 *   4. Predictive ground clearance: every candidate command is evaluated
 *      kinematically on a shadow MjData, and rejected (or scaled down) if
 *      any moving arm geom's bounding sphere would come within `clearance`
 *      of the ground plane. Bounding spheres make the check conservative.
 *
 *  This class is renderer-agnostic (no three.js): positions/quaternions are
 *  plain arrays in MuJoCo world coordinates.
 */
export class DiffIK {
  constructor(mujoco, model, opts = {}) {
    this.mujoco = mujoco;
    this.model  = model;

    this.siteId      = opts.siteId;
    this.damping     = opts.damping     ?? 1e-4;  // DLS regularization (task space)
    this.maxJointVel = opts.maxJointVel ?? 2.0;   // rad/s per joint
    this.leash       = opts.leash       ?? 0.2;   // rad, max command lead over measured qpos
    this.nullGain    = opts.nullGain    ?? 0.3;   // posture task gain (rad/s per rad)
    // Commands are gated at `clearance`; the margin also absorbs the real
    // arm's gravity sag below its position commands (~10mm at stock gains).
    this.clearance   = opts.clearance   ?? 0.020; // m, min bounding-sphere height
    this.floorZ      = opts.floorZ      ?? 0.0;
    this.minTargetZ  = opts.minTargetZ  ?? 0.05;  // m above floor for the TCP target
    this.reachCenter = opts.reachCenter ?? [0, 0, 0.5];
    this.reachRadius = opts.reachRadius ?? 0.9;

    // Arm actuators: explicit list from the caller (e.g. the joint chain
    // from the world to the TCP site), else every joint-transmission
    // actuator in the model (mjTRN_JOINT = 0).
    this.armActIds = opts.armActIds ? [...opts.armActIds] : [];
    if (this.armActIds.length == 0) {
      for (let a = 0; a < model.nu; a++) {
        if (model.actuator_trntype[a] == 0) { this.armActIds.push(a); }
      }
    }
    this.armJointIds = this.armActIds.map((a) => model.actuator_trnid[2 * a]);
    this.armQposAdr  = this.armJointIds.map((j) => model.jnt_qposadr[j]);
    this.armDofAdr   = this.armJointIds.map((j) => model.jnt_dofadr[j]);
    this.narm = this.armActIds.length;

    // Geoms whose ground clearance we guard: every geom on a body that can
    // move (has a joint somewhere between it and the world). The static base
    // is excluded; commands can't move it.
    this.guardGeoms = [];
    for (let g = 0; g < model.ngeom; g++) {
      let b = model.geom_bodyid[g], joints = 0;
      const tcpBody = model.site_bodyid[this.siteId];
      if (model.body_rootid[b] != model.body_rootid[tcpBody]) { continue; }
      for (let p = b; p != 0; p = model.body_parentid[p]) { joints += model.body_jntnum[p]; }
      if (joints > 0) { this.guardGeoms.push(g); }
    }

    // Rest posture for the nullspace task: the model's first keyframe if it
    // has one (e.g. the xArm7 "home" pose), else zeros. Redundant arms need
    // this to resolve the elbow and to escape folded poses near joint
    // limits, where the task-space gradient alone stalls.
    this.restPose = this.armQposAdr.map((adr) => model.nkey > 0 ? model.key_qpos[adr] : 0.0);

    // Shadow data for predictive kinematic checks, and reusable out-buffers.
    this.shadow = new mujoco.MjData(model);
    const nv = model.nv;
    this.jacp  = new mujoco.DoubleBuffer(3 * nv);
    this.jacr  = new mujoco.DoubleBuffer(3 * nv);
    this.quatB = new mujoco.DoubleBuffer(4);
    this.negqB = new mujoco.DoubleBuffer(4);
    this.errqB = new mujoco.DoubleBuffer(4);
    this.errvB = new mujoco.DoubleBuffer(3);

    // The last joint-position command accepted by the safety gate: this is
    // the exact vector you would stream to a real arm's position interface.
    // The command trajectory is integrated persistently (so tracking can run
    // at maxJointVel) but leashed to the measured joint positions, which
    // bounds the position-servo error and prevents windup when the arm is
    // physically blocked.
    this.lastCommand = new Float64Array(this.narm);
    this.commandInitialized = false;
    this.status = { targetClamped: false, groundLimited: false };
  }

  /** Clamp a task-space target (in place) above the floor and into reach. */
  clampTarget(t) {
    this.status.targetClamped = false;
    const zMin = this.floorZ + this.minTargetZ;
    if (t[2] < zMin) { t[2] = zMin; this.status.targetClamped = true; }
    const c = this.reachCenter;
    const d = Math.hypot(t[0] - c[0], t[1] - c[1], t[2] - c[2]);
    if (d > this.reachRadius) {
      const s = this.reachRadius / d;
      t[0] = c[0] + (t[0] - c[0]) * s;
      t[1] = c[1] + (t[1] - c[1]) * s;
      t[2] = c[2] + (t[2] - c[2]) * s;
      this.status.targetClamped = true;
    }
    return t;
  }

  /** Minimum height (above floorZ) of any guarded geom's bounding-sphere
   *  bottom, for the given arm command evaluated kinematically. */
  minClearance(data, cmd) {
    const m = this.model, shadow = this.shadow;
    shadow.qpos.set(data.qpos);
    for (let i = 0; i < this.narm; i++) { shadow.qpos[this.armQposAdr[i]] = cmd[i]; }
    this.mujoco.mj_kinematics(m, shadow);
    let min = Infinity;
    for (const g of this.guardGeoms) {
      min = Math.min(min, shadow.geom_xpos[(g * 3) + 2] - m.geom_rbound[g] - this.floorZ);
    }
    return min;
  }

  /** One IK cycle: solve toward (targetPos, targetQuat) — both in MuJoCo
   *  world coordinates — and write safety-gated position commands into
   *  data.ctrl for the arm actuators. dt is the elapsed time in seconds. */
  step(data, targetPos, targetQuat, dt) {
    const m = this.model, mujoco = this.mujoco, n = this.narm, nv = m.nv;

    // 6D task-space error: position, then orientation as a rotation vector.
    const sadr = this.siteId * 3;
    const err = [
      targetPos[0] - data.site_xpos[sadr + 0],
      targetPos[1] - data.site_xpos[sadr + 1],
      targetPos[2] - data.site_xpos[sadr + 2],
      0, 0, 0];
    mujoco.mju_mat2Quat(this.quatB, [...data.site_xmat.slice(this.siteId * 9, this.siteId * 9 + 9)]);
    mujoco.mju_negQuat(this.negqB, [...this.quatB.GetView()]);
    mujoco.mju_mulQuat(this.errqB, [...targetQuat], [...this.negqB.GetView()]);
    mujoco.mju_quat2Vel(this.errvB, [...this.errqB.GetView()], 1.0);
    const ev = this.errvB.GetView();
    err[3] = ev[0]; err[4] = ev[1]; err[5] = ev[2];

    // 6 x narm Jacobian (arm dof columns only).
    mujoco.mj_jacSite(m, data, this.jacp, this.jacr, this.siteId);
    const jp = this.jacp.GetView(), jr = this.jacr.GetView();
    const J = [];
    for (let r = 0; r < 3; r++) { J.push(this.armDofAdr.map((d) => jp[r * nv + d])); }
    for (let r = 0; r < 3; r++) { J.push(this.armDofAdr.map((d) => jr[r * nv + d])); }

    // Damped least squares: dq = J^T (J J^T + lambda I)^-1 err.
    const A = [];
    for (let i = 0; i < 6; i++) {
      A.push([]);
      for (let j = 0; j < 6; j++) {
        let s = (i == j) ? this.damping : 0.0;
        for (let k = 0; k < n; k++) { s += J[i][k] * J[j][k]; }
        A[i].push(s);
      }
    }
    const y = solveLinear(A, err);
    const dq = new Array(n).fill(0);
    for (let k = 0; k < n; k++) {
      for (let i = 0; i < 6; i++) { dq[k] += J[i][k] * y[i]; }
    }

    // Nullspace posture task: pull toward the rest pose without disturbing
    // the end-effector: dq += (I - J^+ J) * nullGain * (q_rest - q).
    const dqn = this.restPose.map((q0, k) => this.nullGain * (q0 - data.qpos[this.armQposAdr[k]]));
    const Jdqn = new Array(6).fill(0);
    for (let i = 0; i < 6; i++) {
      for (let k = 0; k < n; k++) { Jdqn[i] += J[i][k] * dqn[k]; }
    }
    const y2 = solveLinear(A, Jdqn);
    for (let k = 0; k < n; k++) {
      let proj = 0;
      for (let i = 0; i < 6; i++) { proj += J[i][k] * y2[i]; }
      dq[k] += dqn[k] - proj;
    }

    // Joint-velocity limit: bound this cycle's step to maxJointVel * dt.
    const maxStep = this.maxJointVel * Math.min(Math.max(dt, 1e-3), 0.1);
    const worst = Math.max(...dq.map(Math.abs));
    if (worst > maxStep) { for (let k = 0; k < n; k++) { dq[k] *= maxStep / worst; } }

    // Advance the persistent command trajectory, leashed to the measured
    // joint positions and clipped to joint and actuator ranges.
    if (!this.commandInitialized) {
      for (let k = 0; k < n; k++) { this.lastCommand[k] = data.qpos[this.armQposAdr[k]]; }
      this.commandInitialized = true;
    }
    const cmd = new Array(n);
    const buildCmd = (scale) => {
      for (let k = 0; k < n; k++) {
        const qMeas = data.qpos[this.armQposAdr[k]];
        let q = this.lastCommand[k] + dq[k] * scale;
        q = Math.min(Math.max(q, qMeas - this.leash), qMeas + this.leash);
        const j = this.armJointIds[k], a = this.armActIds[k];
        if (m.jnt_range[2 * j] < m.jnt_range[2 * j + 1]) {
          q = Math.min(Math.max(q, m.jnt_range[2 * j]), m.jnt_range[2 * j + 1]);
        }
        if (m.actuator_ctrlrange[2 * a] < m.actuator_ctrlrange[2 * a + 1]) {
          q = Math.min(Math.max(q, m.actuator_ctrlrange[2 * a]), m.actuator_ctrlrange[2 * a + 1]);
        }
        cmd[k] = q;
      }
    };

    // Ground-clearance gate: accept the largest fraction of the step whose
    // clearance is either above the margin, or no worse than the current
    // command's (so a pose already inside the margin — startup, actuator
    // overshoot — can always climb back out instead of deadlocking).
    this.status.groundLimited = false;
    const nowClearance = this.minClearance(data, [...this.lastCommand]);
    let accepted = false;
    for (const scale of [1.0, 0.5, 0.25, 0.125]) {
      buildCmd(scale);
      const c = this.minClearance(data, cmd);
      if (c >= this.clearance || c >= nowClearance - 1e-6) { accepted = true; break; }
      this.status.groundLimited = true;
    }
    if (accepted) {
      for (let k = 0; k < n; k++) {
        data.ctrl[this.armActIds[k]] = cmd[k];
        this.lastCommand[k] = cmd[k];
      }
    }
    return this.status;
  }

  dispose() {
    this.shadow.delete();
    this.jacp.delete(); this.jacr.delete();
    this.quatB.delete(); this.negqB.delete(); this.errqB.delete(); this.errvB.delete();
  }
}

/** Solve the n x n system A x = b by Gaussian elimination with pivoting. */
export function solveLinear(A, b) {
  const n = b.length, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) { if (Math.abs(M[r][c]) > Math.abs(M[p][c])) { p = r; } }
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) { M[r][k] -= f * M[c][k]; }
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) { s -= M[r][k] * x[k]; }
    x[r] = s / M[r][r];
  }
  return x;
}
