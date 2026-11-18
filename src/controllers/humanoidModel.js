// Index tables, geometry constants and closed-form kinematics for the stock DeepMind capsule humanoid
// (assets/scenes/humanoid.xml). Pure functions, no DOM / three.js, so Node tests import the shipped code.

export const JOINT_NAMES = [
  'abdomen_z', 'abdomen_y', 'abdomen_x',
  'hip_x_right', 'hip_z_right', 'hip_y_right', 'knee_right', 'ankle_y_right', 'ankle_x_right',
  'hip_x_left', 'hip_z_left', 'hip_y_left', 'knee_left', 'ankle_y_left', 'ankle_x_left',
  'shoulder1_right', 'shoulder2_right', 'elbow_right', 'shoulder1_left', 'shoulder2_left', 'elbow_left'];

export const BODY_NAMES = ['torso', 'head', 'waist_lower', 'pelvis', 'thigh_right', 'shin_right', 'foot_right',
  'thigh_left', 'shin_left', 'foot_left', 'hand_right', 'hand_left'];

export const GEOM_NAMES = ['floor', 'foot1_right', 'foot2_right', 'foot1_left', 'foot2_left'];

/** True if the model has every name the controller needs (used to decide whether a scene gets a controller). */
export function isHumanoidModel(mujoco, m) {
  const O = mujoco.mjtObj;
  return JOINT_NAMES.every(n => mujoco.mj_name2id(m, O.mjOBJ_JOINT.value, n) >= 0 &&
                                mujoco.mj_name2id(m, O.mjOBJ_ACTUATOR.value, n) >= 0) &&
         BODY_NAMES.every(n => mujoco.mj_name2id(m, O.mjOBJ_BODY.value, n) >= 0) &&
         GEOM_NAMES.every(n => mujoco.mj_name2id(m, O.mjOBJ_GEOM.value, n) >= 0);
}

/** Build name -> index tables. Throws if anything is missing. */
export function makeIndex(mujoco, m) {
  const O = mujoco.mjtObj;
  const lookup = (type, label) => (n) => {
    const i = mujoco.mj_name2id(m, type, n);
    if (i < 0) throw new Error(`humanoid controller: model has no ${label} "${n}"`);
    return i;
  };
  const jid = lookup(O.mjOBJ_JOINT.value, 'joint');
  const bid = lookup(O.mjOBJ_BODY.value, 'body');
  const aid = lookup(O.mjOBJ_ACTUATOR.value, 'actuator');
  const gid = lookup(O.mjOBJ_GEOM.value, 'geom');
  const J = {};   // name -> { j: joint id, q: qpos adr, d: dof adr, a: actuator id, gear, lo, hi }
  for (const n of JOINT_NAMES) {
    const j = jid(n), a = aid(n);
    J[n] = { j, q: m.jnt_qposadr[j], d: m.jnt_dofadr[j], a, gear: m.actuator_gear[6 * a],
             lo: m.jnt_range[2 * j], hi: m.jnt_range[2 * j + 1] };
  }
  const B = {};
  for (const n of BODY_NAMES) B[n] = bid(n);
  const G = {};
  for (const n of GEOM_NAMES) G[n] = gid(n);
  return { J, B, G, names: JOINT_NAMES };
}

export const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Crouch pose for thigh pitch a: shin pitch b keeps the ankle under the hip (L1 = 0.38, L2 = 0.33). */
export function crouchPose(a) {
  const b = Math.asin(clamp(0.38 / 0.33 * Math.sin(a), -1, 1));
  return { hip_y: -a, knee: -(a + b), ankle_y: -b };
}

/** Thigh-pitch bend `a` that gives COM height z in the crouch pose (bisection). */
export function bendForComZ(z, P) {
  let lo = 0.02, hi = 1.3;
  for (let i = 0; i < 24; i++) {
    const a = 0.5 * (lo + hi), b = Math.asin(clamp(P.L1 / P.L2 * Math.sin(a), -1, 1));
    const zz = P.L1 * Math.cos(a) + P.L2 * Math.cos(b) + P.ANKLE_H + P.comAboveHip;
    if (zz > z) lo = a; else hi = a;
  }
  return 0.5 * (lo + hi);
}

/**
 * Analytic leg IK. v = ankle pivot relative to hip pivot in the PELVIS frame; side 0 = right, 1 = left;
 * qzJoint = desired hip_z joint value. Joint chain: R = Rx(q_x) Rz(q_z) Ry(q_y), knee Ry(-q_k); left x/z axes mirrored.
 */
export function legIK(v, side, qzJoint, P) {
  const L1 = P.L1, L2 = P.L2, eps = side === 0 ? P.EPS : -P.EPS;
  const sgn = side === 0 ? 1 : -1;
  const Qz = sgn * qzJoint;
  let r2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  const rmax = Math.sqrt(eps * eps + (L1 + L2) * (L1 + L2)) * 0.9995, rmin = 0.25;
  const r = Math.sqrt(r2);
  let scale = 1;
  if (r > rmax) scale = rmax / r; else if (r < rmin) scale = rmin / r;
  const vx = v[0] * scale, vy = v[1] * scale, vz = v[2] * scale;
  r2 = vx * vx + vy * vy + vz * vz;
  const cth = clamp((r2 - eps * eps - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
  const th = Math.acos(cth);                                    // knee flexion >= 0
  const ax = -L2 * Math.sin(th), az = -L1 - L2 * Math.cos(th);  // ankle in thigh frame (x, z); y = eps
  const wx = (vx + eps * Math.sin(Qz)) / Math.cos(Qz);
  const wz = -Math.sqrt(Math.max(1e-9, r2 - eps * eps - wx * wx));
  const uy = wx * Math.sin(Qz) + eps * Math.cos(Qz), uz = wz;
  const Qx = Math.atan2(vz, vy) - Math.atan2(uz, uy);
  const qy = Math.atan2(wx, wz) - Math.atan2(ax, az);
  return { hip_x: sgn * wrap(Qx), hip_z: qzJoint, hip_y: wrap(qy), knee: -th };
}
