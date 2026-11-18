// Sword techniques for humanoid_sword.xml as keyframed joint-space splines (no learning).
// The arm has no wrist, so the blade is rigid with the forearm (25 deg grip). Poses were found offline with numeric IK
// on MuJoCo's own kinematics (blade direction + hand position in the pelvis frame) and by sampling the arm workspace.
// Channels per keyframe: right arm [shoulder1, shoulder2, elbow], torso [abdomen_z, abdomen_y, abdomen_x],
// dz (stand-height offset, m) and lean (forward capture-point offset, m).
// abdomen_z > 0 turns the torso to the right of the pelvis, abdomen_y < 0 bends it forward.

export const POSES = {
  guard:   { arm: [0.604, -0.437, 0.245],  torso: [0, 0, 0] },          // blade forward-up in front of the body
  high:    { arm: [0.297, 0.587, 0.366],   torso: [0, 0.12, 0] },       // overhead, blade pointing back
  chop:    { arm: [0.817, -0.712, -0.516], torso: [0, -0.25, 0] },      // blade forward-down
  chopEnd: { arm: [0.985, 0.047, -1.715],  torso: [0, -0.32, 0] },
  hWind:   { arm: [-0.61, -0.80, -1.70],   torso: [0.5, 0, 0] },        // arm out to the right, blade pointing right
  hMid:    { arm: [0.474, -0.891, -0.215], torso: [0, 0, 0] },          // blade straight ahead at waist height
  hEnd:    { arm: [-1.39, 0.27, 0.17],     torso: [-0.55, 0, 0] },      // across the body, blade pointing left
  hRight:  { arm: [0.221, 0.396, -1.70],   torso: [0.25, 0, 0] },       // arm extended, blade forward-right (backhand finish)
  dWind:   { arm: [0.82, 0.323, 0.725],    torso: [0.35, 0.1, 0] },     // high right
  dEnd:    { arm: [1.017, 0.249, -1.70],   torso: [-0.35, -0.2, 0] },   // low left
  tWind:   { arm: [0.663, -1.42, 0.433],   torso: [0.3, 0.05, 0] },     // hand pulled back by the hip
  thrust:  { arm: [0.66, 0.66, -1.70],     torso: [-0.2, -0.15, 0] },   // arm extended, blade straight ahead
};

// [pose, time from move start (s), { dz, lean, stop }]. Every move starts and ends in guard with zero velocity;
// `stop: true` makes the spline come to rest at that keyframe (wind-ups), otherwise it flows through (strikes).
export const MOVES = {
  'Overhead chop': [['high', 0.45, { dz: 0.01, stop: true }], ['chop', 0.66, { dz: -0.05, lean: 0.03 }], ['chopEnd', 0.80, { dz: -0.06, lean: 0.03, stop: true }],
                    ['chopEnd', 0.95, { dz: -0.06, lean: 0.03 }], ['guard', 1.55]],
  'Horizontal slash': [['hWind', 0.50, { dz: -0.02, stop: true }], ['hMid', 0.68, { dz: -0.04, lean: 0.02 }], ['hEnd', 0.84, { dz: -0.04, lean: 0.02, stop: true }],
                       ['hEnd', 1.0, { dz: -0.04 }], ['guard', 1.6]],
  // (finishing fully out to the right, like hWind, throws the weight onto the left foot and spins the robot on it)
  'Backhand slash': [['hEnd', 0.50, { dz: -0.02, stop: true }], ['hMid', 0.68, { dz: -0.04, lean: 0.02 }], ['hRight', 0.84, { dz: -0.04, stop: true }],
                     ['hRight', 1.0, { dz: -0.03 }], ['guard', 1.6]],
  'Diagonal slash': [['dWind', 0.45, { dz: 0.0, stop: true }], ['dEnd', 0.72, { dz: -0.06, lean: 0.03, stop: true }], ['dEnd', 0.9, { dz: -0.06, lean: 0.03 }],
                     ['guard', 1.5]],
  'Thrust': [['tWind', 0.40, { dz: -0.03, stop: true }], ['thrust', 0.58, { dz: -0.05, lean: 0.04, stop: true }], ['thrust', 0.8, { dz: -0.05, lean: 0.04 }],
             ['guard', 1.4]],
};
export const MOVE_NAMES = Object.keys(MOVES);
// Tempo per technique (time scale). The capsule feet have no torsional friction, so the wide horizontal sweeps run
// slower: at full speed their arm momentum spins the robot on its feet.
export const MOVE_SPEED = { 'Overhead chop': 1.0, 'Horizontal slash': 0.6, 'Backhand slash': 0.6, 'Diagonal slash': 1.0, 'Thrust': 0.8 };
export const KATA = ['Overhead chop', 'Horizontal slash', 'Backhand slash', 'Diagonal slash', 'Thrust'];

const NCH = 8;   // arm(3) torso(3) dz lean
const vec = (pose, o = {}) => [...POSES[pose].arm, ...POSES[pose].torso, o.dz || 0, o.lean || 0];

// Precompute keyframe times, values and Hermite tangents per move.
const COMPILED = {};
for (const [name, frames] of Object.entries(MOVES)) {
  const K = [{ t: 0, v: vec('guard'), stop: true }, ...frames.map(([p, t, o = {}]) => ({ t, v: vec(p, o), stop: !!o.stop }))];
  K[K.length - 1].stop = true;
  for (let i = 0; i < K.length; i++) {
    K[i].m = new Array(NCH).fill(0);
    if (K[i].stop || i === 0 || i === K.length - 1) continue;
    const a = K[i - 1], b = K[i + 1];
    for (let c = 0; c < NCH; c++) K[i].m[c] = (b.v[c] - a.v[c]) / (b.t - a.t);   // Catmull-Rom tangent
  }
  COMPILED[name] = K;
}
export const moveDuration = (name) => { const K = COMPILED[name]; return K[K.length - 1].t / (MOVE_SPEED[name] || 1); };

/** Sample a move at time t (s since it started). Returns null once it is over. */
export function sampleMove(name, t, speed = 1) {
  const K = COMPILED[name];
  if (!K || t >= K[K.length - 1].t) return null;
  let i = 0; while (K[i + 1].t <= t) i++;
  const a = K[i], b = K[i + 1], T = b.t - a.t, s = (t - a.t) / T;
  const s2 = s * s, s3 = s2 * s;
  // cubic Hermite basis and its derivatives (w.r.t. s)
  const h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
  const d00 = 6 * s2 - 6 * s, d10 = 3 * s2 - 4 * s + 1, d01 = -6 * s2 + 6 * s, d11 = 3 * s2 - 2 * s;
  const e00 = 12 * s - 6, e10 = 6 * s - 4, e01 = -12 * s + 6, e11 = 6 * s - 2;
  const q = [], qd = [], qdd = [];
  for (let c = 0; c < NCH; c++) {
    const p0 = a.v[c], p1 = b.v[c], m0 = a.m[c] * T, m1 = b.m[c] * T;
    q.push(h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1);
    qd.push(speed * (d00 * p0 + d10 * m0 + d01 * p1 + d11 * m1) / T);
    qdd.push(speed * speed * (e00 * p0 + e10 * m0 + e01 * p1 + e11 * m1) / (T * T));
  }
  return { arm: { q: q.slice(0, 3), qd: qd.slice(0, 3), qdd: qdd.slice(0, 3) }, torso: { q: q.slice(3, 6), qd: qd.slice(3, 6) }, dz: q[6], lean: q[7] };
}

export const GUARD = { arm: { q: POSES.guard.arm, qd: [0, 0, 0], qdd: [0, 0, 0] }, torso: { q: [0, 0, 0], qd: [0, 0, 0] }, dz: 0, lean: 0 };
export const LEFT_ARM_SWORD = [0.65, -1.05, -0.6];   // free hand hangs at the side

/**
 * Sequencer used by both the browser UI and the tests: queue moves, loop a kata, and overlay the sword arm / torso
 * onto a base controller command.
 */
export class SwordPlayer {
  constructor() { this.queue = []; this.current = null; this.start = 0; this.kata = false; this.kataIdx = 0; this.torsoScale = 1; this.settle = 0.4; this.speed = 1;
    this.walkSpeed = 0.6; this.walkTorso = 0.3;    // gentler variant latched when a move starts while walking (full intensity fell in 2/5 walking tests)
    this.curSpeed = 1; this.curTorso = 1; }
  trigger(name) { if (MOVES[name] && this.queue.length < 3) this.queue.push(name); }
  get busy() { return !!this.current; }
  /** Returns the sword sample for this time and advances the sequence. New moves only start when `ready`. */
  sample(time, ready = true, walking = false) {
    if (this.current && time < this.start) this.current = null;   // simulation reset
    let s = this.current ? sampleMove(this.current, (time - this.start) * this.curSpeed, this.curSpeed) : null;
    if (!s) {
      this.current = null;
      if (!this.queue.length && this.kata && ready) { this.queue.push(KATA[this.kataIdx++ % KATA.length]); }
      if (this.queue.length && ready) {
        this.current = this.queue.shift(); this.start = time;
        this.curSpeed = this.speed * (MOVE_SPEED[this.current] || 1) * (walking ? this.walkSpeed : 1);
        this.curTorso = this.torsoScale * (walking ? this.walkTorso : 1);
        s = sampleMove(this.current, 0, 1);
      }
    }
    return s || GUARD;
  }
  /** Overlay onto a controller command (base from the behaviour runner). Height dip / lean only while standing. */
  apply(cmd, time, controller, baseHeight = 0.8) {
    const S = controller ? controller.S : null;
    const standing = !S || S.mode === 'stand';
    // start a technique only from a settled stand (not during a capture step) or while deliberately walking
    const ready = !S || (cmd.walk ? true : standing && S.standT > this.settle);
    const s = this.sample(time, ready, !!cmd.walk || (S && S.mode === 'walk'));
    const k = this.current ? this.curTorso : this.torsoScale;
    const out = { ...cmd, torso: { q: s.torso.q.map(x => k * x), qd: s.torso.qd.map(x => k * x) } };
    const left = cmd.arms && cmd.arms[1] ? cmd.arms[1] : LEFT_ARM_SWORD;
    out.arms = [{ q: s.arm.q, qd: s.arm.qd, qdd: s.arm.qdd, kp: [120, 120, 80], kd: [8, 8, 5] }, left];
    if (standing && !cmd.walk) {
      out.height = (cmd.height ?? baseHeight) + s.dz;
      const off = cmd.comOffset || [0, 0];
      out.comOffset = [off[0] + s.lean, off[1]];
    }
    return out;
  }
}
