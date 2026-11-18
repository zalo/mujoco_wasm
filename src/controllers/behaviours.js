// Scripted "actions" built on the analytic controller's command interface. Each behaviour maps the time since it was
// selected (s) and the user's walk command to a full controller command. Amplitudes ramp in over ~1.5 s so switching
// behaviours never produces a step change in the balance targets.

const ramp = (t, T = 1.5) => Math.min(1, Math.max(0, t / T));

// right-arm wave: hand raised beside the head, forearm swinging at 2 Hz
export const waveArm = (t) => [-1.45, 0.10 + 0.35 * Math.sin(4 * Math.PI * t), -1.3 + 0.4 * Math.sin(4 * Math.PI * t)];
const slowArm = (t) => [-1.2, 0.10 + 0.3 * Math.sin(1.2 * Math.PI * t), -1.3 + 0.3 * Math.sin(1.2 * Math.PI * t)];

export const BEHAVIOURS = {
  // Stand still; any walk command (keys / sliders) makes it walk, releasing makes it stop.
  'Stand / Walk': (t, u) => ({ walk: u.walk, vx: u.vx, vy: u.vy, wz: u.wz, height: u.height }),
  // Never stops stepping: zero command marches in place (most robust gait mode).
  'March': (t, u) => ({ walk: true, vx: u.vx, vy: u.vy, wz: u.wz }),
  // Deep knee bends, COM 0.66 <-> 0.80 m at 0.4 Hz.
  'Squats': (t) => ({ walk: false, height: 0.80 - ramp(t) * (0.07 - 0.07 * Math.sin(0.8 * Math.PI * t + Math.PI / 2)) }),
  // Circle the COM (via the capture-point target) inside the support polygon.
  'Sway': (t) => { const r = ramp(t); return { walk: false, comOffset: [r * 0.04 * Math.sin(Math.PI * t), r * 0.06 * Math.cos(Math.PI * t)] }; },
  // Wave with the right arm (also works while walking: hold W).
  'Wave': (t, u) => ({ walk: u.walk, vx: u.vx, vy: u.vy, wz: u.wz, arms: [waveArm(t), null] }),
  // Gentle dance: knee bob + lateral sway + slow arm waves.
  'Dance': (t) => {
    const r = ramp(t);
    return { walk: false, height: 0.80 - r * (0.05 - 0.03 * Math.sin(1.2 * Math.PI * t)), comOffset: [0, r * 0.05 * Math.sin(0.6 * Math.PI * t)],
             arms: [slowArm(t), slowArm(t + 0.8)] };
  },
};

export const BEHAVIOUR_NAMES = Object.keys(BEHAVIOURS);

// Behaviours that move the COM around a standing robot: their clock only runs once the controller has been standing
// (no stepping) for SETTLE_TIME, so they never start on top of a capture step / shuffle after a stop.
export const NEEDS_STAND = new Set(['Squats', 'Sway', 'Dance']);
export const SETTLE_TIME = 0.8;

/** Tracks behaviour time and produces the per-step controller command. Shared by the browser UI and the tests. */
export class BehaviourRunner {
  constructor(name = BEHAVIOUR_NAMES[0]) { this.select(name, 0); }
  select(name, time) { this.name = name; this.start = time; }
  command(time, user, controller) {
    if (time < this.start) this.start = time;                         // simulation was reset
    if (NEEDS_STAND.has(this.name) && controller) {
      const S = controller.S;
      if (S.mode !== 'stand' || S.standT < SETTLE_TIME) {
        this.start = time;                                            // hold the clock (amplitude ramps from zero)
        return { walk: false };
      }
    }
    return BEHAVIOURS[this.name](time - this.start, user);
  }
}
