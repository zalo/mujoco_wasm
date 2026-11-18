// Controller registry: returns an analytic controller for scenes it knows how to drive, or null.
import { AnalyticHumanoidController } from './analyticHumanoid.js';
import { isHumanoidModel } from './humanoidModel.js';

export { AnalyticHumanoidController } from './analyticHumanoid.js';
export { BEHAVIOURS, BEHAVIOUR_NAMES, BehaviourRunner } from './behaviours.js';

/** @returns {AnalyticHumanoidController|null} */
export function createController(mujoco, model, data, opts = {}) {
  if (isHumanoidModel(mujoco, model)) return new AnalyticHumanoidController(mujoco, model, data, opts);
  return null;
}
export { SwordPlayer, MOVE_NAMES, KATA } from './swordMoves.js';

/** True if the scene's humanoid holds the training sword (humanoid_sword.xml). */
export function hasSword(mujoco, model) {
  return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, 'sword_blade') >= 0;
}
