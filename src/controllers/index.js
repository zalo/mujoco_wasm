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
