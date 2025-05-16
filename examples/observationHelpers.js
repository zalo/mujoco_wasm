import * as THREE from 'three';

function getOscillator(time) {
  const omega = 4.0 * Math.PI
  const phase = [
    omega * time + Math.PI,
    omega * time,
    omega * time,
    omega * time + Math.PI,
  ];
  return [...phase.map(Math.sin), ...phase.map(Math.cos), omega, omega, omega, omega];
}


/**
 * @param {THREE.Quaternion} quat
 */
function yawQuat(quat) {
  const result = new THREE.Quaternion(quat.x, quat.y, quat.z, quat.w);
  const yaw = Math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y * quat.y + quat.z * quat.z));
  result.z = Math.sin(yaw / 2);
  result.w = Math.cos(yaw / 2);
  return result.normalize();
}


class VelocityCommand {
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
    // Unpack kwargs with defaults
    const { setvel = [1.0, 0.0, 0.0], angvel_kp = 1.0, oscillator = true } = kwargs;
    this.setvel_w = new THREE.Vector3(...setvel);
    this.angvel_kp = angvel_kp;
    this.oscillator = oscillator;
  }

  compute(extra_info) {
    const setvel_b = this.setvel_w.clone().applyQuaternion(this.demo.quat.clone().invert());
    if (this.oscillator) {
      const osc = getOscillator(this.demo.mujoco_time / 1000.);
      return [setvel_b.x, setvel_b.y, this.angvel_kp * (0 - this.demo.rpy.z), 0.75, ...osc];
    } else {
      // return [setvel_b.x, setvel_b.y, this.angvel_kp * (0 - this.demo.rpy.z), 0];
      return [1.0, 0.0, 2.0, 0.75];
    }
  }

}


class ImpedanceCommand {
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
    // Unpack kwargs with defaults
    const {
      impedance_kp = 24.0,
      mass = 1.0,
      setvel = [1.0, 0.0, 0.0]
    } = kwargs;
    this.impedance_kp = impedance_kp;
    this.impedance_kd = 1.8 * Math.sqrt(impedance_kp);
    this.mass = mass;
    this.setvel = new THREE.Vector3(...setvel);
  }

  compute(extra_info) {
    const kp = this.impedance_kp;
    const kd = this.impedance_kd;
    const osc = getOscillator(this.demo.mujoco_time / 1000.);

    // Get base position in world frame
    const base_pos_w = new THREE.Vector3(...this.simulation.qpos.subarray(0, 3));
    const setpoint = new THREE.Vector3();
    if (this.demo.ball) {
      setpoint.x = this.demo.ball.position.x;
      setpoint.y = -this.demo.ball.position.z;
    } else {
      setpoint.copy(this.setvel.clone().multiplyScalar(kd / kp).add(base_pos_w));
    }

    // Transform setpoint to body frame
    let setpoint_b = setpoint.sub(base_pos_w).applyQuaternion(this.demo.quat.clone().invert());

    const command = [
      setpoint_b.x, setpoint_b.y,
      0 - this.demo.rpy.z,
      kp * setpoint_b.x, kp * setpoint_b.y,
      kd, kd, kd,
      kp * (0 - this.demo.rpy.z),
      this.mass,
      kp * setpoint_b.x / this.mass, kp * setpoint_b.y / this.mass,
      kd / this.mass, kd / this.mass, kd / this.mass
    ];

    return [...command, ...osc];
  }
}

class BaseAngVelMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {string} base_joint_name
   * @param {number} steps 
   */
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    // Unpack kwargs with defaults
    const { 
      base_joint_name = "floating_base_joint",
      history_steps = 4 
    } = kwargs;
    
    this.steps = history_steps;
    this.angvel_multistep = new Array(this.steps).fill().map(() => new Float32Array(3));
    const joint_idx = demo.jointNamesMJC.indexOf(base_joint_name);
    this.joint_qpos_adr = model.jnt_qposadr[joint_idx];
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    const quat_wxyz = this.simulation.qpos.subarray(this.joint_qpos_adr + 3, this.joint_qpos_adr + 7);
    const quat = new THREE.Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]);
    const ang_vel_w = new THREE.Vector3(...this.simulation.cvel.subarray(6, 9));
    const ang_vel_b = ang_vel_w.clone().applyQuaternion(quat.invert());
    // console.log("cvel", this.simulation.cvel.subarray(6, 9));

    // Update history - shift all elements forward
    for (let i = this.angvel_multistep.length - 1; i > 0; i--) {
      this.angvel_multistep[i] = this.angvel_multistep[i - 1];
    }
    this.angvel_multistep[0] = [ang_vel_b.x, ang_vel_b.y, ang_vel_b.z];
    // Flatten all steps into single array
    const flattened = new Float32Array(this.steps * 3);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.angvel_multistep[i], i * 3);
    }
    return flattened;
  }
}

class BaseLinVel {
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
    
    const {
      joint_name = "floating_base_joint",
      yaw_only = false
    } = kwargs;

    const joint_idx = demo.jointNamesMJC.indexOf(joint_name);
    this.joint_qpos_adr = model.jnt_qposadr[joint_idx];
    this.yaw_only = yaw_only;
  }

  compute(extra_info) {
    const lin_vel_w = new THREE.Vector3(...this.simulation.cvel.subarray(9, 12));
    const quat_wxyz = this.simulation.qpos.subarray(this.joint_qpos_adr + 3, this.joint_qpos_adr + 7);
    // convert to THREE.Quaternion xyzw
    let quat = new THREE.Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]);
    if (this.yaw_only) {
      quat = yawQuat(quat);
    }
    const lin_vel_b = lin_vel_w.clone().applyQuaternion(quat.invert());
    return [lin_vel_b.x, lin_vel_b.y, lin_vel_b.z];
  }
}

class GravityMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {string} base_joint_name
   * @param {number} steps 
   */
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    // Unpack kwargs with defaults
    const { 
      joint_name = "floating_base_joint",
      history_steps = 4,
      gravity = [0, 0, -1.0]
    } = kwargs;
    
    this.steps = history_steps;
    this.gravity_multistep = new Array(this.steps).fill().map(() => new Float32Array(3));

    const joint_idx = demo.jointNamesMJC.indexOf(joint_name);
    this.joint_qpos_adr = model.jnt_qposadr[joint_idx];
    this.gravity = new THREE.Vector3(...gravity);
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Get projected gravity and normalize it
    const quat = this.simulation.qpos.subarray(this.joint_qpos_adr + 3, this.joint_qpos_adr + 7);
    // Create quaternion directly from the array values
    const quat_inv = new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]).invert();
    const gravity = this.gravity.clone().applyQuaternion(quat_inv);

    // Update history
    for (let i = this.gravity_multistep.length - 1; i > 0; i--) {
      this.gravity_multistep[i] = this.gravity_multistep[i - 1];
    }
    this.gravity_multistep[0] = new Float32Array([gravity.x, gravity.y, gravity.z]);

    // Flatten all steps
    const flattened = new Float32Array(this.steps * 3);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.gravity_multistep[i], i * 3);
    }
    return flattened;
  }
}

class JointPosMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {list[string]} joint_names 
   * @param {number} steps 
   */
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    // Unpack kwargs with defaults
    const { 
      joint_names = [],
      history_steps = 4 
    } = kwargs;
    
    this.steps = history_steps;
    this.joint_names = joint_names;
    this.joint_pos_multistep = new Array(this.steps).fill().map(() => new Float32Array(joint_names.length));
    console.log("joint_names", joint_names);

    this.joint_qpos_adr = [];
    for (let i = 0; i < joint_names.length; i++) {
      const idx = demo.jointNamesMJC.indexOf(joint_names[i]);
      const joinqposadr = model.jnt_qposadr[idx];
      this.joint_qpos_adr.push(joinqposadr);
    }
    console.log("jposadr", this.joint_qpos_adr);
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Update history
    for (let i = this.joint_pos_multistep.length - 1; i > 0; i--) {
      this.joint_pos_multistep[i] = this.joint_pos_multistep[i - 1];
    }
    for (let i = 0; i < this.joint_names.length; i++) {
      this.joint_pos_multistep[0][i] = this.simulation.qpos[this.joint_qpos_adr[i]];
    }

    // Flatten all steps
    const flattened = new Float32Array(this.steps * this.joint_names.length);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.joint_pos_multistep[i], i * this.joint_names.length);
    }
    // console.log(flattened.subarray(0, 12));
    return flattened;
  }
}

class JointVelMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {list[string]} joint_names 
   * @param {number} steps 
   */
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    // Unpack kwargs with defaults
    const { 
      joint_names = [],
      history_steps = 4 
    } = kwargs;
    
    this.steps = history_steps;
    this.joint_names = joint_names;
    this.numJoints = joint_names.length;
    this.joint_vel_multistep = new Array(this.steps).fill().map(() => new Float32Array(this.numJoints));

    this.joint_qvel_adr = [];
    for (let i = 0; i < joint_names.length; i++) {
      const idx = demo.jointNamesMJC.indexOf(joint_names[i]);
      const jointqveladr = model.jnt_dofadr[idx];
      this.joint_qvel_adr.push(jointqveladr);
    }
    console.log("jveladr", this.joint_qvel_adr);
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Update history
    for (let i = this.joint_vel_multistep.length - 1; i > 0; i--) {
      this.joint_vel_multistep[i] = this.joint_vel_multistep[i - 1];
    }
    for (let i = 0; i < this.joint_names.length; i++) {
      this.joint_vel_multistep[0][i] = this.simulation.qvel[this.joint_qvel_adr[i]];
    }

    // Flatten all steps
    const flattened = new Float32Array(this.steps * this.numJoints);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.joint_vel_multistep[i], i * this.numJoints);
    }
    return flattened;
  }
}

class PrevActions {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {number} steps 
   */
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    // Unpack kwargs with defaults
    const { history_steps = 4, permute = false } = kwargs;
    
    this.steps = history_steps;
    this.permute = permute;
    this.numActions = demo.numActions;
    this.actionBuffer = demo.actionBuffer;
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    const flattened = new Float32Array(this.steps * this.numActions);
    if (this.permute) {
      for (let i = 0; i < this.steps; i++) {
        for (let j = 0; j < this.numActions; j++) {
          flattened[i * this.numActions + j] = this.actionBuffer[i][j];
        }
      }
    } else {
      for (let j = 0; j < this.numActions; j++) {
        for (let i = 0; i < this.steps; i++) {
          flattened[j * this.steps + i] = this.actionBuffer[i][j];
        }
      }
    }
    return flattened;
  }
}

class AppliedTorque {
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
  }

  compute(extra_info) {
    return this.demo.appliedTorque;
  }
}

class AppliedAction {
  constructor(model, simulation, demo, kwargs = {}) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
  }

  compute(extra_info) {
    return this.demo.lastActions;
  }
}

// Export a dictionary of all observation classes
export const Observations = {
  VelocityCommand,
  ImpedanceCommand,
  BaseAngVelMultistep,
  GravityMultistep,
  JointPosMultistep,
  JointVelMultistep,
  PrevActions,
  BaseLinVel,
  AppliedTorque,
  AppliedAction
};

