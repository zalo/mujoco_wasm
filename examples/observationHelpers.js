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

export class VelocityCommand {
  constructor(model, simulation, demo) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
    this.setvel = new THREE.Vector3(1.0, 0.0, 0.0);
    this.angvel_kp = 1.0
  }

  compute(extra_info) {
    const osc = getOscillator(this.demo.mujoco_time / 1000.);
    const setvel_b = this.setvel.clone().applyQuaternion(this.demo.quat.clone().invert());
    return [setvel_b.x, setvel_b.y, this.angvel_kp * (0 - this.demo.rpy.z), 0, ...osc];
  }

}


export class ImpedanceCommand {
  constructor(model, simulation, demo) {
    this.model = model;
    this.simulation = simulation;
    this.demo = demo;
    // Add impedance control parameters
    this.impedance_kp = 100.0; // Position gain
    this.impedance_kd = 20.0;  // Velocity gain
    this.setvel = new THREE.Vector3(1.0, 0.0, 0.0);
    this.mass = 1.0;
  }

  compute(extra_info) {
    const kp = this.impedance_kp;
    const kd = this.impedance_kd;
    const osc = getOscillator(this.demo.mujoco_time / 1000.);

    // Get base position in world frame
    const base_pos_w = new THREE.Vector3(...this.simulation.qpos.subarray(0, 3));
    const setpoint = this.setvel.clone().multiplyScalar(kd / kp).add(base_pos_w);

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

export class BaseAngVelMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {string} base_joint_name
   * @param {number} steps 
   */
  constructor(model, simulation, demo, base_joint_name, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.angvel_multistep = new Array(steps).fill().map(() => new Float32Array(3));

    const joint_idx = demo.jointNamesMJC.indexOf(base_joint_name);
    this.joint_qvel_adr = model.jnt_dofadr[joint_idx];
    console.log("joint_qvel_adr", this.joint_qvel_adr);
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Update history - shift all elements forward
    for (let i = this.angvel_multistep.length - 1; i > 0; i--) {
      this.angvel_multistep[i] = this.angvel_multistep[i - 1];
    }
    const angvel = this.simulation.qvel.subarray(this.joint_qvel_adr, this.joint_qvel_adr + 3);
    this.angvel_multistep[0] = angvel;
    // Flatten all steps into single array
    const flattened = new Float32Array(this.steps * 3);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.angvel_multistep[i], i * 3);
    }
    return flattened;
  }
}

export class GravityMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {string} base_joint_name
   * @param {number} steps 
   */
  constructor(model, simulation, demo, base_joint_name, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.gravity_multistep = new Array(steps).fill().map(() => new Float32Array(3));

    // Fix undefined variables
    const joint_idx = demo.jointNamesMJC.indexOf(base_joint_name);
    this.joint_qpos_adr = model.jnt_qposadr[joint_idx];
    this.gravity = new THREE.Vector3(0, 0, -1.0);
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

export class JointPosMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {list[string]} joint_names 
   * @param {number} steps 
   */
  constructor(model, simulation, demo, joint_names, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.joint_names = joint_names;
    this.joint_pos_multistep = new Array(steps).fill().map(() => new Float32Array(joint_names.length));

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

export class JointVelMultistep {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {list[string]} joint_names 
   * @param {number} steps 
   */
  constructor(model, simulation, demo, joint_names, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.joint_names = joint_names;
    this.numJoints = joint_names.length;
    this.joint_vel_multistep = new Array(steps).fill().map(() => new Float32Array(this.numJoints));

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

export class PrevActions {
  /**
   * 
   * @param {mujoco.Model} model 
   * @param {mujoco.Simulation} simulation 
   * @param {MuJoCoDemo} demo
   * @param {number} steps 
   */
  constructor(model, simulation, demo, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
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
    for (let j = 0; j < this.numActions; j++) {
      for (let i = 0; i < this.steps; i++) {
        flattened[j * this.steps + i] = this.actionBuffer[i][j];
      }
    }
    return flattened;
  }
}
