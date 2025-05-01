import * as THREE from 'three';

class BaseAngVelMultistep {
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

    const base_joint_id = demo.jointNames.findIndex(joint => joint === base_joint_name);
    const joint_dof_adr = model.jnt_dofadr[base_joint_id];
    this.joint_dof_adr = joint_dof_adr;
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
    this.angvel_multistep[0] = this.simulation.qvel.subarray(this.joint_dof_adr, this.joint_dof_adr + 3);

    // Flatten all steps into single array
    const flattened = new Float32Array(this.steps * 3);
    for (let i = 0; i < this.steps; i++) {
      flattened.set(this.angvel_multistep[i], i * 3);
    }
    return flattened;
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
  constructor(model, simulation, demo, base_joint_name, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.gravity_multistep = new Array(steps).fill().map(() => new Float32Array(3));

    // Fix undefined variables
    const base_joint_id = demo.jointNames.findIndex(joint => joint === base_joint_name);
    const joint_qpos_adr = model.jnt_qposadr[base_joint_id];
    this.joint_qposadr = joint_qpos_adr;

    this.gravity = new THREE.Vector3(0, 0, -1.0);
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Get projected gravity and normalize it
    const quat = this.simulation.qpos.subarray(this.joint_qposadr, this.joint_qposadr + 4);
    // Create quaternion directly from the array values
    const quat_inv = new THREE.Quaternion(quat[0], quat[1], quat[2], quat[3]).invert();
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
  constructor(model, simulation, demo, joint_names, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.joint_names = joint_names;
    this.joint_pos_multistep = new Array(steps).fill().map(() => new Float32Array(joint_names.length));

    this.joint_qpos_adr = [];
    for (let i = 0; i < joint_names.length; i++) {
      const joinidx = demo.jointNames.findIndex(joint => joint === joint_names[i]);
      const joinqposadr = model.jnt_qposadr[joinidx];
      this.joint_qpos_adr.push(joinqposadr);
    }
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
  constructor(model, simulation, demo, joint_names, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.joint_names = joint_names;
    this.numJoints = joint_names.length;
    this.joint_vel_multistep = new Array(steps).fill().map(() => new Float32Array(this.numJoints));

    this.joint_qvel_adr = [];
    for (let i = 0; i < joint_names.length; i++) {
      const joinidx = demo.jointNames.findIndex(joint => joint === joint_names[i]);
      const joinqveladr = model.jnt_dofadr[joinidx];
      this.joint_qvel_adr.push(joinqveladr);
    }
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
  constructor(model, simulation, demo, steps = 4) {
    this.model = model;
    this.simulation = simulation;
    this.steps = steps;
    this.numActions = demo.numActions;
    this.prevActions = new Array(steps).fill().map(() => new Float32Array(this.numActions));
  }

  /**
   * 
   * @param {dict} extra_info
   * @returns {Float32Array}
   */
  compute(extra_info) {
    // Return previous actions and update them
    for (let i = this.prevActions.length - 1; i > 0; i--) {
      // Create a new Float32Array with copied values
      this.prevActions[i] = new Float32Array(this.prevActions[i - 1]);
    }
    // Create a new Float32Array for the current control values
    this.prevActions[0] = new Float32Array(this.simulation.ctrl);

    // Flatten all steps
    const flattened = new Float32Array(this.steps * this.numActions);
    for (let i = 0; i < this.steps; i++) {
      if (this.prevActions[i]) {  // Check if the array exists
        flattened.set(this.prevActions[i], i * this.numActions);
      }
    }
    return flattened;
  }
}

export { BaseAngVelMultistep, GravityMultistep, JointPosMultistep, JointVelMultistep, PrevActions };