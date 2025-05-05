import * as THREE from 'three';
import * as ort from 'onnxruntime-web';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal, reloadScene, reloadPolicy } from './mujocoUtils.js';

// Load the MuJoCo Module
import load_mujoco from '../dist/mujoco_wasm.js';
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
var initialScene = "unitree_go2/scene.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    this.params = { scene: initialScene, paused: true, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0, policy: './examples/checkpoints/policy-05-03_21-31.json' };
    this.lastActions = null;
    this.isInferencing = false;
    this.observations = {}

    this.bodies = {}, this.lights = {};
    this.updateGUICallbacks = [];

    this.container = document.createElement('div');
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap

    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);

    this.lastSimState = {
      bodies: new Map(),
      lights: new Map(),
      tendons: {
        numWraps: 0,
        matrix: new THREE.Matrix4()
      }
    };

    this.renderer.setAnimationLoop(this.render.bind(this));
  }

  async init() {
    // Download the examples first
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model
    let reload_scene = reloadScene.bind(this);
    await reload_scene();

    // Set up simulation parameters
    this.timestep = this.model.getOptions().timestep;
    this.decimation = Math.round(0.02 / this.timestep);
    this.mujoco_time = 0.0;
    this.simStepCount = 0;
    this.inferenceStepCount = 0;

    console.log("timestep:", this.timestep, "decimation:", this.decimation);

    this.adapt_hx = new Float32Array(128);
    this.rpy = new THREE.Euler();

    // Reload policy
    let reload_policy = reloadPolicy.bind(this);
    await reload_policy();

    this.jntKp = new Float32Array(this.numActions).fill(25.);
    this.jntKd = new Float32Array(this.numActions).fill(0.5);

    this.gui = new GUI();
    setupGUI(this);
  }

  async main_loop() {
    while (true) {
      const loopStart = performance.now();
      if (!this.params["paused"] && this.model != null && this.state != null && this.simulation != null && this.observations != null) {
        let time_start = performance.now();
        // Run policy inference
        const quat = this.simulation.qpos.subarray(3, 7);
        this.quat = new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]);
        this.rpy.setFromQuaternion(this.quat);
        const obs_dict = this.getObservations(this.simulation);
        await this.runInference(obs_dict);

        let time_end = performance.now();
        const policy_inference_time = time_end - time_start;
        time_start = time_end;

        // step simulation for decimation times
        for (let substep = 0; substep < this.decimation; substep++) {
          // Apply control torque
          if (this.lastActions) {
            for (let i = 0; i < this.numActions; i++) {
              const qpos_adr = this.qpos_adr_isaac[i];
              const qvel_adr = this.qvel_adr_isaac[i];
              const ctrl_adr = this.ctrl_adr_isaac[i];
            
              const targetJpos = 0.5 * this.lastActions[i] + this.defaultJpos[i];
              const torque = this.jntKp[i] * (targetJpos - this.simulation.qpos[qpos_adr]) + this.jntKd[i] * (0 - this.simulation.qvel[qvel_adr]);
              this.simulation.ctrl[ctrl_adr] = torque;
            }
          }

          // Handle perturbations
          for (let i = 0; i < this.simulation.qfrc_applied.length; i++) {
            this.simulation.qfrc_applied[i] = 0.0;
          }
          let dragged = this.dragStateManager.physicsObject;
          if (dragged && dragged.bodyID) {
            for (let b = 0; b < this.model.nbody; b++) {
              if (this.bodies[b]) {
                getPosition(this.simulation.xpos, b, this.bodies[b].position);
                getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
                this.bodies[b].updateWorldMatrix();
              }
            }
            let bodyID = dragged.bodyID;
            this.dragStateManager.update(); // Update the world-space force origin
            // TODO: add damping force, need to add body velocity sensor
            let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(25));
            console.log("force", force);
            let point = toMujocoPos(this.dragStateManager.worldHit.clone());
            this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);
          }

          // Step simulation
          this.simulation.step();
          this.mujoco_time += this.timestep * 1000.0;
          this.simStepCount += 1;
        }
        time_end = performance.now();
        const sim_step_time = time_end - time_start;
        time_start = time_end;

        // Update cached state for renderer
        for (let b = 0; b < this.model.nbody; b++) {
          if (this.bodies[b]) {
            if (!this.lastSimState.bodies.has(b)) {
              this.lastSimState.bodies.set(b, {
                position: new THREE.Vector3(),
                quaternion: new THREE.Quaternion()
              });
            }
            getPosition(this.simulation.xpos, b, this.lastSimState.bodies.get(b).position);
            getQuaternion(this.simulation.xquat, b, this.lastSimState.bodies.get(b).quaternion);
          }
        }

        // Cache light states
        for (let l = 0; l < this.model.nlight; l++) {
          if (this.lights[l]) {
            if (!this.lastSimState.lights.has(l)) {
              this.lastSimState.lights.set(l, {
                position: new THREE.Vector3(),
                direction: new THREE.Vector3()
              });
            }
            getPosition(this.simulation.light_xpos, l, this.lastSimState.lights.get(l).position);
            getPosition(this.simulation.light_xdir, l, this.lastSimState.lights.get(l).direction);
          }
        }

        // Pre-compute tendon visualization data
        if (this.mujocoRoot && this.mujocoRoot.cylinders) {
          let numWraps = 0;
          const mat = this.lastSimState.tendons.matrix;

          for (let t = 0; t < this.model.ntendon; t++) {
            let startW = this.simulation.ten_wrapadr[t];
            let r = this.model.tendon_width[t];
            for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] - 1; w++) {
              let tendonStart = getPosition(this.simulation.wrap_xpos, w, new THREE.Vector3());
              let tendonEnd = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
              let tendonAvg = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

              let validStart = tendonStart.length() > 0.01;
              let validEnd = tendonEnd.length() > 0.01;

              if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps, mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
              if (validEnd) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
              if (validStart && validEnd) {
                mat.compose(tendonAvg,
                  new THREE.Quaternion().setFromUnitVectors(
                    new THREE.Vector3(0, 1, 0),
                    tendonEnd.clone().sub(tendonStart).normalize()
                  ),
                  new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r)
                );
                this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
                numWraps++;
              }
            }
          }
          this.lastSimState.tendons.numWraps = numWraps;
        }

        time_end = performance.now();
        const update_render_time = time_end - time_start;
        if ((this.simStepCount) % (50 * this.decimation) == 0) {
          console.log("policy inference time:", policy_inference_time / 1000);
          console.log("sim_step_time:", sim_step_time / 1000);
          console.log("update_render_time:", update_render_time / 1000)
        }
      }

      // Calculate time to sleep
      const loopEnd = performance.now();
      const elapsed = (loopEnd - loopStart) / 1000;
      const sleepTime = Math.max(0, this.timestep * this.decimation - elapsed);

      // calculate actual frequency
      if ((this.simStepCount) % (50 * this.decimation) == 0) {
        const actualFreq = 1 / (elapsed + sleepTime);
        console.log("elapsed", elapsed);
        console.log("timestep", this.timestep);
        console.log("sleepTime", sleepTime);
        console.log("main loop frequency:", actualFreq);
      }

      await new Promise(resolve => setTimeout(resolve, sleepTime * 1000));
    }
  }

  async runInference(obs_dict) {
    if (!this.policy || this.isInferencing) {
      console.log("inference lag");
      return;
    }

    const inferenceStart = performance.now();

    this.isInferencing = true;
    this.inferenceStepCount += 1;
    try {
      // Create input object with initial tensors
      const input = {
        "is_init": new ort.Tensor('bool', [false], [1]),
        "adapt_hx": new ort.Tensor('float32', this.adapt_hx, [1, 128])
      };

      // Convert observation arrays to tensors and add to input
      for (const [key, value] of Object.entries(obs_dict)) {
        input[key] = new ort.Tensor('float32', value, [1, value.length]);
      }

      const result = await this.policy.runInference(input);

      if (this.lastActions !== null) {
        for (let i = 0; i < this.lastActions.length; i++) {
          this.lastActions[i] = this.lastActions[i] * 0.8 + result["action"][i] * 0.2;
        }
      } else {
        this.lastActions = result["action"];
      }
      // console.log("lastActions", this.lastActions);
      for (let i = this.actionBuffer.length - 1; i > 0; i--) {
        this.actionBuffer[i] = this.actionBuffer[i - 1];
      }
      this.actionBuffer[0] = this.lastActions;
      this.adapt_hx = result["next,adapt_hx"];
    } catch (e) {
      console.error("Failed to start inference:", e);
    } finally {
      this.isInferencing = false;
    }

    const inferenceEnd = performance.now();
    // console.log("onnx inference time", (inferenceEnd - inferenceStart) / 1000);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  getObservations(simulation) {
    let obs_dict = {};
    for (const [obs_key, obs_funcs] of Object.entries(this.observations)) {
      let obs_for_this_key = [];
      for (const obs_func of obs_funcs) {
        const obs = obs_func.compute(simulation);
        if (obs.some(isNaN)) {
          console.log("NaN in observation", obs_func.constructor.name);
        }
        obs_for_this_key.push(...obs);
      }
      obs_dict[obs_key] = obs_for_this_key;
    }
    return obs_dict;
  }

  render() {
    if (!this.model || !this.state || !this.simulation) {
      return;
    }

    const render_start = performance.now();

    this.controls.update();
    // Update threejs scene from cached simulation state
    for (const [b, state] of this.lastSimState.bodies) {
      if (this.bodies[b]) {
        this.bodies[b].position.copy(state.position);
        this.bodies[b].quaternion.copy(state.quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    for (const [l, state] of this.lastSimState.lights) {
      if (this.lights[l]) {
        this.lights[l].position.copy(state.position);
        this.lights[l].lookAt(state.direction.add(this.lights[l].position));
      }
    }

    // Update tendon visualization from pre-computed data
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      const { numWraps } = this.lastSimState.tendons.numWraps;
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres.count = numWraps > 0 ? numWraps + 1 : 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres.instanceMatrix.needsUpdate = true;
    }

    // Render the scene
    this.renderer.render(this.scene, this.camera);

    const render_end = performance.now();
    if ((this.simStepCount / this.decimation) % 50 == 0) {
      const render_time = render_end - render_start;
      // console.log("render time", render_time);
    }
  }
}

let demo = new MuJoCoDemo();
await demo.init();
demo.params["paused"] = false;
demo.main_loop();