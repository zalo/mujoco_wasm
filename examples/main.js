import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

import * as ort from 'onnxruntime-web';
// Set up Emscripten's Virtual File System
// var initialScene = "humanoid.xml";
var initialScene = "unitree_go2/scene.xml";
// var initialScene = "unitree_g1/scene.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

class ONNXModule {
  constructor(modelPath) {
    this.modelPath = modelPath;
    this.metaPath = modelPath.replace('.onnx', '.json');
  }

  async init() {
    // Load the ONNX model
    const modelResponse = await fetch(this.modelPath);
    const modelArrayBuffer = await modelResponse.arrayBuffer();
    const meta = await (fetch(this.metaPath).then(response => response.json()));
    
    this.inKeys = meta["in_keys"];
    this.outKeys = meta["out_keys"];

    // Create session from the array buffer
    this.session = await ort.InferenceSession.create(modelArrayBuffer, {
      executionProviders: ['wasm'],
      graphOptimizationLevel: 'all'
    });

    console.log('ONNX model loaded successfully');
    console.log(this.session.inputNames);
    console.log(this.session.outputNames);
  }

  async runInference(input) {
    let onnxInput = {};
    for (let i = 0; i < this.inKeys.length; i++) {
      onnxInput[this.session.inputNames[i]] = input[this.inKeys[i]];
    }
    // const inferenceStart = performance.now();
    const onnxOutput = await this.session.run(onnxInput);
    // const inferenceEnd = performance.now();
    // console.log("inference time", inferenceEnd - inferenceStart)
    let result = {};
    for (let i = 0; i < this.outKeys.length; i++) {
      result[this.outKeys[i]] = onnxOutput[this.session.outputNames[i]].data;
    }
    return result;
  }
}

// Observation helper classes
import { BaseAngVelMultistep, GravityMultistep, JointPosMultistep, JointVelMultistep, PrevActions } from './observationHelpers.js';
export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Move model loading to init()
    // Just initialize basic parameters here
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.jointNamesMJC = [];
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio( window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap

    this.container.appendChild( this.renderer.domElement );

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

    // Add new properties for ONNX Runtime
    this.session = null;
    this.lastActions = null;
    this.isInferencing = false;
    // this.modelPath = './examples/checkpoints/20250410_121925/model_14000.onnx';
    // this.modelPath = './examples/checkpoints/policy-05-02_23-55.onnx';
    this.modelPath = './examples/checkpoints/policy-05-03_21-31.onnx';

    // Pre-allocate reusable objects to avoid garbage collection
    this.lastSimState = {
      bodies: new Map(),
      lights: new Map(),
      tendons: {
        numWraps: 0,
        matrix: new THREE.Matrix4()
      }
    };
    
    this.renderer.setAnimationLoop(this.render.bind(this));
    
    // Start the main simulation loop
    this.main_loop();
  }

  async init() {
    // Download the examples first
    await downloadExampleScenesFolder(mujoco);

    // // Now load the initial model
    console.log("initialScene", initialScene);
    this.model = new mujoco.Model("/working/" + initialScene);
    this.state = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // Initialize the three.js Scene using the .xml Model
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);
    
      // Parse joint names
    console.log("model.njnt", this.model.njnt);
    const textDecoder = new TextDecoder();
    const names_array = new Uint8Array(this.model.names);

    this.qposAdr = [];
    this.qvelAdr = [];
    for (let j = 0; j < this.model.njnt; j++) {
      let start_idx = this.model.name_jntadr[j];
      let end_idx = start_idx;
      while (end_idx < names_array.length && names_array[end_idx] !== 0) {
        end_idx++;
      }
      let name = textDecoder.decode(names_array.subarray(start_idx, end_idx));
      if (this.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE.value) {
        this.jointNamesMJC.push(name);
        this.qposAdr.push(this.model.jnt_qposadr[j]);
        this.qvelAdr.push(this.model.jnt_dofadr[j]);
      };
    }
    console.log("parsed jointName", this.jointNamesMJC);

    const asset_meta = await fetch("./examples/checkpoints/asset_meta.json").then(response => response.json());
    this.jointNamesIsaac = asset_meta["joint_names_isaac"];
    this.isaac2mjc = this.jointNamesMJC.map(name => this.jointNamesIsaac.indexOf(name));
    this.mjc2isaac = this.jointNamesIsaac.map(name => this.jointNamesMJC.indexOf(name));
    this.numActions = this.jointNamesIsaac.length;
    this.defaultJpos = new Float32Array(asset_meta["default_joint_pos"]);
    this.jntKp = new Float32Array(this.numActions).fill(25.);
    this.jntKd = new Float32Array(this.numActions).fill(0.5);
    this.timestep = this.model.getOptions().timestep;
    this.decimation = Math.round(0.02 / this.timestep);
    this.simStepCount = 0;
    this.simTimeStamp = performance.now();
    this.inferenceStepCount = 0;
    this.actionBuffer = new Array(4).fill().map(() => new Float32Array(this.numActions));

    console.log("timestep:", this.timestep, "decimation:", this.decimation);

    this.gui = new GUI();
    setupGUI(this);

    this.policy = new ONNXModule(this.modelPath);
    await this.policy.init();
    this.adapt_hx = new Float32Array(128);
    this.rpy = new THREE.Euler();

    // console.log(this.model.timeMS);

    // set up observations
    this.observations = {
      // base_angvel_multistep: new BaseAngVelMultistep(this.model, this.simulation, this, "free_joint", steps=3),
      gravity_multistep: new GravityMultistep(this.model, this.simulation, this, "free_joint", 3),
      joint_pos_multistep: new JointPosMultistep(this.model, this.simulation, this, this.jointNamesIsaac, 3),
      joint_vel_multistep: new JointVelMultistep(this.model, this.simulation, this, this.jointNamesIsaac, 3),
      prev_actions: new PrevActions(this.model, this.simulation, this, 3)
    };
  }

  async main_loop() {
    while (true) {
      const loopStart = performance.now();

      if (!this.params["paused"] && this.model && this.state && this.simulation && this.observations) {
        // Run policy inference
        if (this.simStepCount % this.decimation == 0) {
          const quat = this.simulation.qpos.subarray(3, 7);
          this.quat = new THREE.Quaternion(quat[1], quat[2], quat[3], quat[0]);
          this.rpy.setFromQuaternion(this.quat);
          const command = this.getCommand();
          const policy = this.getObservations(this.simulation);
          await this.runInference(command, policy);
        }

        // Apply control torque
        if (this.lastActions) {
          const jpos = this.qposAdr.map(adr => this.simulation.qpos[adr]);
          const jvel = this.qvelAdr.map(adr => this.simulation.qvel[adr]);
          for (let i = 0; i < this.simulation.ctrl.length; i++) {
            const j = this.isaac2mjc[i];
            const targetJpos = 0.5 * this.lastActions[j] + this.defaultJpos[j];
            const torque = this.jntKp[i] * (targetJpos - jpos[i]) + this.jntKd[i] * (0 - jvel[i]);
            this.simulation.ctrl[i] = torque;
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
              getPosition  (this.simulation.xpos , b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);
        }

        // Step simulation
        this.simulation.step();
        this.mujoco_time += this.timestep * 1000.0;
        this.simStepCount += 1;

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
              let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
              let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
              let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

              let validStart = tendonStart.length() > 0.01;
              let validEnd   = tendonEnd  .length() > 0.01;

              if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
              if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
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
      }

      // Calculate time to sleep
      const loopEnd = performance.now();
      const elapsed = (loopEnd - loopStart) / 1000;
      const sleepTime = Math.max(0, this.timestep - elapsed);

      // calculate actual frequency
      if ((this.simStepCount / this.decimation) % 50 == 0){
        const actualFreq = 1 / (elapsed + sleepTime);
        console.log("elapsed", elapsed);
        console.log("timestep", this.timestep);
        console.log("sleepTime", sleepTime);
        console.log("main loop frequency:", actualFreq);
      }

      await new Promise(resolve => setTimeout(resolve, sleepTime * 1000));
    }
  }

  async runInference(command, policy) {
    if (!this.policy || this.isInferencing) {
      console.log("inference lag");
      return;
    }

    const inferenceStart = performance.now();

    this.isInferencing = true;
    this.inferenceStepCount += 1;
    try {
      const input = {
        "command": new ort.Tensor('float32', new Float32Array(command), [1, 27]),
        "policy": new ort.Tensor('float32', new Float32Array(policy), [1, policy.length]),
        "is_init": new ort.Tensor('bool', [false], [1]),
        "adapt_hx": new ort.Tensor('float32', this.adapt_hx, [1, 128])
      }
      const result = await this.policy.runInference(input);
      this.lastActions = result["action"];
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

  getCommand(simulation) {
    const omega = 4.0 * Math.PI
    const time = this.mujoco_time / 1000.;
    // const time = 0.;
    const phase = [
      omega * time + Math.PI,
      omega * time,
      omega * time,
      omega * time + Math.PI,
    ];
    const osc = [...phase.map(Math.sin), ...phase.map(Math.cos), omega, omega, omega, omega];
    const setpoint = new THREE.Vector3(0, 0, 0);
    const base_pos_w = new THREE.Vector3(...this.simulation.qpos.subarray(0, 3));
    const kp = 12.;
    const kd = 1.8 * Math.sqrt(kp);
    const mass = 1.;
    let setpoint_b = setpoint.sub(base_pos_w).applyQuaternion(this.quat.invert());

    const command = [
      setpoint_b.x, setpoint_b.y,
      0. - this.rpy.y,
      kp * setpoint_b.x, kp * setpoint_b.y,
      kd, kd, kd,
      kp * (0. - this.rpy.y),
      mass,
      kp * setpoint_b.x / mass, kp * setpoint_b.y / mass,
      kd / mass, kd / mass, kd / mass
    ];
    return [...command, ...osc];
  }

  getObservations(simulation) {
    let allObservations = [];
    for (const obs_func of Object.values(this.observations)) {
      const obs = obs_func.compute(simulation);
      if (obs.some(isNaN)) {
        console.log("NaN in observation", obs_func.constructor.name);
      }
      allObservations.push(...obs);
    }
    return allObservations;
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
      const {numWraps} = this.lastSimState.tendons.numWraps;
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
      console.log("render time", render_time);
    }
  }
}

let demo = new MuJoCoDemo();
await demo.init();
