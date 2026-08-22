
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { VRButton         } from '../node_modules/three/examples/jsm/webxr/VRButton.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { XRInputManager    } from './utils/XRInputManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, drawTendonsAndFlex, updateSleepState, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import   load_mujoco        from '../node_modules/@mujoco/mujoco/mujoco.js';

// Load the MuJoCo Module
// The .wasm binary ships separately from the .js loader, so point Emscripten
// at it explicitly; this resolves correctly from both ./src and the esbuild bundle.
const mujoco = await load_mujoco({
  locateFile: (path, prefix) => path.endsWith(".wasm") ?
    new URL('../node_modules/@mujoco/mujoco/mujoco.wasm', import.meta.url).href : prefix + path
});

// Set up Emscripten's Virtual File System
var initialScene = "humanoid.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/" + initialScene, await(await fetch("./assets/scenes/" + initialScene)).text());

// Rotation of -90 degrees about x (in MuJoCo axes), aligning the teleoperated
// gripper's +z (finger direction) with the hand's pointing direction.
const teleopGripAlignment = new THREE.Quaternion(-Math.SQRT1_2, 0, 0, Math.SQRT1_2);

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML
    this.model = mujoco.MjModel.mj_loadXML("/working/" + initialScene);
    this.data  = new mujoco.MjData(this.model);

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0 };
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(2.0, 1.7, 1.7);

    // Camera rig: identity outside VR (so OrbitControls sees plain world
    // coordinates), repositioned on session start so the viewer stands a
    // couple of meters back from the scene instead of inside it.
    this.cameraRig = new THREE.Group();
    this.cameraRig.name = 'CameraRig';
    this.cameraRig.add(this.camera);
    this.scene.add(this.cameraRig);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 * 3.14 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle = 1.11;
    this.spotlight.distance = 10000;
    this.spotlight.penumbra = 0.5;
    this.spotlight.castShadow = true; // default false
    this.spotlight.intensity = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width = 1024; // default
    this.spotlight.shadow.mapSize.height = 1024; // default
    this.spotlight.shadow.camera.near = 0.1; // default
    this.spotlight.shadow.camera.far = 100; // default
    this.spotlight.position.set(0, 3, 3);
    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add( this.spotlight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio(1.0);////window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    THREE.ColorManagement.enabled = false;
    this.renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
    //this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    //this.renderer.toneMappingExposure = 2.0;

    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    // WebXR / VR support. The button reads "VR NOT SUPPORTED" on devices
    // without an immersive-vr runtime; entering VR hands camera pose to the
    // headset while the rig places the viewer 2m back from the scene.
    this.renderer.xr.enabled = true;
    document.body.appendChild(VRButton.createButton(this.renderer, { optionalFeatures: ['hand-tracking'] }));
    this.renderer.xr.addEventListener('sessionstart', () => {
      // Stand closer in hand-teleop scenes so the robot is within arm's reach.
      this.cameraRig.position.set(0, 0, this.teleop ? 1.0 : 2.0);
      this.cameraRig.rotation.set(0, 0, 0);
    });
    this.renderer.xr.addEventListener('sessionend', () => {
      // Restore the desktop camera; the headset overwrote its transform.
      this.cameraRig.position.set(0, 0, 0);
      this.camera.position.set(2.0, 1.7, 1.7);
      this.controls.target.set(0, 0.7, 0);
      this.controls.update();
    });

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.7, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);

    // Initialize VR controller / hand-tracking input. `teleop` is set by
    // loadSceneFromURL when the scene has a "hand_target" mocap body.
    this.teleop = null;
    this.xrInput = new XRInputManager(this);
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Initialize the three.js Scene using the .xml Model in initialScene
    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    this.gui = new GUI();
    setupGUI(this);
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    // In VR the headset owns the camera pose.
    if (!this.renderer.xr.isPresenting) { this.controls.update(); }
    else { this.xrInput.updateFrame(); }

    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      // Cap the physics catch-up debt at 35ms; clamping to (timeMS - 35) rather
      // than timeMS ensures at least some steps run even after a slow frame.
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS - 35.0; }

      // Hand teleoperation: drive the mocap target from the VR hand pose,
      // and the gripper actuator from the pinch diameter (hands) or the
      // trigger (controllers). The weld constraint does the rest.
      if (this.teleop && this.renderer.xr.isPresenting) {
        let src = this.xrInput.getTeleopSource();
        if (src) {
          let mocapId = this.model.body_mocapid[this.teleop.bodyID];
          let pos = toMujocoPos(src.position.clone());
          // Rate-limit the target's travel so tracking jumps (entering VR,
          // tracking reacquisition) sweep the arm smoothly instead of
          // yanking it across the workspace; human-speed motion stays 1:1.
          let dt = Math.min((timeMS - (this.lastTeleopMS ?? timeMS)) / 1000.0, 0.1);
          this.lastTeleopMS = timeMS;
          let maxStep = 1.5 * dt; // meters, at 1.5 m/s
          this.tmpVec.set(
            pos.x - this.data.mocap_pos[(mocapId * 3) + 0],
            pos.y - this.data.mocap_pos[(mocapId * 3) + 1],
            pos.z - this.data.mocap_pos[(mocapId * 3) + 2]);
          if (this.tmpVec.length() > maxStep) { this.tmpVec.setLength(maxStep); }
          this.data.mocap_pos[(mocapId * 3) + 0] += this.tmpVec.x;
          this.data.mocap_pos[(mocapId * 3) + 1] += this.tmpVec.y;
          this.data.mocap_pos[(mocapId * 3) + 2] += this.tmpVec.z;
          // Quaternion vector parts map to MuJoCo axes like positions do
          // ((x, y, z) -> (x, -z, y)); then rotate -90 degrees about local x
          // so the gripper's +z (finger direction) points where the hand points.
          this.tmpQuat.set(src.quaternion.x, -src.quaternion.z, src.quaternion.y, src.quaternion.w);
          this.tmpQuat.multiply(teleopGripAlignment);
          this.data.mocap_quat[(mocapId * 4) + 0] = this.tmpQuat.w;
          this.data.mocap_quat[(mocapId * 4) + 1] = this.tmpQuat.x;
          this.data.mocap_quat[(mocapId * 4) + 2] = this.tmpQuat.y;
          this.data.mocap_quat[(mocapId * 4) + 3] = this.tmpQuat.z;
          if (this.teleop.gripperActId >= 0) {
            let lo = this.model.actuator_ctrlrange[(this.teleop.gripperActId * 2) + 0];
            let hi = this.model.actuator_ctrlrange[(this.teleop.gripperActId * 2) + 1];
            let closed = null; // 0 = open, 1 = closed
            if      (src.aperture != null) { closed = Math.min(Math.max((0.07 - src.aperture) / 0.055, 0.0), 1.0); }
            else if (src.trigger  != null) { closed = src.trigger; }
            if (closed != null) { this.data.ctrl[this.teleop.gripperActId] = lo + closed * (hi - lo); }
          }
        }
      }

      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.data.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones. Mouse drags and XR
        // pinch/trigger grabs share the same spring-force treatment.
        for (let i = 0; i < this.data.qfrc_applied.length; i++) { this.data.qfrc_applied[i] = 0.0; }
        let drags = [];
        if (this.dragStateManager.physicsObject && this.dragStateManager.physicsObject.bodyID) { drags.push(this.dragStateManager); }
        drags.push(...this.xrInput.activeGrabs());
        if (drags.length > 0) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.data.xpos , b, this.bodies[b].position);
              getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          for (let drag of drags) {
            let bodyID = drag.physicsObject.bodyID;
            drag.update(); // Update the world-space force origin
            let force = toMujocoPos(drag.currentWorld.clone().sub(drag.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
            let point = toMujocoPos(drag.worldHit.clone());
            mujoco.mj_applyFT(this.model, this.data, [force.x, force.y, force.z], [0, 0, 0], [point.x, point.y, point.z], bodyID, this.data.qfrc_applied);
          }

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        mujoco.mj_step(this.model, this.data);

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.data.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.data.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.data.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.data.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        }
      }

      mujoco.mj_forward(this.model, this.data);
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.data.xpos , b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Draw Tendons and Flex verts
    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);

    // Tint sleeping bodies blue.
    updateSleepState(this.bodies, this.model, this.data);

    // Render!
    this.renderer.render( this.scene, this.camera );
  }
}

let demo = new MuJoCoDemo();
await demo.init();

// Expose for debugging / scripting from the console.
window.demo = demo;
