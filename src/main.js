
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { VRButton         } from '../node_modules/three/examples/jsm/webxr/VRButton.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { XRInputManager    } from './utils/XRInputManager.js';
import { DiffIK            } from './utils/DiffIK.js';
import { HandRetarget      } from './utils/HandRetarget.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, drawTendonsAndFlex, updateSleepState, applyTeleopHomeKeyframe, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
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

// Rotation of 180 degrees about y, aligning the Shadow Hand's palm-site
// frame (+z fingers, +x thumb side, -y palm) with the WebXR wrist frame
// (-z fingers, -x thumb side, -y palm).
const palmAlignment = new THREE.Quaternion(0, 1, 0, 0);

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
      // In teleop scenes the user stands AT the robot's mounting column,
      // yawed 180 degrees so they face the cube table, and the robot arm
      // works where their own arm does (absolute-coordinate tracking).
      if (this.teleop) {
        this.cameraRig.position.set(0, 0, -0.25);
        this.cameraRig.rotation.set(0, Math.PI, 0);
      } else {
        this.cameraRig.position.set(0, 0, 2.0);
        this.cameraRig.rotation.set(0, 0, 0);
      }
      // Reset the scene and recalibrate the teleop origin, now and whenever
      // the user recenters their headset (long-press the system button).
      this.pendingSceneReset = true;
      const refSpace = this.renderer.xr.getReferenceSpace();
      if (refSpace && refSpace.addEventListener) {
        refSpace.addEventListener('reset', () => { this.pendingSceneReset = true; });
      }
    });
    this.renderer.xr.addEventListener('sessionend', () => {
      // Restore the desktop camera; the headset overwrote its transform.
      this.cameraRig.position.set(0, 0, 0);
      this.cameraRig.rotation.set(0, 0, 0);
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
    // loadSceneFromURL when the scene has a "hand_target" mocap body; `ik`
    // and `handRetarget` are created lazily for scenes that support
    // actuator-driven IK. `teleopOrigin` is the delta-teleop calibration,
    // captured when the hand is first tracked and cleared on session start
    // and on headset recentering.
    this.teleop = null;
    this.ik = null;
    this.handRetarget = null;
    this.teleopOrigin = null;
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

  /** Fingertip calibration: capture the neutral fingertip layout (with
   *  per-finger human-to-robot scale factors) so fingertip motion retargets
   *  as scaled deltas around this pose. */
  captureTeleopOrigin(handPose, mocapId) {
    const sid = this.teleop.tcpSiteId;
    const sp = this.data.site_xpos, sm = this.data.site_xmat;
    const invWrist = handPose.wristQuat.clone().invert();
    const palmRoot = [0, 0.035, -0.09]; // approx. palm base in the palm-site frame
    let tipNeutral = [], tipHome = [], scale = [];
    for (let f = 0; f < 5; f++) {
      const rel = handPose.tips[f].clone().sub(handPose.wristPos).applyQuaternion(invWrist);
      const neutral = [-rel.x, rel.y, -rel.z]; // WebXR wrist frame -> palm-site axes
      tipNeutral.push(neutral);
      let home = [0, 0, 0];
      if (this.teleop.hand) {
        const tb = this.teleop.hand.tipBodyIds[f];
        const d = [
          this.data.xpos[(tb * 3) + 0] - sp[(sid * 3) + 0],
          this.data.xpos[(tb * 3) + 1] - sp[(sid * 3) + 1],
          this.data.xpos[(tb * 3) + 2] - sp[(sid * 3) + 2]];
        home = [ // R^T * d (site_xmat is row-major local-to-world)
          sm[(sid * 9) + 0] * d[0] + sm[(sid * 9) + 3] * d[1] + sm[(sid * 9) + 6] * d[2],
          sm[(sid * 9) + 1] * d[0] + sm[(sid * 9) + 4] * d[1] + sm[(sid * 9) + 7] * d[2],
          sm[(sid * 9) + 2] * d[0] + sm[(sid * 9) + 5] * d[1] + sm[(sid * 9) + 8] * d[2]];
      }
      tipHome.push(home);
      const robotLen = Math.hypot(home[0] - palmRoot[0], home[1] - palmRoot[1], home[2] - palmRoot[2]);
      const humanLen = Math.max(Math.hypot(neutral[0], neutral[1], neutral[2]), 0.02);
      // The 1.25 boost over the pure length ratio deepens the retargeted
      // curls; without it the robot fingers visibly under-close.
      scale.push(Math.min(Math.max(1.25 * robotLen / humanLen, 0.6), 2.0));
    }
    return { tipNeutral: tipNeutral, tipHome: tipHome, scale: scale };
  }

  /** Fingertip targets (MuJoCo world) for the dexterous hand: the user's
   *  fingertip deltas from the calibration pose, expressed in the wrist
   *  frame, scaled per finger, and re-rooted in the robot's palm frame. */
  computeFingerTargets(handPose) {
    const o = this.teleopOrigin, sid = this.teleop.tcpSiteId;
    const sp = this.data.site_xpos, sm = this.data.site_xmat;
    const invWrist = handPose.wristQuat.clone().invert();
    const targets = [];
    for (let f = 0; f < 5; f++) {
      const rel = handPose.tips[f].clone().sub(handPose.wristPos).applyQuaternion(invWrist);
      const local = [
        o.tipHome[f][0] + o.scale[f] * ((-rel.x) - o.tipNeutral[f][0]),
        o.tipHome[f][1] + o.scale[f] * (( rel.y) - o.tipNeutral[f][1]),
        o.tipHome[f][2] + o.scale[f] * ((-rel.z) - o.tipNeutral[f][2])];
      targets.push([
        sp[(sid * 3) + 0] + sm[(sid * 9) + 0] * local[0] + sm[(sid * 9) + 1] * local[1] + sm[(sid * 9) + 2] * local[2],
        sp[(sid * 3) + 1] + sm[(sid * 9) + 3] * local[0] + sm[(sid * 9) + 4] * local[1] + sm[(sid * 9) + 5] * local[2],
        sp[(sid * 3) + 2] + sm[(sid * 9) + 6] * local[0] + sm[(sid * 9) + 7] * local[1] + sm[(sid * 9) + 8] * local[2]]);
    }

    // Pinch refinement (DexPilot-style): scaled delta retargeting alone
    // leaves a residual thumb-index gap, so as the user's pinch closes
    // below 4cm, blend the thumb and index targets toward a shared midpoint
    // separated by exactly the human gap — making robot fingertips meet
    // when the user's do.
    const humanGap = handPose.tips[0].distanceTo(handPose.tips[1]);
    if (humanGap < 0.04) {
      const w = Math.min((0.04 - humanGap) / 0.02, 1.0);
      const t0 = targets[0], t1 = targets[1];
      const mid = [(t0[0] + t1[0]) / 2, (t0[1] + t1[1]) / 2, (t0[2] + t1[2]) / 2];
      const len = Math.max(Math.hypot(t0[0] - t1[0], t0[1] - t1[1], t0[2] - t1[2]), 1e-6);
      for (let r = 0; r < 3; r++) {
        const dir = (t0[r] - t1[r]) / len;
        t0[r] = t0[r] * (1 - w) + (mid[r] + dir * humanGap / 2) * w;
        t1[r] = t1[r] * (1 - w) + (mid[r] - dir * humanGap / 2) * w;
      }
    }
    return targets;
  }

  render(timeMS) {
    // In VR the headset owns the camera pose.
    if (!this.renderer.xr.isPresenting) { this.controls.update(); }
    else { this.xrInput.updateFrame(); }

    // Headset recenter / session start: put the scene back to its initial
    // state and recalibrate the teleop mapping.
    if (this.pendingSceneReset) {
      this.pendingSceneReset = false;
      if (this.teleop) {
        mujoco.mj_resetData(this.model, this.data);
        applyTeleopHomeKeyframe(mujoco, this.model, this.data);
        this.teleopOrigin = null;
        if (this.ik) { this.ik.reset(); }
        if (this.handRetarget) { this.handRetarget.reset(); }
      }
    }

    if (!this.params["paused"]) {
      let timestep = this.model.opt.timestep;
      // Cap the physics catch-up debt at 35ms; clamping to (timeMS - 35) rather
      // than timeMS ensures at least some steps run even after a slow frame.
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS - 35.0; }

      // Hand teleoperation: the VR hand drives the mocap target marker
      // (rate-limited), and differential IK drives the arm's position
      // actuators toward it with command-level ground-clearance safeties.
      // The gripper follows the pinch diameter (hands) or trigger.
      if (this.teleop) {
        let mocapId = this.model.body_mocapid[this.teleop.bodyID];
        let dt = Math.min((timeMS - (this.lastTeleopMS ?? timeMS)) / 1000.0, 0.1);
        this.lastTeleopMS = timeMS;

        let handPose = null;
        if (this.renderer.xr.isPresenting) {
          let src = this.xrInput.getTeleopSource();
          if (src) {
            let pos = null;
            if (this.teleop.hand && src.hand) {
              // Dexterous-hand scenes track the wrist's ABSOLUTE pose: the
              // user stands inside the robot's workspace, so the robot palm
              // goes exactly where their hand is.
              handPose = src.hand;
              pos = toMujocoPos(handPose.wristPos.clone());
              this.tmpQuat.set(handPose.wristQuat.x, -handPose.wristQuat.z, handPose.wristQuat.y, handPose.wristQuat.w);
              this.tmpQuat.multiply(palmAlignment);
              // Fingertip retargeting still needs a calibration snapshot of
              // the neutral hand; wait a few tracked frames first (the
              // first frames can carry a stale rig transform or
              // tracking-acquisition glitches).
              if (!this.teleopOrigin) {
                this.teleopOriginCountdown = (this.teleopOriginCountdown ?? 15) - 1;
                if (this.teleopOriginCountdown <= 0) {
                  this.teleopOrigin = this.captureTeleopOrigin(handPose, mocapId);
                  this.teleopOriginCountdown = null;
                }
              }
            } else if (!this.teleop.hand) {
              // Gripper scenes track the hand/controller position absolutely.
              pos = toMujocoPos(src.position.clone());
              // Quaternion vector parts map to MuJoCo axes like positions do
              // ((x, y, z) -> (x, -z, y)); then rotate -90 degrees about local x
              // so the gripper's +z (finger direction) points where the hand points.
              this.tmpQuat.set(src.quaternion.x, -src.quaternion.z, src.quaternion.y, src.quaternion.w);
              this.tmpQuat.multiply(teleopGripAlignment);
            }
            if (pos != null) {
              // Rate-limit the target's travel so tracking jumps (entering VR,
              // tracking reacquisition) sweep the arm smoothly instead of
              // yanking it across the workspace; human-speed motion stays 1:1.
              let maxStep = 3.0 * dt; // meters, at 3 m/s
              this.tmpVec.set(
                pos.x - this.data.mocap_pos[(mocapId * 3) + 0],
                pos.y - this.data.mocap_pos[(mocapId * 3) + 1],
                pos.z - this.data.mocap_pos[(mocapId * 3) + 2]);
              if (this.tmpVec.length() > maxStep) { this.tmpVec.setLength(maxStep); }
              this.data.mocap_pos[(mocapId * 3) + 0] += this.tmpVec.x;
              this.data.mocap_pos[(mocapId * 3) + 1] += this.tmpVec.y;
              this.data.mocap_pos[(mocapId * 3) + 2] += this.tmpVec.z;
              this.data.mocap_quat[(mocapId * 4) + 0] = this.tmpQuat.w;
              this.data.mocap_quat[(mocapId * 4) + 1] = this.tmpQuat.x;
              this.data.mocap_quat[(mocapId * 4) + 2] = this.tmpQuat.y;
              this.data.mocap_quat[(mocapId * 4) + 3] = this.tmpQuat.z;
            }
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

        // Differential IK toward the (workspace-clamped) target. Runs on
        // desktop too, so the arm rises to the marker on scene load and
        // follows the marker when it is dragged while paused.
        if (this.teleop.tcpSiteId >= 0) {
          if (!this.ik || this.ik.model != this.model) {
            if (this.ik) { this.ik.dispose(); }
            // Center the reach sphere on the arm's mounting point (the
            // chain body closest to the world), wherever the scene put it.
            let rootBody = this.model.site_bodyid[this.teleop.tcpSiteId];
            while (this.model.body_parentid[rootBody] != 0) { rootBody = this.model.body_parentid[rootBody]; }
            this.ik = new DiffIK(mujoco, this.model, {
              siteId: this.teleop.tcpSiteId, armActIds: this.teleop.armActIds,
              reachCenter: [
                this.data.xpos[(rootBody * 3) + 0],
                this.data.xpos[(rootBody * 3) + 1],
                this.data.xpos[(rootBody * 3) + 2] + 0.3],
              reachRadius: 0.95,
            });
          }
          let target = [
            this.data.mocap_pos[(mocapId * 3) + 0],
            this.data.mocap_pos[(mocapId * 3) + 1],
            this.data.mocap_pos[(mocapId * 3) + 2]];
          this.ik.clampTarget(target);
          // Write the clamp back so the marker shows the actual command.
          this.data.mocap_pos[(mocapId * 3) + 0] = target[0];
          this.data.mocap_pos[(mocapId * 3) + 1] = target[1];
          this.data.mocap_pos[(mocapId * 3) + 2] = target[2];
          this.ik.step(this.data, target,
            [...this.data.mocap_quat.slice(mocapId * 4, (mocapId * 4) + 4)], dt);

          // Fingertip retargeting: drive the dexterous hand's actuators so
          // its fingertips track the user's, expressed in the palm frame.
          if (this.teleop.hand && handPose && this.teleopOrigin) {
            if (!this.handRetarget || this.handRetarget.model != this.model) {
              if (this.handRetarget) { this.handRetarget.dispose(); }
              this.handRetarget = new HandRetarget(mujoco, this.model, this.teleop.hand);
            }
            this.handRetarget.step(this.data, this.computeFingerTargets(handPose), dt);
          }

          // Hook for streaming the safety-gated joint commands to real
          // hardware: (armJointTargets: Float64Array, gripperCtrl: number,
          // handActuatorTargets: Float64Array|null).
          if (this.onTeleopCommand) {
            this.onTeleopCommand(this.ik.lastCommand,
              this.teleop.gripperActId >= 0 ? this.data.ctrl[this.teleop.gripperActId] : 0,
              this.handRetarget ? this.handRetarget.lastCommand : null);
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
