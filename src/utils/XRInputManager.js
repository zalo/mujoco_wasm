import * as THREE from 'three';
import { XRHandModelFactory } from '../../node_modules/three/examples/jsm/webxr/XRHandModelFactory.js';

const tmpA = new THREE.Vector3();
const tmpB = new THREE.Vector3();
const tmpScale = new THREE.Vector3();

/** One active pinch/trigger grab. Mirrors the interface of DragStateManager
 *  (physicsObject, worldHit, currentWorld, update()) so the physics loop can
 *  treat mouse drags and XR grabs uniformly. */
class XRGrab {
  constructor() {
    this.physicsObject = null;
    this.localHit     = new THREE.Vector3();
    this.worldHit     = new THREE.Vector3();
    this.currentWorld = new THREE.Vector3();
  }
  update() {
    if (this.physicsObject) {
      this.worldHit.copy(this.localHit);
      this.physicsObject.localToWorld(this.worldHit);
    }
  }
}

/** Manages WebXR controllers and articulated hands: renders hand/controller
 *  visuals, lets the user grab nearby dynamic bodies by pinching (hands) or
 *  pulling the trigger (controllers), and exposes a "teleop source" pose used
 *  by hand-teleoperated scenes (e.g. the xArm7). */
export class XRInputManager {
  constructor(demo) {
    this.demo = demo;
    this.slots = [];

    const handModelFactory = new XRHandModelFactory();
    const gripGeometry = new THREE.IcosahedronGeometry(0.015, 2);
    const gripMaterial = new THREE.MeshPhysicalMaterial({ color: 0xdddddd, roughness: 0.3 });

    for (let i = 0; i < 2; i++) {
      const controller = demo.renderer.xr.getController(i);      // target-ray space; receives events
      const grip       = demo.renderer.xr.getControllerGrip(i);  // only posed for physical controllers
      const hand       = demo.renderer.xr.getHand(i);            // only posed for articulated hands
      hand.add(handModelFactory.createHandModel(hand, 'spheres'));
      grip.add(new THREE.Mesh(gripGeometry, gripMaterial));

      // Controllers and hands are posed in the XR reference space, so they
      // must share a parent with the camera for the rig offset to apply.
      demo.cameraRig.add(controller);
      demo.cameraRig.add(grip);
      demo.cameraRig.add(hand);

      const slot = { index: i, controller, grip, hand, inputSource: null, grab: new XRGrab() };
      controller.addEventListener('connected'   , (e) => { slot.inputSource = e.data; });
      controller.addEventListener('disconnected', ( ) => { this.endGrab(slot); slot.inputSource = null; });
      controller.addEventListener('selectstart' , ( ) => { this.startGrab(slot); });
      controller.addEventListener('selectend'   , ( ) => { this.endGrab(slot); });
      this.slots.push(slot);
    }
    this.demo.renderer.xr.addEventListener('sessionend', () => {
      for (let slot of this.slots) { this.endGrab(slot); }
    });
  }

  /** The point (in world space) that grabs happen from: the pinch midpoint
   *  for hands, the grip position for controllers. */
  getGrabPoint(slot, target) {
    const joints = slot.hand.joints;
    if (slot.inputSource && slot.inputSource.hand && joints &&
        joints['index-finger-tip'] && joints['thumb-tip'] && joints['index-finger-tip'].visible) {
      joints['index-finger-tip'].getWorldPosition(tmpA);
      joints['thumb-tip'      ].getWorldPosition(target);
      return target.add(tmpA).multiplyScalar(0.5);
    }
    return slot.grip.getWorldPosition(target);
  }

  /** Thumb-tip to index-tip distance in meters, or null without hand tracking. */
  getPinchAperture(slot) {
    const joints = slot.hand.joints;
    if (slot.inputSource && slot.inputSource.hand && joints &&
        joints['index-finger-tip'] && joints['thumb-tip'] && joints['index-finger-tip'].visible) {
      joints['index-finger-tip'].getWorldPosition(tmpA);
      joints['thumb-tip'      ].getWorldPosition(tmpB);
      return tmpA.distanceTo(tmpB);
    }
    return null;
  }

  /** Analog trigger value 0..1, or null when no gamepad is exposed. */
  getTriggerValue(slot) {
    const gamepad = slot.inputSource ? slot.inputSource.gamepad : null;
    if (gamepad && gamepad.buttons && gamepad.buttons.length > 0) { return gamepad.buttons[0].value; }
    return null;
  }

  /** In teleop scenes, the input driving the robot (right hand preferred). */
  getTeleopSlot() {
    let fallback = null;
    for (let slot of this.slots) {
      if (!slot.inputSource) { continue; }
      if (slot.inputSource.handedness == 'right') { return slot; }
      fallback = slot;
    }
    return fallback;
  }

  /** Pose and grip state of the teleop input, in three.js world space.
   *  Uses the target-ray pose (not the pinch midpoint) so that closing the
   *  fingers doesn't translate the robot mid-grasp. */
  getTeleopSource() {
    const slot = this.getTeleopSlot();
    if (!slot) { return null; }
    return {
      position  : slot.controller.getWorldPosition(new THREE.Vector3()),
      quaternion: slot.controller.getWorldQuaternion(new THREE.Quaternion()),
      aperture  : this.getPinchAperture(slot),
      trigger   : this.getTriggerValue(slot),
      hand      : this.getHandPose(slot),
    };
  }

  /** Full articulated-hand pose in three.js world space: wrist and
   *  middle-knuckle positions, the five fingertips (thumb..pinky), and an
   *  orthonormal hand frame CONSTRUCTED FROM JOINT POSITIONS — fingers
   *  (wrist toward middle knuckle), thumbSide (ring toward index knuckle),
   *  back (their cross product, out of the back of the hand). Building the
   *  frame geometrically avoids relying on the wrist joint's orientation
   *  convention, which differs between runtimes. Null when hand tracking
   *  isn't available. */
  getHandPose(slot) {
    const joints = slot.hand.joints;
    if (!slot.inputSource || !slot.inputSource.hand || !joints) { return null; }
    const names = ['wrist',
                   'thumb-phalanx-proximal', 'index-finger-phalanx-proximal', 'middle-finger-phalanx-proximal',
                   'ring-finger-phalanx-proximal', 'pinky-finger-phalanx-proximal',
                   'thumb-tip', 'index-finger-tip', 'middle-finger-tip', 'ring-finger-tip', 'pinky-finger-tip'];
    const p = [];
    for (const name of names) {
      const joint = joints[name];
      if (!joint || !joint.visible) { return null; }
      p.push(joint.getWorldPosition(new THREE.Vector3()));
    }
    const wrist = p[0], knuckles = p.slice(1, 6), tips = p.slice(6);
    const fingers = knuckles[2].clone().sub(wrist).normalize(); // wrist -> middle knuckle
    const thumbSide = knuckles[1].clone().sub(knuckles[3]);     // ring -> index knuckle
    thumbSide.addScaledVector(fingers, -thumbSide.dot(fingers)).normalize();
    const back = fingers.clone().cross(thumbSide); // out of the back of the hand
    return { wristPos: wrist, knucklePos: knuckles[2], knuckles: knuckles, tips: tips,
             thumbSide: thumbSide, back: back, fingers: fingers };
  }

  startGrab(slot) {
    // The hand driving a teleoperated robot pinches to close the gripper;
    // don't let it also grab (usually the robot itself).
    if (this.demo.teleop && slot == this.getTeleopSlot()) { return; }
    if (!this.demo.mujocoRoot) { return; }

    const grabPoint = this.getGrabPoint(slot, new THREE.Vector3());
    let best = null, bestDist = 0.15; // max reach beyond a body's bounding sphere
    this.demo.mujocoRoot.traverse((obj) => {
      if (!obj.isMesh || !(obj.bodyID > 0) || obj.isInstancedMesh) { return; }
      if (!obj.geometry.boundingSphere) { obj.geometry.computeBoundingSphere(); }
      tmpA.copy(obj.geometry.boundingSphere.center).applyMatrix4(obj.matrixWorld);
      obj.getWorldScale(tmpScale);
      const radius = obj.geometry.boundingSphere.radius *
        Math.max(Math.abs(tmpScale.x), Math.abs(tmpScale.y), Math.abs(tmpScale.z));
      if (radius > 2.0) { return; } // skip room-scale geometry like floor planes
      const dist = grabPoint.distanceTo(tmpA) - radius;
      if (dist < bestDist) { bestDist = dist; best = obj; }
    });
    if (!best) { return; }

    // Anchor the grab at the grab point, clamped into the bounding sphere.
    if (!best.geometry.boundingSphere) { best.geometry.computeBoundingSphere(); }
    tmpA.copy(best.geometry.boundingSphere.center).applyMatrix4(best.matrixWorld);
    best.getWorldScale(tmpScale);
    const radius = best.geometry.boundingSphere.radius *
      Math.max(Math.abs(tmpScale.x), Math.abs(tmpScale.y), Math.abs(tmpScale.z));
    const anchor = grabPoint.clone().sub(tmpA);
    if (anchor.length() > radius) { anchor.setLength(radius); }
    anchor.add(tmpA);

    const grab = slot.grab;
    grab.physicsObject = best;
    grab.localHit.copy(best.worldToLocal(anchor.clone()));
    grab.worldHit.copy(anchor);
    grab.currentWorld.copy(grabPoint);
    if (best.material && best.material.emissive) { best.material.emissive.setHex(0x553300); }
  }

  endGrab(slot) {
    const grab = slot.grab;
    if (grab.physicsObject && grab.physicsObject.material && grab.physicsObject.material.emissive) {
      grab.physicsObject.material.emissive.setHex(0x000000);
    }
    grab.physicsObject = null;
  }

  /** Called once per frame to track the grab targets to the current hand pose. */
  updateFrame() {
    for (let slot of this.slots) {
      if (slot.grab.physicsObject) { this.getGrabPoint(slot, slot.grab.currentWorld); }
    }
  }

  /** Active grabs on dynamic bodies, for the physics perturbation loop. */
  activeGrabs() {
    let grabs = [];
    for (let slot of this.slots) {
      if (slot.grab.physicsObject && slot.grab.physicsObject.bodyID) { grabs.push(slot.grab); }
    }
    return grabs;
  }
}
