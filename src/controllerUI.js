// Browser-side UI for the analytic controller: lil-gui folder, keyboard driving, follow camera, algorithm markers
// (capture point / CoP / next foothold), random pushes and a HUD. All controller maths lives in ./controllers/.
import * as THREE from 'three';
import { BEHAVIOUR_NAMES, BehaviourRunner, SwordPlayer, MOVE_NAMES, hasSword } from './controllers/index.js';

const STOCK_ARMS = { armPose: [0, 0, -0.6], armSwing: 0 };          // arms forward: the preset that copes with stock hip motors
const DEFAULT_ARMS = { armPose: [0.65, -1.05, -0.6], armSwing: 0.8 };
const KEY_SPEED = { forward: 0.5, back: -0.3, turn: 0.6, strafe: 0.2, fast: 0.8 };

/** MuJoCo world point -> three.js (y-up) vector. */
const toThree = (p, z, target) => target.set(p[0], z ?? p[2], -p[1]);

export function setupControllerUI(demo) {
  const P = demo.ctrlParams;
  const keys = new Set();
  const runner = new BehaviourRunner(P.behaviour);
  const sword = new SwordPlayer();
  let swordOn = false;
  let fallenSince = null, push = null, lastTorso = null;

  // ------------------------------------------------------------------ GUI
  const folder = demo.gui.addFolder('Analytic Controller (no neural net)');
  folder.add(P, 'enabled').name('Enabled').onChange((on) => {
    demo.controller?.reset();
    if (!on) demo.data.ctrl.fill(0);   // hand the motors back: the humanoid ragdolls like the original demo
    updateVisibility();
  });
  folder.add(P, 'behaviour', BEHAVIOUR_NAMES).name('Behaviour').onChange((b) => runner.select(b, demo.data.time));
  folder.add(P, 'vx', -0.4, 0.9, 0.05).name('Forward m/s (W/S)').listen();
  folder.add(P, 'vy', -0.25, 0.25, 0.05).name('Sideways m/s (Q/E)').listen();
  folder.add(P, 'wz', -0.8, 0.8, 0.05).name('Turn rad/s (A/D)').listen();
  folder.add(P, 'height', 0.64, 0.82, 0.01).name('Stand height m');
  folder.add(P, 'hipBoost', 1, 2, 0.25).name('Hip motor × (1 = stock, fragile)').onChange(applyHipBoost);
  folder.add({ stop: () => { P.vx = P.vy = P.wz = 0; } }, 'stop').name('Zero command (X)');
  folder.add({ push: () => randomPush() }, 'push').name('Random push (P)');
  folder.add(P, 'followCamera').name('Follow camera');
  folder.add(P, 'showMarkers').name('Show markers').onChange(updateVisibility);
  folder.add(P, 'autoReset').name('Auto-reset after fall');
  folder.open();

  // Sword techniques (only for humanoid_sword.xml)
  const swordFolder = demo.gui.addFolder('Sword techniques (keys 1-5, K = kata)');
  MOVE_NAMES.forEach((name, i) => swordFolder.add({ go: () => sword.trigger(name) }, 'go').name(`${i + 1}  ${name}`));
  swordFolder.add(sword, 'kata').name('Kata (loop all techniques)').listen();
  swordFolder.open();

  function applyHipBoost() {
    const c = demo.controller; if (!c) return;
    c.setHipBoost(P.hipBoost);
    Object.assign(c.P, P.hipBoost < 1.25 ? STOCK_ARMS : DEFAULT_ARMS);
  }

  // ------------------------------------------------------------------ keyboard
  const DRIVE = new Set(['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);
  document.addEventListener('keydown', (e) => {
    if (e.ctrlKey || e.metaKey || e.altKey || e.target.tagName === 'INPUT' || !demo.controller) return;   // Ctrl+A / Ctrl+L stay with the demo
    if (DRIVE.has(e.code)) { keys.add(e.code); e.preventDefault(); }
    else if (e.code === 'KeyP') randomPush();
    else if (swordOn && /^Digit[1-9]$/.test(e.code) && +e.code.slice(5) <= MOVE_NAMES.length) sword.trigger(MOVE_NAMES[+e.code.slice(5) - 1]);
    else if (swordOn && e.code === 'KeyK') sword.kata = !sword.kata;
    else if (e.code === 'KeyX') { P.vx = P.vy = P.wz = 0; }
  });
  document.addEventListener('keyup', (e) => keys.delete(e.code));
  window.addEventListener('blur', () => keys.clear());

  function userCommand() {
    let vx = P.vx, vy = P.vy, wz = P.wz;
    if (keys.size) {
      const fwd = keys.has('KeyW') || keys.has('ArrowUp'), back = keys.has('KeyS') || keys.has('ArrowDown');
      const left = keys.has('KeyA') || keys.has('ArrowLeft'), right = keys.has('KeyD') || keys.has('ArrowRight');
      if (fwd || back) vx = fwd ? (keys.has('ShiftLeft') ? KEY_SPEED.fast : KEY_SPEED.forward) : KEY_SPEED.back;
      if (left || right) wz = left ? KEY_SPEED.turn : -KEY_SPEED.turn;
      if (keys.has('KeyQ')) vy = KEY_SPEED.strafe; else if (keys.has('KeyE')) vy = -KEY_SPEED.strafe;
    }
    const moving = Math.abs(vx) > 1e-3 || Math.abs(vy) > 1e-3 || Math.abs(wz) > 1e-3;
    return { walk: moving, vx, vy, wz, height: P.height };
  }
  // track Shift too (held with W for the fast gait)
  document.addEventListener('keydown', (e) => { if (e.code === 'ShiftLeft') keys.add('ShiftLeft'); });

  // ------------------------------------------------------------------ perturbations
  function randomPush(N = 110) {
    if (!demo.controller) return;
    const a = Math.random() * 2 * Math.PI;
    push = { until: demo.data.time + 0.1, F: [N * Math.cos(a), N * Math.sin(a), 0], shownUntil: performance.now() + 700 };
  }

  // ------------------------------------------------------------------ markers + HUD
  const markers = new THREE.Group(); markers.name = 'Controller Markers';
  const dot = (color, r) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.9 }));
    m.renderOrder = 10; markers.add(m); return m;
  };
  const dcmDot = dot(0xff3355, 0.028), copDot = dot(0x33ff77, 0.022), comDot = dot(0xffffff, 0.015);
  const target = new THREE.Mesh(new THREE.RingGeometry(0.035, 0.05, 32), new THREE.MeshBasicMaterial({ color: 0x3399ff, side: THREE.DoubleSide, depthTest: false }));
  target.rotation.x = -Math.PI / 2; target.renderOrder = 10; markers.add(target);
  const comLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.03, gapSize: 0.02, depthTest: false, transparent: true, opacity: 0.6 }));
  comLine.renderOrder = 10; markers.add(comLine);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.5, 0xffaa00, 0.12, 0.08);
  arrow.visible = false; markers.add(arrow);
  demo.scene.add(markers);

  const hud = document.createElement('div');
  Object.assign(hud.style, { position: 'absolute', left: '10px', bottom: '10px', color: '#e8eef4', font: '13px/1.45 ui-monospace, Menlo, Consolas, monospace',
    background: 'rgba(10, 18, 28, 0.72)', padding: '8px 12px', borderRadius: '8px', pointerEvents: 'none', whiteSpace: 'pre', maxWidth: '92vw', zIndex: 5 });
  document.body.appendChild(hud);   // appended once at startup (the help menu toggles body.lastChild, so never append later)

  const tmp = new THREE.Vector3(), tmp2 = new THREE.Vector3();
  function updateVisibility() {
    const on = demo.controllerActive;
    markers.visible = on && P.showMarkers;
    hud.style.display = demo.controller ? 'block' : 'none';
    folder.show(!!demo.controller);
    swordFolder.show(!!demo.controller && swordOn);
  }

  // ------------------------------------------------------------------ public API used by main.js
  const api = {
    /** Full controller command for this physics step. */
    command(time) {
      const cmd = runner.command(time, userCommand(), demo.controller);
      return swordOn ? sword.apply(cmd, time, demo.controller, P.height) : cmd;
    },

    /** Apply scripted perturbations (random push) for the coming physics step. */
    beforeStep() {
      const c = demo.controller, d = demo.data; if (!c) return;
      const b = c.idx.B.torso;
      if (push && d.time < push.until) { for (let k = 0; k < 3; k++) d.xfrc_applied[6 * b + k] = push.F[k]; }
      else { for (let k = 0; k < 6; k++) d.xfrc_applied[6 * b + k] = 0; }
    },

    /** Called after a model (re)load. */
    onControllerChanged() {
      lastTorso = null; runner.select(P.behaviour, 0); fallenSince = null; push = null;
      swordOn = !!demo.controller && hasSword(demo.mujoco, demo.model);
      sword.queue.length = 0; sword.current = null; sword.kata = false;
      if (demo.controller) applyHipBoost();
      updateVisibility();
    },

    /** Per rendered frame: auto-reset, follow camera, markers, HUD, live torque sliders. */
    update(timeMS) {
      const c = demo.controller;
      updateVisibility();
      if (!c) return;
      const d = demo.data, info = c.info;

      // auto-reset a few seconds after a fall
      if (info.fallen && demo.controllerActive) {
        fallenSince ??= timeMS;
        if (P.autoReset && timeMS - fallenSince > 2500 && !demo.params.paused) { demo.resetSimulation(); fallenSince = null; lastTorso = null; }
      } else fallenSince = null;

      // follow camera: carry camera + orbit target along with the torso's horizontal motion
      const tb = c.idx.B.torso, tx = d.xpos[3 * tb], ty = d.xpos[3 * tb + 1];
      if (P.followCamera && lastTorso) {
        const dx = tx - lastTorso[0], dz = -(ty - lastTorso[1]);
        if (Math.hypot(dx, dz) < 1.0) {   // ignore teleports (reset)
          demo.camera.position.x += dx; demo.camera.position.z += dz;
          demo.controls.target.x += dx; demo.controls.target.z += dz;
        }
      }
      lastTorso = [tx, ty];

      // markers (MuJoCo world -> three.js)
      if (markers.visible) {
        const g = 0.004;
        if (info.dcm) { dcmDot.visible = true; toThree(info.dcm, g, dcmDot.position); } else dcmDot.visible = false;
        if (info.cop) { copDot.visible = true; toThree(info.cop, g, copDot.position); } else copDot.visible = false;
        if (info.target) { target.visible = true; toThree(info.target, 0.003, target.position); } else target.visible = false;
        toThree(info.com, g, comDot.position);
        const pos = comLine.geometry.attributes.position;
        toThree(info.com, info.com[2], tmp); pos.setXYZ(0, tmp.x, tmp.y, tmp.z); pos.setXYZ(1, tmp.x, g, tmp.z); pos.needsUpdate = true;
        comLine.computeLineDistances();
        if (push && performance.now() < push.shownUntil) {
          arrow.visible = true;
          toThree([d.xpos[3 * tb], d.xpos[3 * tb + 1], d.xpos[3 * tb + 2]], null, tmp);
          tmp2.set(push.F[0], 0, -push.F[1]).normalize();
          arrow.position.copy(tmp).addScaledVector(tmp2, -0.6); arrow.setDirection(tmp2);
        } else arrow.visible = false;
      }

      // live torques in the existing actuator sliders
      for (const n of c.idx.names) demo.params[n] = d.ctrl[c.idx.J[n].a];

      // HUD
      const v = Math.hypot(info.vFiltered[0], info.vFiltered[1]);
      const u = userCommand();
      const mode = !demo.controllerActive ? 'OFF (ragdoll)' : info.fallen ? `FALLEN${P.autoReset ? ' - resetting' : ' - Backspace to reset'}`
        : info.mode === 'walk' ? `walk  (${info.stance} foot stance, phase ${info.phase.toFixed(2)})` : 'stand';
      hud.textContent =
        `Analytic controller - capture point balance + foot placement, no neural net\n` +
        `mode ${mode}\n` +
        `cmd vx ${u.vx.toFixed(2)} vy ${u.vy.toFixed(2)} wz ${u.wz.toFixed(2)}   speed ${v.toFixed(2)} m/s   COM z ${info.com[2].toFixed(2)} m\n` +
        `steps ${info.steps}  capture steps ${info.captureSteps}  saturated motors ${info.satCount}/21  hip x${info.hipBoost}\n` +
        (swordOn ? `sword: ${sword.current || 'guard'}${sword.kata ? '  (kata)' : ''}  |  1 chop  2 horizontal  3 backhand  4 diagonal  5 thrust  K kata\n` : '') +
        `W/S A/D Q/E drive, Shift fast, X stop, P push, drag to shove  |  ` +
        `red = capture point  green = CoP  blue = next foothold`;
    },
  };
  api.onControllerChanged();   // the initial scene's controller was created before this UI existed
  return api;
}
