
import  *  as  THREE     from 'three';
import { GUI           } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { Reflector     } from './utils/Reflector.js';
import { DragStateManager       } from './utils/DragStateManager.js';
import { downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos } from './mujocoSceneLoader.js';
import   load_mujoco     from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();
// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./examples/scenes/humanoid.xml")).text());

// Load in the state from XML
let model       = new mujoco.Model("/working/humanoid.xml");
let state       = new mujoco.State(model);
let simulation  = new mujoco.Simulation(model, state);

let updateGUICallbacks = [];

let container, controls;
let camera, scene, renderer;
const params = { scene: "humanoid.xml", paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber:0 };
/** @type {DragStateManager} */
let dragStateManager;
let bodies, lights;
let tmpVec  = new THREE.Vector3();
let tmpQuat = new THREE.Quaternion();

const resetCamera = () => {
  // TODO: Use free camera parameters from MuJoCo
  camera.position.set( 2.0, 1.7, 1.7 );
}

async function init() {
  container = document.createElement( 'div' );
  document.body.appendChild( container );

  scene = new THREE.Scene();
  scene.name = 'scene';

  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
  camera.name = 'PerspectiveCamera';
  resetCamera(camera);
  scene.add(camera);
  // Make sure we reset the camera when the scene is changed or reloaded.
  updateGUICallbacks.push((model, simulation, params) => {
    resetCamera(camera);
  });

  scene.background = new THREE.Color(0.15, 0.25, 0.35);
  scene.fog = new THREE.Fog(scene.background, 15, 25.5 );

  const ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
  ambientLight.name = 'AmbientLight';
  scene.add( ambientLight );

  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap ; // default THREE.PCFShadowMap

  container.appendChild( renderer.domElement );

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.7, 0);
  controls.panSpeed = 2;
  controls.zoomSpeed = 1;
  controls.enableDamping = true;
  controls.dampingFactor = 0.10;
  controls.screenSpacePanning = true;
  controls.update();

  window.addEventListener('resize', onWindowResize);

  // Initialize the Drag State Manager.
  dragStateManager = new DragStateManager(scene, renderer, camera, container.parentElement, controls);

  const gui = new GUI();

  // Add scene selection dropdown.
  const reload = () => {
    scene.remove(scene.getObjectByName("MuJoCo Root"));
    loadSceneFromURL(mujoco, params.scene, scene).then((returnArray) => {
      [model, state, simulation, bodies, lights] = returnArray;
      simulation.forward();
      for (let i = 0; i < updateGUICallbacks.length; i++) {
        updateGUICallbacks[i](model, simulation, params);
      }
    });

  };
  gui.add(params, 'scene', { "Humanoid": "humanoid.xml", "Cassie": "agility_cassie/scene.xml", "Hammock": "hammock.xml", "Balloons": "balloons.xml", "Hand": "shadow_hand/scene_right.xml", "Flag": "flag.xml", "Mug": "mug.xml"})
    .name('Example Scene').onChange(_ => { reload(); });

  // Add a help menu.
  // Parameters:
  //  Name: "Help".
  //  When pressed, a help menu is displayed in the top left corner. When pressed again
  //  the help menu is removed.
  //  Can also be triggered by pressing F1.
  // Has a dark transparent background.
  // Has two columns: one for putting the action description, and one for the action key trigger.keyframeNumber
  let keyInnerHTML = '';
  let actionInnerHTML = '';
  const displayHelpMenu = () => {
    if (params.help) {
      const helpMenu = document.createElement('div');
      helpMenu.style.position = 'absolute';
      helpMenu.style.top = '10px';
      helpMenu.style.left = '10px';
      helpMenu.style.color = 'white';
      helpMenu.style.font = 'normal 18px Arial';
      helpMenu.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
      helpMenu.style.padding = '10px';
      helpMenu.style.borderRadius = '10px';
      helpMenu.style.display = 'flex';
      helpMenu.style.flexDirection = 'column';
      helpMenu.style.alignItems = 'center';
      helpMenu.style.justifyContent = 'center';
      helpMenu.style.width = '400px';
      helpMenu.style.height = '400px';
      helpMenu.style.overflow = 'auto';
      helpMenu.style.zIndex = '1000';

      const helpMenuTitle = document.createElement('div');
      helpMenuTitle.style.font = 'bold 24px Arial';
      helpMenuTitle.innerHTML = '';
      helpMenu.appendChild(helpMenuTitle);

      const helpMenuTable = document.createElement('table');
      helpMenuTable.style.width = '100%';
      helpMenuTable.style.marginTop = '10px';
      helpMenu.appendChild(helpMenuTable);

      const helpMenuTableBody = document.createElement('tbody');
      helpMenuTable.appendChild(helpMenuTableBody);

      const helpMenuRow = document.createElement('tr');
      helpMenuTableBody.appendChild(helpMenuRow);

      const helpMenuActionColumn = document.createElement('td');
      helpMenuActionColumn.style.width = '50%';
      helpMenuActionColumn.style.textAlign = 'right';
      helpMenuActionColumn.style.paddingRight = '10px';
      helpMenuRow.appendChild(helpMenuActionColumn);

      const helpMenuKeyColumn = document.createElement('td');
      helpMenuKeyColumn.style.width = '50%';
      helpMenuKeyColumn.style.textAlign = 'left';
      helpMenuKeyColumn.style.paddingLeft = '10px';
      helpMenuRow.appendChild(helpMenuKeyColumn);

      const helpMenuActionText = document.createElement('div');
      helpMenuActionText.innerHTML = actionInnerHTML;
      helpMenuActionColumn.appendChild(helpMenuActionText);

      const helpMenuKeyText = document.createElement('div');
      helpMenuKeyText.innerHTML = keyInnerHTML;
      helpMenuKeyColumn.appendChild(helpMenuKeyText);

      // Close buttom in the top.
      const helpMenuCloseButton = document.createElement('button');
      helpMenuCloseButton.innerHTML = 'Close';
      helpMenuCloseButton.style.position = 'absolute';
      helpMenuCloseButton.style.top = '10px';
      helpMenuCloseButton.style.right = '10px';
      helpMenuCloseButton.style.zIndex = '1001';
      helpMenuCloseButton.onclick = () => {
        helpMenu.remove();
      };
      helpMenu.appendChild(helpMenuCloseButton);

      document.body.appendChild(helpMenu);
    } else {
      document.body.removeChild(document.body.lastChild);
    }
  }
  document.addEventListener('keydown', (event) => {
    if (event.key === 'F1') {
      params.help = !params.help;
      displayHelpMenu();
    }
  });
  keyInnerHTML += 'F1<br>';
  actionInnerHTML += 'Help<br>';

  let simulationFolder = gui.addFolder("Simulation");

  // Add pause simulation checkbox.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Pause Simulation".
  //  When paused, a "pause" text in white is displayed in the top left corner.
  //  Can also be triggered by pressing the spacebar.
  const pauseSimulation = simulationFolder.add(params, 'paused').name('Pause Simulation');
  pauseSimulation.onChange((value) => {
    if (value) {
      const pausedText = document.createElement('div');
      pausedText.style.position = 'absolute';
      pausedText.style.top = '10px';
      pausedText.style.left = '10px';
      pausedText.style.color = 'white';
      pausedText.style.font = 'normal 18px Arial';
      pausedText.innerHTML = 'pause';
      container.appendChild(pausedText);
    } else {
      container.removeChild(container.lastChild);
    }
  });
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space') {
      params.paused = !params.paused;
      pauseSimulation.setValue(params.paused);
    }
  });
  actionInnerHTML += 'Play / Pause<br>';
  keyInnerHTML += 'Space<br>';

  // Add reload model button.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Reload".
  //  When pressed, calls the reload function.
  //  Can also be triggered by pressing ctrl + L.
  simulationFolder.add({reload: () => { reload(); }}, 'reload').name('Reload');
  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey && event.code === 'KeyL') {
      reload();
    }
  });
  actionInnerHTML += 'Reload XML<br>';
  keyInnerHTML += 'Ctrl L<br>';

  // Add reset simulation button.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Reset".
  //  When pressed, resets the simulation to the initial state.
  //  Can also be triggered by pressing backspace.
  const resetSimulation = () => {
    simulation.resetData();
    simulation.forward();
  };
  simulationFolder.add({reset: () => { resetSimulation(); }}, 'reset').name('Reset');
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Backspace') {
      resetSimulation();
    }
  });
  actionInnerHTML += 'Reset simulation<br>';
  keyInnerHTML += 'Backspace<br>';

  // Add keyframe slider.
  let nkeys = model.nkey();
  let keyframeGUI = simulationFolder.add(params, "keyframeNumber", 0, nkeys - 1, 1).name('Load Keyframe').listen();
  keyframeGUI.onChange((value) => {
    if (value < model.nkey()) {
      console.log("Loading keyframe " + value + "...");
      simulation.qpos().set(model.key_qpos().slice(
        value * model.nq(), (value + 1) * model.nq())); }});
  updateGUICallbacks.push((model, simulation, params) => {
    let nkeys = model.nkey();
    console.log("new model loaded. has " + nkeys + " keyframes.");
    if (nkeys > 0) {
      keyframeGUI.max(nkeys - 1);
      keyframeGUI.domElement.style.opacity = 1.0;
    } else {
      // Disable keyframe slider if no keyframes are available.
      keyframeGUI.max(0);
      keyframeGUI.domElement.style.opacity = 0.5;
    }
  });

  // Add sliders for ctrlnoiserate and ctrlnoisestd; min = 0, max = 2, step = 0.01.
  simulationFolder.add(params, 'ctrlnoiserate', 0.0, 2.0, 0.01).name('Noise rate' );
  simulationFolder.add(params, 'ctrlnoisestd' , 0.0, 2.0, 0.01).name('Noise scale');

  // Add actuator sliders.
  let actuatorFolder = gui.addFolder("Actuators");
  const addActuators = (model, simulation, params) => {
    let act_range = model.actuator_ctrlrange();
    let actuatorGUIs = [];
    for (let i = 0; i < model.nu(); i++) {
      if (!model.actuator_ctrllimited()[i]) { continue; }
      let name = "Actuator " + i;
      params[name] = 0.0;
      let actuatorGUI = actuatorFolder.add(params, name, act_range[2 * i], act_range[2 * i + 1], 0.01).name(name).listen();
      actuatorGUIs.push(actuatorGUI);
      actuatorGUI.onChange((value) => {
        simulation.ctrl()[i] = value;
      });
    }
    return actuatorGUIs;
  };
  let actuatorGUIs = addActuators(model, simulation, params);
  updateGUICallbacks.push((model, simulation, params) => {
    for (let i = 0; i < actuatorGUIs.length; i++) {
      actuatorGUIs[i].destroy();
    }
    actuatorGUIs = addActuators(model, simulation, params);
  });
  actuatorFolder.close();

  // Add function that resets the camera to the default position.
  // Can be triggered by pressing ctrl + A.
  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey && event.code === 'KeyA') {
      resetCamera(camera);
    }
  });
  actionInnerHTML += 'Reset free camera<br>';
  keyInnerHTML += 'Ctrl A<br>';

  gui.open();

  await downloadExampleScenesFolder(mujoco);    // Download the the examples to MuJoCo's virtual file system
  [model, state, simulation, bodies, lights] =  // Initialize the three.js Scene using this .xml Model
    await loadSceneFromURL(mujoco, "piano_with_shadow_hands/scene.xml", scene);
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize( window.innerWidth, window.innerHeight );
}

function animate(time) {
  requestAnimationFrame( animate );
  render(time);
}

// Standard normal random number generator using Box-Muller transform.
function standardNormal() {
  return Math.sqrt(-2.0 * Math.log( Math.random())) *
         Math.cos ( 2.0 * Math.PI * Math.random()); }

let mujoco_time = 0.0;
function render(timeMS) {
  controls.update();

  if (!params["paused"]) {
    let timestep = model.getOptions().timestep;
    if (timeMS - mujoco_time > 35.0) { mujoco_time = timeMS; }
    while (mujoco_time < timeMS) {

      // Jitter the control state with gaussian random noise
      if (params["ctrlnoisestd"] > 0.0) {
        let rate  = Math.exp(-timestep / Math.max(1e-10, params["ctrlnoiserate"]));
        let scale = params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
        let currentCtrl = simulation.ctrl();
        for (let i = 0; i < currentCtrl.length; i++) {
          currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
          params["Actuator " + i] = currentCtrl[i];
        }
      }

      // Clear old perturbations, apply new ones.
      for (let i = 0; i < simulation.qfrc_applied().length; i++) { simulation.qfrc_applied()[i] = 0.0; }
      let dragged = dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        for (let b = 0; b < model.nbody(); b++) {
          if (bodies[b]) {
            getPosition  (simulation.xpos (), b, bodies[b].position);
            getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
            bodies[b].updateWorldMatrix();
          }
        }
        let bodyID = dragged.bodyID;
        dragStateManager.update(); // Update the world-space force origin
        let force = toMujocoPos(dragStateManager.currentWorld.clone().sub(dragStateManager.worldHit).multiplyScalar(model.body_mass()[bodyID] * 250));
        let point = toMujocoPos(dragStateManager.worldHit.clone());
        simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

        // TODO: Apply pose perturbations (mocap bodies only).
      }

      simulation.step();

      mujoco_time += timestep * 1000.0;
    }

  } else if (params["paused"]) {
    dragStateManager.update(); // Update the world-space force origin
    let dragged = dragStateManager.physicsObject;
    if (dragged && dragged.bodyID) {
      let b = dragged.bodyID;
      getPosition  (simulation.xpos (), b, tmpVec , false); // Get raw coordinate from MuJoCo
      getQuaternion(simulation.xquat(), b, tmpQuat, false); // Get raw coordinate from MuJoCo

      let offset = toMujocoPos(dragStateManager.currentWorld.clone()
        .sub(dragStateManager.worldHit).multiplyScalar(0.1));
      if (model.body_mocapid()[b] >= 0) {
        // Set the root body's mocap position...
        console.log("Trying to move mocap body", b);
        let addr = model.body_mocapid()[b] * 3;
        let pos = simulation.mocap_pos();
        pos[addr+0] += offset.x;
        pos[addr+1] += offset.y;
        pos[addr+2] += offset.z;
      } else {
        // Set the root body's position directly...
        let root = model.body_rootid()[b];
        let addr = model.jnt_qposadr()[model.body_jntadr()[root]];
        let pos = simulation.qpos();
        pos[addr+0] += offset.x;
        pos[addr+1] += offset.y;
        pos[addr+2] += offset.z;

        //// Save the original root body position
        //let x  = pos[addr + 0], y  = pos[addr + 1], z  = pos[addr + 2];
        //let xq = pos[addr + 3], yq = pos[addr + 4], zq = pos[addr + 5], wq = pos[addr + 6];

        //// Clear old perturbations, apply new ones.
        //for (let i = 0; i < simulation.qfrc_applied().length; i++) { simulation.qfrc_applied()[i] = 0.0; }
        //for (let bi = 0; bi < model.nbody(); bi++) {
        //  if (bodies[b]) {
        //    getPosition  (simulation.xpos (), bi, bodies[bi].position);
        //    getQuaternion(simulation.xquat(), bi, bodies[bi].quaternion);
        //    bodies[bi].updateWorldMatrix();
        //  }
        //}
        ////dragStateManager.update(); // Update the world-space force origin
        //let force = toMujocoPos(dragStateManager.currentWorld.clone()
        //  .sub(dragStateManager.worldHit).multiplyScalar(model.body_mass()[b] * 0.01));
        //let point = toMujocoPos(dragStateManager.worldHit.clone());
        //// This force is dumped into xrfc_applied
        //simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, b);
        //simulation.integratePos(simulation.qpos().byteOffset, simulation.qfrc_applied().byteOffset, 1);

        //// Add extra drag to the root body
        //pos[addr + 0] = x  + (pos[addr + 0] - x ) * 0.1;
        //pos[addr + 1] = y  + (pos[addr + 1] - y ) * 0.1;
        //pos[addr + 2] = z  + (pos[addr + 2] - z ) * 0.1;
        //pos[addr + 3] = xq + (pos[addr + 3] - xq) * 0.1;
        //pos[addr + 4] = yq + (pos[addr + 4] - yq) * 0.1;
        //pos[addr + 5] = zq + (pos[addr + 5] - zq) * 0.1;
        //pos[addr + 6] = wq + (pos[addr + 6] - wq) * 0.1;

      }
    }

    simulation.forward();
  }

  // Update body transforms.
  for (let b = 0; b < model.nbody(); b++) {
    if (bodies[b]) {
      getPosition  (simulation.xpos (), b, bodies[b].position);
      getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
      bodies[b].updateWorldMatrix();
    }
  }

  for (let b = 0; b < model.nbody(); b++) {
    if (bodies[b]) {
      let jnt_adr = model.body_jntadr()[b];
      if (jnt_adr < 0) { continue; }
      let qpos_adr = model.jnt_qposadr()[jnt_adr];
      if (simulation.qpos()[qpos_adr] > 0.025) {
        console.log("Key at", qpos_adr, "is pressed!");
      }
    }
  }

  // Update light transforms.
  for (let l = 0; l < model.nlight(); l++) {
    if (lights[l]) {
      getPosition(simulation.light_xpos(), l, lights[l].position);
      getPosition(simulation.light_xdir(), l, tmpVec);
      lights[l].lookAt(tmpVec.add(lights[l].position));
    }
  }

  // Render!
  renderer.render( scene, camera );
}

await init();
animate();
