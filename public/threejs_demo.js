
import * as THREE from 'three';
import { GUI           } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { Reflector } from './Reflector.js';
//import { Debug } from './Debug.js';
import { Grabber } from './Grabber.js';
import load_mujoco from "./mujoco_wasm.js"

// Load the MuJoCo WASM
const mujoco = await load_mujoco();
// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./public/scenes/humanoid.xml")).text());

// Load in the state from XML
let model       = mujoco.Model.load_from_xml("/working/humanoid.xml");
let state       = new mujoco.State     (model);
let simulation  = new mujoco.Simulation(model, state);

let container, controls; // ModelLoader
let camera, scene, renderer, material;
/** @type {THREE.Mesh} */
let mainModel, connections;
/** @type {THREE.Vector3} */
let tmpVec = new THREE.Vector3();
/** @type {THREE.Quaternion} */
let tmpQuat = new THREE.Quaternion();
let raycaster, pointer = new THREE.Vector2();
const params = { acceleration: 0.0, scene: "humanoid.xml" };
let grabber;

/** @type {Object.<number, THREE.Group>} */
let bodies = {};
/** @type {Object.<number, THREE.BufferGeometry>} */
let meshes = {};
/** @type {THREE.Light[]} */
let lights = [];

async function init() {
  container = document.createElement( 'div' );
  document.body.appendChild( container );

  scene = new THREE.Scene();
  scene.name = 'scene';

  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
  camera.position.set( 2.0, 1.7, 1.7 );

  camera.name = 'PerspectiveCamera';
  scene.add(camera);
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

  raycaster = new THREE.Raycaster();
  pointer   = new THREE.Vector2();

  window.addEventListener('resize', onWindowResize);

  const gui = new GUI();
  //gui.add(params, "acceleration", -0.1, 0.1, 0.001).name('Artificial Acceleration');
  gui.add(params, 'scene', { "Humanoid": "humanoid.xml", "Cassie": "agility_cassie/scene.xml", "Hand": "shadow_hand/scene_right.xml", "Balloons": "balloons.xml",  "Hammock": "hammock.xml", "Flag": "flag.xml", "Mug": "mug.xml",  /*"Arm": "arm26.xml", "Adhesion": "adhesion.xml", "Boxes": "simple.xml" */})
    .name('Example Scene').onChange(value => { scene.remove(bodies[0]); bodies = {}; meshes = {}; lights = [];  loadSceneFromURL(value); } );
  gui.open();

  grabber = new Grabber(scene, renderer, camera, container.parentElement, controls);

  material = new THREE.MeshPhysicalMaterial();
  material.color = new THREE.Color(1, 1, 1);

  let allFiles = [
    "agility_cassie/assets/achilles-rod.obj",
    "agility_cassie/assets/cassie-texture.png",
    "agility_cassie/assets/foot-crank.obj",
    "agility_cassie/assets/foot.obj",
    "agility_cassie/assets/heel-spring.obj",
    "agility_cassie/assets/hip-pitch.obj",
    "agility_cassie/assets/hip-roll.obj",
    "agility_cassie/assets/hip-yaw.obj",
    "agility_cassie/assets/knee-spring.obj",
    "agility_cassie/assets/knee.obj",
    "agility_cassie/assets/pelvis.obj",
    "agility_cassie/assets/plantar-rod.obj",
    "agility_cassie/assets/shin.obj",
    "agility_cassie/assets/tarsus.obj",
    "agility_cassie/cassie.xml",
    "agility_cassie/LICENSE",
    "agility_cassie/README.md",
    "agility_cassie/scene.xml",
    "shadow_hand/assets/forearm_0.obj",
    "shadow_hand/assets/forearm_1.obj",
    "shadow_hand/assets/forearm_collision.obj",
    "shadow_hand/assets/f_distal_pst.obj",
    "shadow_hand/assets/f_knuckle.obj",
    "shadow_hand/assets/f_middle.obj",
    "shadow_hand/assets/f_proximal.obj",
    "shadow_hand/assets/lf_metacarpal.obj",
    "shadow_hand/assets/mounting_plate.obj",
    "shadow_hand/assets/palm.obj",
    "shadow_hand/assets/th_distal_pst.obj",
    "shadow_hand/assets/th_middle.obj",
    "shadow_hand/assets/th_proximal.obj",
    "shadow_hand/assets/wrist.obj",
    "shadow_hand/left_hand.xml",
    "shadow_hand/LICENSE",
    "shadow_hand/README.md",
    "shadow_hand/right_hand.xml",
    "shadow_hand/scene_left.xml",
    "shadow_hand/scene_right.xml",
    "22_humanoids.xml",
    "adhesion.xml",
    "arm26.xml",
    "balloons.xml",
    "flag.xml",
    "generate_index.py",
    "hammock.xml",
    "humanoid.xml",
    "humanoid_body.xml",
    "index.json",
    "mug.obj",
    "mug.png",
    "mug.xml",
    "simple.xml",
    "slider_crank.xml"
  ];

  let requests  = allFiles.map((url) => fetch("./public/scenes/" + url));
  let responses = await Promise.all(requests);
  for (let i = 0; i < responses.length; i++) {
    let split = allFiles[i].split("/");
    let working = '/working/';
    for (let f = 0; f < split.length - 1; f++){
      working += split[f];
      if (!mujoco.FS.analyzePath(working).exists) { mujoco.FS.mkdir(working); }
      working += "/";
    }

    if (allFiles[i].endsWith(".png") || allFiles[i].endsWith(".stl") || allFiles[i].endsWith(".skn")) {
      mujoco.FS.writeFile("/working/" + allFiles[i], new Uint8Array(await responses[i].arrayBuffer()));
    } else {
      mujoco.FS.writeFile("/working/" + allFiles[i], await responses[i].text());
    }
  }

  await loadSceneFromURL("humanoid.xml");
}

async function loadSceneFromURL(filename) {
  // Load in the state from XML
  model       = mujoco.Model.load_from_xml("/working/"+filename);
  state       = new mujoco.State     (model);
  simulation  = new mujoco.Simulation(model, state);

  // Decode the null-terminated string names
  let textDecoder = new TextDecoder("utf-8");
  let fullString  = textDecoder.decode(model.names());
  let names       = fullString.split(textDecoder.decode(new ArrayBuffer(1)));
  console.log(names);

  for (let g = 0; g < model.ngeom(); g++) {
    // Only visualize geom groups up to 2 (same default behavior as simulate).
    if (!(model.geom_group()[g] < 3)) { continue; }

    let b = model.geom_bodyid()[g];
    let type = model.geom_type  ()[g];
    let size = [
      model.geom_size()[(g*3) + 0],
      model.geom_size()[(g*3) + 1],
      model.geom_size()[(g*3) + 2]];
    // Figure out how to use model.name_bodyadr()[b]
    console.log("Found geometry", g, " for body", b, ", Type:", type, ", named:", names[b+1], "with mesh at:", model.geom_dataid()[g]);

    if (!(b in bodies)) { bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; bodies[b].bodyID = b; bodies[b].has_custom_mesh = false; }
    if (bodies[b].has_custom_mesh && type != 7) { continue; }

    let geometry = new THREE.SphereGeometry(size[0] * 0.5);
    if (type == 0)        { // Plane is 0
      //geometry = new THREE.PlaneGeometry(size[0], size[1]); // Can't rotate this...
      //geometry = new THREE.BoxGeometry(100, 0.0001, 100);
    } else if (type == 1) { // Heightfield is 1
    } else if (type == 2) { // Sphere is 2
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type == 3) { // Capsule is 3
      geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
    } else if (type == 4) { // Ellipsoid is 4
      geometry = new THREE.SphereGeometry(1); // Stretch this below
    } else if (type == 5) { // Cylinder is 5
      // size[0] is radius, size[1] is half-height in MuJoCo.
      geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
    } else if (type == 6) { // Box is 6
      geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
    } else if (type == 7) { // Generic Mesh is 7
      let meshID = model.geom_dataid()[g];

      if (!(meshID in meshes)) {
        geometry = new THREE.BufferGeometry(); // TODO: Populate the Buffer Geometry with Generic Mesh Data

        let vertex_buffer = model.mesh_vert().subarray(
           model.mesh_vertadr()[meshID] * 3,
          (model.mesh_vertadr()[meshID]  + model.mesh_vertnum()[meshID]) * 3);
        for (let v = 0; v < vertex_buffer.length; v+=3){
          //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
          let temp             =  vertex_buffer[v + 1];
          vertex_buffer[v + 1] =  vertex_buffer[v + 2];
          vertex_buffer[v + 2] = -temp;
        }

        let normal_buffer = model.mesh_normal().subarray(
           model.mesh_vertadr()[meshID] * 3,
          (model.mesh_vertadr()[meshID]  + model.mesh_vertnum()[meshID]) * 3);
        for (let v = 0; v < normal_buffer.length; v+=3){
          //normal_buffer[v + 0] =  normal_buffer[v + 0];
          let temp             =  normal_buffer[v + 1];
          normal_buffer[v + 1] =  normal_buffer[v + 2];
          normal_buffer[v + 2] = -temp;
        }

        let uv_buffer = model.mesh_texcoord().subarray(
           model.mesh_texcoordadr()[meshID] * 2,
          (model.mesh_texcoordadr()[meshID]  + model.mesh_vertnum()[meshID]) * 2);
        let triangle_buffer = model.mesh_face().subarray(
           model.mesh_faceadr()[meshID] * 3,
          (model.mesh_faceadr()[meshID]  + model.mesh_facenum()[meshID]) * 3);
        geometry.setAttribute("position", new THREE.BufferAttribute(vertex_buffer, 3));
        geometry.setAttribute("normal"  , new THREE.BufferAttribute(normal_buffer, 3));
        geometry.setAttribute("uv"      , new THREE.BufferAttribute(    uv_buffer, 2));
        geometry.setIndex    (Array.from(triangle_buffer));
        meshes[meshID] = geometry;
      } else {
        geometry = meshes[meshID];
      }

      bodies[b].has_custom_mesh = true;
    }

    // Set the Material Properties of incoming bodies
    let texture = undefined;
    let color = [
      model.geom_rgba()[(g * 4) + 0],
      model.geom_rgba()[(g * 4) + 1],
      model.geom_rgba()[(g * 4) + 2],
      model.geom_rgba()[(g * 4) + 3]];
    if (model.geom_matid()[g] != -1) {
      let matId = model.geom_matid()[g];
      color = [
        model.mat_rgba()[(matId * 4) + 0],
        model.mat_rgba()[(matId * 4) + 1],
        model.mat_rgba()[(matId * 4) + 2],
        model.mat_rgba()[(matId * 4) + 3]];

      // Construct Texture from
      texture = undefined;
      let texId = model.mat_texid()[matId];
      if (texId != -1) {
        let width    = model.tex_width ()[texId];
        let height   = model.tex_height()[texId];
        let offset   = model.tex_adr   ()[texId];
        let rgbArray = model.tex_rgb   ();
        let rgbaArray = new Uint8Array(width * height * 4);
        for (let p = 0; p < width * height; p++){
          rgbaArray[(p * 4) + 0] = rgbArray[offset + ((p * 3) + 0)];
          rgbaArray[(p * 4) + 1] = rgbArray[offset + ((p * 3) + 1)];
          rgbaArray[(p * 4) + 2] = rgbArray[offset + ((p * 3) + 2)];
          rgbaArray[(p * 4) + 3] = 1.0;
        }
        texture = new THREE.DataTexture(rgbaArray, width, height, THREE.RGBAFormat, THREE.UnsignedByteType);
        if (texId == 2) {
          texture.repeat = new THREE.Vector2(50, 50);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        } else {
          texture.repeat = new THREE.Vector2(1, 1);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        }

        texture.needsUpdate = true;
      }
    }
    if (material.color.r != color[0] ||
        material.color.g != color[1] ||
        material.color.b != color[2] ||
        material.opacity != color[3] ||
        material.map != texture) {
      material = new THREE.MeshPhysicalMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        transparent: color[3] < 1.0,
        opacity: color[3],
        specularIntensity: model.geom_matid()[g] != -1 ?       model.mat_specular   ()[model.geom_matid()[g]] *0.5 : undefined,
        reflectivity     : model.geom_matid()[g] != -1 ?       model.mat_reflectance()[model.geom_matid()[g]] : undefined,
        roughness        : model.geom_matid()[g] != -1 ? 1.0 - model.mat_shininess  ()[model.geom_matid()[g]] : undefined,
        metalness        : model.geom_matid()[g] != -1 ? 0.1 : undefined,
        map              : texture
      });
    }

    let mesh = new THREE.Mesh();
    if (type == 0) {
      mesh = new Reflector( new THREE.PlaneGeometry( 100, 100 ), { clipBias: 0.003,texture: texture } );
      mesh.rotateX( - Math.PI / 2 );
    } else {
      mesh = new THREE.Mesh(geometry, material);
    }

    mesh.castShadow = g == 0 ? false : true;
    mesh.receiveShadow = type != 7;
    mesh.bodyID = b;
    bodies[b].add(mesh);
    getPosition  (model.geom_pos (), g, mesh.position  );
    if (type != 0) { getQuaternion(model.geom_quat(), g, mesh.quaternion); }
    if (type == 4) { mesh.scale.set(size[0], size[2], size[1]) } // Stretch the Ellipsoid
  }

  for (let l = 0; l < model.nlight(); l++) {
    let light = new THREE.SpotLight();
    if (model.light_directional()[l]) {
      light = new THREE.DirectionalLight();
    } else {
      light = new THREE.SpotLight();
    }
    light.decay = model.light_attenuation()[l] * 100;
    light.penumbra = 0.5;
    light.castShadow = true; // default false

    light.shadow.mapSize.width = 1024; // default
    light.shadow.mapSize.height = 1024; // default
    light.shadow.camera.near = 1; // default
    light.shadow.camera.far = 10; // default
    //bodies[model.light_bodyid()].add(light);
    if (bodies[0]) {
      bodies[0].add(light);
    } else {
      scene.add(light);
    }
    lights.push(light);
  }

  for (let b = 0; b < model.nbody(); b++) {
    //let parent_body = model.body_parentid()[b];
    if (b == 0 || !bodies[0]) {
      scene.add(bodies[b]);
    } else if(bodies[b]){
      bodies[0].add(bodies[b]);
    } else {
      console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
      bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; bodies[b].bodyID = b; bodies[b].has_custom_mesh = false;
      bodies[0].add(bodies[b]);
    }
  }
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

function getPosition(buffer, index, target) {
  return target.set(
     buffer[(index * 3) + 0],
     buffer[(index * 3) + 2],
    -buffer[(index * 3) + 1]);
}

function getQuaternion(buffer, index, target) {
  return target.set(
    -buffer[(index * 4) + 1],
    -buffer[(index * 4) + 3],
     buffer[(index * 4) + 2],
    -buffer[(index * 4) + 0]);
}

function toMujocoPos(target) { return target.set(target.x, -target.z, target.y); }

let mujoco_time = 0.0;
function render(timeMS) {
  controls.update();

  // Update MuJoCo Simulation
  if (timeMS - mujoco_time > 1000.0) { mujoco_time = timeMS; }
  while (mujoco_time < timeMS) {
    simulation.step();

    // Set the transforms of the bodies
    for (let b = 0; b < model.nbody(); b++) {
      if (bodies[b]) {
        getPosition(simulation.xpos(), b, bodies[b].position);
        getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
        bodies[b].updateWorldMatrix();
      }
    }

    grabber.update();

    // Reset Applied Forces
    for (let i = 0; i < simulation.qfrc_applied().length; i++) { simulation.qfrc_applied()[i] = 0.0; }
    if (grabber.active && grabber.physicsObject) {
      let bodyID = grabber.physicsObject.bodyID;
      let force = toMujocoPos(grabber.currentWorld.clone().sub(grabber.worldHit).multiplyScalar(model.body_mass()[bodyID] * 250));
      let point = toMujocoPos(grabber.worldHit.clone());
      simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID); // Body ID
    }

    mujoco_time += params.scene.includes("cassie") ? 0.5 : 5.0; // Assume each MuJoCo timestep is 5ms for now
  }

  // Set the transforms of lights
  for (let l = 0; l < model.nlight(); l++) {
    if (lights[l]) {
      getPosition(simulation.light_xpos(), l, lights[l].position);
      getPosition(simulation.light_xdir(), l, tmpVec);
      lights[l].lookAt(tmpVec.add(lights[l].position));
    }
  }

  renderer.render( scene, camera );
}

await init();
animate();
