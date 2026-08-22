// Smoke test: compile every demo scene with the new @mujoco/mujoco WASM module.
import load_mujoco from './node_modules/@mujoco/mujoco/mujoco.js';
import { readFileSync, readdirSync, statSync } from 'fs';
import { join, relative } from 'path';

const mujoco = await load_mujoco();
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

// Copy the whole assets/scenes tree into the VFS.
function copyDir(dir) {
  for (const entry of readdirSync(dir)) {
    const full = join(dir, entry);
    const rel = relative('assets/scenes', full);
    if (statSync(full).isDirectory()) {
      mujoco.FS.mkdir('/working/' + rel);
      copyDir(full);
    } else {
      mujoco.FS.writeFile('/working/' + rel, readFileSync(full));
    }
  }
}
copyDir('assets/scenes');

const scenes = [
  "22_humanoids.xml", "adhesion.xml", "agility_cassie/scene.xml", "arm26.xml",
  "balloons.xml", "car.xml", "conveyor_magnets.xml", "sleep_pile.xml",
  "flex.xml", "hammock.xml", "humanoid.xml",
  "model.xml", "mug.xml", "scene.xml", "ufactory_xarm7/scene.xml",
  "ufactory_xarm7/scene_teleop.xml",
  "ufactory_xarm7/scene_hand_teleop.xml",
  "shadow_hand/scene_right.xml",
  "shadow_hand/scene_left.xml", "simple.xml", "slider_crank.xml",
  "model_with_tendon.xml",
];

let failures = 0;
for (const scene of scenes) {
  try {
    const model = mujoco.MjModel.mj_loadXML('/working/' + scene);
    const data = new mujoco.MjData(model);
    for (let i = 0; i < 100; i++) mujoco.mj_step(model, data);
    // Touch the fields the renderer reads.
    const probes = [model.names, model.name_bodyadr, model.geom_rgba, model.mat_texid,
      model.tex_data, data.xpos, data.xquat, data.ten_wrapadr, data.wrap_xpos];
    if (probes.some(p => p === undefined)) throw new Error('missing field');
    console.log(`OK   ${scene}  (nbody=${model.nbody} ngeom=${model.ngeom} nu=${model.nu} nkey=${model.nkey} time=${data.time.toFixed(3)})`);
    data.delete(); model.delete();
  } catch (e) {
    failures++;
    console.log(`FAIL ${scene}: ${e.message ?? e}`);
  }
}
process.exit(failures ? 1 : 0);
