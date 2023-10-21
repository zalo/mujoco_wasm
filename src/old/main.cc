#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/fetch.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

int finish(const char *msg = NULL, mjModel *m = NULL) {
  if (m  ) { mj_deleteModel(m); }
  if (msg) { std::printf("%s\n", msg); }
  return 0;
}

class Model {
public:
  Model() { m = NULL; }

  static Model load_from_xml(const std::string filename) {
    Model model;
    char error[1000] = "Could not load xml model";
    model.m = mj_loadXML(filename.c_str(), 0, error, 1000);
    if (!model.m) { finish(error, model.m); }
    return model;
  }

  mjModel *ptr     () { return m; }
  mjModel getVal   () { return *m; }

  int nq                    () { return m->nq                    ; } // number of generalized coordinates = dim(qpos)
  int nv                    () { return m->nv                    ; } // number of degrees of freedom = dim(qvel)
  int nu                    () { return m->nu                    ; } // number of actuators/controls = dim(ctrl)
  int na                    () { return m->na                    ; } // number of activation states = dim(act)
  int nbody                 () { return m->nbody                 ; } // number of bodies
  int njnt                  () { return m->njnt                  ; } // number of joints
  int ngeom                 () { return m->ngeom                 ; } // number of geoms
  int nsite                 () { return m->nsite                 ; } // number of sites
  int ncam                  () { return m->ncam                  ; } // number of cameras
  int nlight                () { return m->nlight                ; } // number of lights
  int nmesh                 () { return m->nmesh                 ; } // number of meshes
  int nmeshvert             () { return m->nmeshvert             ; } // number of vertices in all meshes
  int nmeshtexvert          () { return m->nmeshtexvert          ; } // number of vertices with texcoords in all meshes
  int nmeshface             () { return m->nmeshface             ; } // number of triangular faces in all meshes
  int nmeshgraph            () { return m->nmeshgraph            ; } // number of ints in mesh auxiliary data
  int nskin                 () { return m->nskin                 ; } // number of skins
  int nskinvert             () { return m->nskinvert             ; } // number of vertices in all skins
  int nskintexvert          () { return m->nskintexvert          ; } // number of vertiex with texcoords in all skins
  int nskinface             () { return m->nskinface             ; } // number of triangular faces in all skins
  int nskinbone             () { return m->nskinbone             ; } // number of bones in all skins
  int nskinbonevert         () { return m->nskinbonevert         ; } // number of vertices in all skin bones
  int nhfield               () { return m->nhfield               ; } // number of heightfields
  int nhfielddata           () { return m->nhfielddata           ; } // number of data points in all heightfields
  int ntex                  () { return m->ntex                  ; } // number of textures
  int ntexdata              () { return m->ntexdata              ; } // number of bytes in texture rgb data
  int nmat                  () { return m->nmat                  ; } // number of materials
  int npair                 () { return m->npair                 ; } // number of predefined geom pairs
  int nexclude              () { return m->nexclude              ; } // number of excluded geom pairs
  int neq                   () { return m->neq                   ; } // number of equality constraints
  int ntendon               () { return m->ntendon               ; } // number of tendons
  int nwrap                 () { return m->nwrap                 ; } // number of wrap objects in all tendon paths
  int nsensor               () { return m->nsensor               ; } // number of sensors
  int nnumeric              () { return m->nnumeric              ; } // number of numeric custom fields
  int nnumericdata          () { return m->nnumericdata          ; } // number of mjtNums in all numeric fields
  int ntext                 () { return m->ntext                 ; } // number of text custom fields
  int ntextdata             () { return m->ntextdata             ; } // number of mjtBytes in all text fields
  int ntuple                () { return m->ntuple                ; } // number of tuple custom fields
  int ntupledata            () { return m->ntupledata            ; } // number of objects in all tuple fields
  int nkey                  () { return m->nkey                  ; } // number of keyframes
  int nmocap                () { return m->nmocap                ; } // number of mocap bodies
  int nplugin               () { return m->nplugin               ; } // number of plugin instances
  int npluginattr           () { return m->npluginattr           ; } // number of chars in all plugin config attributes
  int nuser_body            () { return m->nuser_body            ; } // number of mjtNums in body_user
  int nuser_jnt             () { return m->nuser_jnt             ; } // number of mjtNums in jnt_user
  int nuser_geom            () { return m->nuser_geom            ; } // number of mjtNums in geom_user
  int nuser_site            () { return m->nuser_site            ; } // number of mjtNums in site_user
  int nuser_cam             () { return m->nuser_cam             ; } // number of mjtNums in cam_user
  int nuser_tendon          () { return m->nuser_tendon          ; } // number of mjtNums in tendon_user
  int nuser_actuator        () { return m->nuser_actuator        ; } // number of mjtNums in actuator_user
  int nuser_sensor          () { return m->nuser_sensor          ; } // number of mjtNums in sensor_user
  int nnames                () { return m->nnames                ; } // number of names in the names buffer
  int nM                    () { return m->nM                    ; } // number of non-zeros in sparse inertia matrix
  int nD                    () { return m->nD                    ; } // number of non-zeros in sparse derivative matrix
  int nemax                 () { return m->nemax                 ; } // number of potential equality-constraint rows
  int njmax                 () { return m->njmax                 ; } // number of available rows in constraint Jacobian
  int nconmax               () { return m->nconmax               ; } // number of potential contacts in contact list
  int nstack                () { return m->nstack                ; } // number of fields in mjData stack
  int nuserdata             () { return m->nuserdata             ; } // number of extra fields in mjData
  int nsensordata           () { return m->nsensordata           ; } // number of fields in sensor data vector
  int npluginstate          () { return m->npluginstate          ; } // number of fields in the plugin state vector
  int nbuffer               () { return m->nbuffer               ; } // number of bytes in buffer

  val names        () { return val(typed_memory_view(m->nnames       , m->names        )); }
  val mesh_vertadr () { return val(typed_memory_view(m->nmesh        , m->mesh_vertadr )); }
  val mesh_vertnum () { return val(typed_memory_view(m->nmesh        , m->mesh_vertnum )); }
  val mesh_faceadr () { return val(typed_memory_view(m->nmesh        , m->mesh_faceadr )); }
  val mesh_facenum () { return val(typed_memory_view(m->nmesh        , m->mesh_facenum )); }
  val name_meshadr () { return val(typed_memory_view(m->nmesh        , m->name_meshadr )); }
  val mesh_vert    () { return val(typed_memory_view(m->nmeshvert * 3, m->mesh_vert    )); }
  val mesh_normal  () { return val(typed_memory_view(m->nmeshvert * 3, m->mesh_normal  )); }
  val mesh_face    () { return val(typed_memory_view(m->nmeshface * 3, m->mesh_face    )); }
  val body_parentid() { return val(typed_memory_view(m->nbody        , m->body_parentid)); }
  val body_geomnum () { return val(typed_memory_view(m->nbody        , m->body_geomnum )); }
  val body_geomadr () { return val(typed_memory_view(m->nbody        , m->body_geomadr )); }
  val body_pos     () { return val(typed_memory_view(m->nbody     * 3, m->body_pos     )); }
  val body_quat    () { return val(typed_memory_view(m->nbody     * 4, m->body_quat    )); }
  val geom_type    () { return val(typed_memory_view(m->ngeom        , m->geom_type    )); }
  val geom_bodyid  () { return val(typed_memory_view(m->ngeom        , m->geom_bodyid  )); }
  val geom_group   () { return val(typed_memory_view(m->ngeom        , m->geom_group   )); }
  val geom_contype () { return val(typed_memory_view(m->ngeom        , m->geom_contype )); }
  val geom_size    () { return val(typed_memory_view(m->ngeom     * 3, m->geom_size    )); }
  val geom_pos     () { return val(typed_memory_view(m->ngeom     * 3, m->geom_pos     )); }
  val geom_quat    () { return val(typed_memory_view(m->ngeom     * 4, m->geom_quat    )); }
  val geom_rgba    () { return val(typed_memory_view(m->ngeom     * 4, m->geom_rgba    )); }
  val geom_matid   () { return val(typed_memory_view(m->ngeom        , m->geom_matid   )); }
  val mat_emission   () { return val(typed_memory_view(m->nmat       , m->mat_emission   )); }   
  val mat_specular   () { return val(typed_memory_view(m->nmat       , m->mat_specular   )); }   
  val mat_shininess  () { return val(typed_memory_view(m->nmat       , m->mat_shininess  )); }   
  val mat_reflectance() { return val(typed_memory_view(m->nmat       , m->mat_reflectance)); }   
  val mat_rgba       () { return val(typed_memory_view(m->nmat    * 4, m->mat_rgba       )); }   

private:
  mjModel *m;
};

class State {
public:
  State(Model m)  { d = mj_makeData(m.ptr()); }
  mjData *ptr  () { return d; }
  mjData getVal() { return *d; }

private:
  mjData *d;
};

class Simulation {
public:
  Simulation(Model *m, State *s) {
    _model = m;
    _state = s;
  }

  State *state() { return _state; }
  Model *model() { return _model; }
  void    step() { mj_step(_model->ptr(), _state->ptr()); }

  val    xquat() { return val(typed_memory_view(_model->ptr()->nbody * 4, _state->ptr()->xquat)); }
  val    xpos () { return val(typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->xpos )); }

private:
  Model *_model;
  State *_state;
};

// main function
int main(int argc, char **argv) {
  std::printf("MuJoCo version: %d\n\n", mj_version());
  return 0;
}

EMSCRIPTEN_BINDINGS(mujoco_wasm) {
  class_<Model>("Model")
      .constructor<>()
      .class_function("load_from_xml", &Model::load_from_xml)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("getVal"        , &Model::getVal        )
      .function("nq"            , &Model::nq            )
      .function("nv"            , &Model::nv            )
      .function("nu"            , &Model::nu            )
      .function("na"            , &Model::na            )
      .function("nbody"         , &Model::nbody         )
      .function("njnt"          , &Model::njnt          )
      .function("ngeom"         , &Model::ngeom         )
      .function("nsite"         , &Model::nsite         )
      .function("ncam"          , &Model::ncam          )
      .function("nlight"        , &Model::nlight        )
      .function("nmesh"         , &Model::nmesh         )
      .function("nmeshvert"     , &Model::nmeshvert     )
      .function("nmeshtexvert"  , &Model::nmeshtexvert  )
      .function("nmeshface"     , &Model::nmeshface     )
      .function("nmeshgraph"    , &Model::nmeshgraph    )
      .function("nskin"         , &Model::nskin         )
      .function("nskinvert"     , &Model::nskinvert     )
      .function("nskintexvert"  , &Model::nskintexvert  )
      .function("nskinface"     , &Model::nskinface     )
      .function("nskinbone"     , &Model::nskinbone     )
      .function("nskinbonevert" , &Model::nskinbonevert )
      .function("nhfield"       , &Model::nhfield       )
      .function("nhfielddata"   , &Model::nhfielddata   )
      .function("ntex"          , &Model::ntex          )
      .function("ntexdata"      , &Model::ntexdata      )
      .function("nmat"          , &Model::nmat          )
      .function("npair"         , &Model::npair         )
      .function("nexclude"      , &Model::nexclude      )
      .function("neq"           , &Model::neq           )
      .function("ntendon"       , &Model::ntendon       )
      .function("nwrap"         , &Model::nwrap         )
      .function("nsensor"       , &Model::nsensor       )
      .function("nnumeric"      , &Model::nnumeric      )
      .function("nnumericdata"  , &Model::nnumericdata  )
      .function("ntext"         , &Model::ntext         )
      .function("ntextdata"     , &Model::ntextdata     )
      .function("ntuple"        , &Model::ntuple        )
      .function("ntupledata"    , &Model::ntupledata    )
      .function("nkey"          , &Model::nkey          )
      .function("nmocap"        , &Model::nmocap        )
      .function("nplugin"       , &Model::nplugin       )
      .function("npluginattr"   , &Model::npluginattr   )
      .function("nuser_body"    , &Model::nuser_body    )
      .function("nuser_jnt"     , &Model::nuser_jnt     )
      .function("nuser_geom"    , &Model::nuser_geom    )
      .function("nuser_site"    , &Model::nuser_site    )
      .function("nuser_cam"     , &Model::nuser_cam     )
      .function("nuser_tendon"  , &Model::nuser_tendon  )
      .function("nuser_actuator", &Model::nuser_actuator)
      .function("nuser_sensor"  , &Model::nuser_sensor  )
      .function("nnames"        , &Model::nnames        )
      .function("nM"            , &Model::nM            )
      .function("nD"            , &Model::nD            )
      .function("nemax"         , &Model::nemax         )
      .function("njmax"         , &Model::njmax         )
      .function("nconmax"       , &Model::nconmax       )
      .function("nstack"        , &Model::nstack        )
      .function("nuserdata"     , &Model::nuserdata     )
      .function("nsensordata"   , &Model::nsensordata   )
      .function("npluginstate"  , &Model::npluginstate  )
      .function("nbuffer"       , &Model::nbuffer       )
      .function("names"         , &Model::names)
      .function("mesh_vertadr"  , &Model::mesh_vertadr)
      .function("mesh_vertnum"  , &Model::mesh_vertnum)
      .function("mesh_faceadr"  , &Model::mesh_faceadr)
      .function("mesh_facenum"  , &Model::mesh_facenum)
      .function("body_parentid" , &Model::body_parentid)
      .function("body_geomnum"  , &Model::body_geomnum)
      .function("body_geomadr"  , &Model::body_geomadr)
      .function("body_pos"      , &Model::body_pos)
      .function("body_quat"     , &Model::body_quat)
      .function("geom_type"     , &Model::geom_type)
      .function("geom_bodyid"   , &Model::geom_bodyid)
      .function("geom_group"    , &Model::geom_group)
      .function("geom_contype"  , &Model::geom_contype)
      .function("mesh_normal"   , &Model::mesh_normal)
      .function("mesh_face"     , &Model::mesh_face)
      .function("mesh_vert"     , &Model::mesh_vert)
      .function("name_meshadr"  , &Model::name_meshadr)
      .function("geom_pos"      , &Model::geom_pos)
      .function("geom_quat"     , &Model::geom_quat)
      .function("geom_size"     , &Model::geom_size)
      .function("geom_rgba"     , &Model::geom_rgba)
      .function("geom_matid"    , &Model::geom_matid)
      .function("mat_emission"  , &Model::mat_emission)
      .function("mat_specular"  , &Model::mat_specular)
      .function("mat_shininess" , &Model::mat_shininess)
      .function("mat_reflectance", &Model::mat_reflectance)
      .function("mat_rgba"      , &Model::mat_rgba);


  class_<State>("State")
      .constructor<Model>()
      .function("ptr"   , &State::ptr, allow_raw_pointers())
      .function("getVal", &State::getVal);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("step" , &Simulation::step)
      .function("state", &Simulation::state, allow_raw_pointers())
      .function("model", &Simulation::model, allow_raw_pointers())
      .function("xquat", &Simulation::xquat)
      .function("xpos" , &Simulation::xpos);

  //value_object<mjModel>("mjModel")
  //    .field("ngeom"      , &mjModel::ngeom)
  //    .field("nq"         , &mjModel::nq)
  //    .field("na"         , &mjModel::na)
  //    .field("nv"         , &mjModel::nv)
  //    .field("nu"         , &mjModel::nu)
  //    .field("nbody"      , &mjModel::nbody)
  //    .field("nsensordata", &mjModel::nsensordata)
  //    .field("nmesh"      , &mjModel::nmesh)
  //    .field("nmeshvert"  , &mjModel::nmeshvert)
  //    .field("nmeshface"  , &mjModel::nmeshface);
  //value_object<mjData>("mjData");
}
