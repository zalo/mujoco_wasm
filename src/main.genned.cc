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
  Model(const std::string filename) {
    if(0 == filename.compare(filename.length() - 3, 3, "mjb")){
      char error[1000] = "Could not load mjb model";
      m = mj_loadModel(filename.c_str(), 0); 
      if (!m) { finish(error, m); }
    } else {
      char error[1000] = "Could not load xml model";
      m = mj_loadXML(filename.c_str(), 0, error, 1000); 
      if (!m) { finish(error, m); }
    }
  }

  static Model load_from_xml(const std::string filename) { return Model(filename); }
  static Model load_from_mjb(const std::string filename) { return Model(filename); }

  mjModel *ptr       () { return m; }
  mjModel getVal     () { return *m; }
  mjOption getOptions() { return (*m).opt; }
  void free          () { return mju_free(m); }

  // MJMODEL_DEFINITIONS
  int  nq            () const { return m->nq            ; }
  int  nv            () const { return m->nv            ; }
  int  nu            () const { return m->nu            ; }
  int  na            () const { return m->na            ; }
  int  nbody         () const { return m->nbody         ; }
  int  njnt          () const { return m->njnt          ; }
  int  ngeom         () const { return m->ngeom         ; }
  int  nsite         () const { return m->nsite         ; }
  int  ncam          () const { return m->ncam          ; }
  int  nlight        () const { return m->nlight        ; }
  int  nmesh         () const { return m->nmesh         ; }
  int  nmeshvert     () const { return m->nmeshvert     ; }
  int  nmeshtexvert  () const { return m->nmeshtexvert  ; }
  int  nmeshface     () const { return m->nmeshface     ; }
  int  nmeshgraph    () const { return m->nmeshgraph    ; }
  int  nskin         () const { return m->nskin         ; }
  int  nskinvert     () const { return m->nskinvert     ; }
  int  nskintexvert  () const { return m->nskintexvert  ; }
  int  nskinface     () const { return m->nskinface     ; }
  int  nskinbone     () const { return m->nskinbone     ; }
  int  nskinbonevert () const { return m->nskinbonevert ; }
  int  nhfield       () const { return m->nhfield       ; }
  int  nhfielddata   () const { return m->nhfielddata   ; }
  int  ntex          () const { return m->ntex          ; }
  int  ntexdata      () const { return m->ntexdata      ; }
  int  nmat          () const { return m->nmat          ; }
  int  npair         () const { return m->npair         ; }
  int  nexclude      () const { return m->nexclude      ; }
  int  neq           () const { return m->neq           ; }
  int  ntendon       () const { return m->ntendon       ; }
  int  nwrap         () const { return m->nwrap         ; }
  int  nsensor       () const { return m->nsensor       ; }
  int  nnumeric      () const { return m->nnumeric      ; }
  int  nnumericdata  () const { return m->nnumericdata  ; }
  int  ntext         () const { return m->ntext         ; }
  int  ntextdata     () const { return m->ntextdata     ; }
  int  ntuple        () const { return m->ntuple        ; }
  int  ntupledata    () const { return m->ntupledata    ; }
  int  nkey          () const { return m->nkey          ; }
  int  nmocap        () const { return m->nmocap        ; }
  int  nplugin       () const { return m->nplugin       ; }
  int  npluginattr   () const { return m->npluginattr   ; }
  int  nuser_body    () const { return m->nuser_body    ; }
  int  nuser_jnt     () const { return m->nuser_jnt     ; }
  int  nuser_geom    () const { return m->nuser_geom    ; }
  int  nuser_site    () const { return m->nuser_site    ; }
  int  nuser_cam     () const { return m->nuser_cam     ; }
  int  nuser_tendon  () const { return m->nuser_tendon  ; }
  int  nuser_actuator() const { return m->nuser_actuator; }
  int  nuser_sensor  () const { return m->nuser_sensor  ; }
  int  nnames        () const { return m->nnames        ; }
  int  nM            () const { return m->nM            ; }
  int  nD            () const { return m->nD            ; }
  int  nemax         () const { return m->nemax         ; }
  int  njmax         () const { return m->njmax         ; }
  int  nconmax       () const { return m->nconmax       ; }
  int  nstack        () const { return m->nstack        ; }
  int  nuserdata     () const { return m->nuserdata     ; }
  int  nsensordata   () const { return m->nsensordata   ; }
  int  npluginstate  () const { return m->npluginstate  ; }
  int  nbuffer       () const { return m->nbuffer       ; }
  val  qpos0                  () const { return val(typed_memory_view(m->nq              * 1        , m->qpos0                  )); }
  val  qpos_spring            () const { return val(typed_memory_view(m->nq              * 1        , m->qpos_spring            )); }
  val  body_parentid          () const { return val(typed_memory_view(m->nbody           * 1        , m->body_parentid          )); }
  val  body_rootid            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_rootid            )); }
  val  body_weldid            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_weldid            )); }
  val  body_mocapid           () const { return val(typed_memory_view(m->nbody           * 1        , m->body_mocapid           )); }
  val  body_jntnum            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_jntnum            )); }
  val  body_jntadr            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_jntadr            )); }
  val  body_dofnum            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_dofnum            )); }
  val  body_dofadr            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_dofadr            )); }
  val  body_geomnum           () const { return val(typed_memory_view(m->nbody           * 1        , m->body_geomnum           )); }
  val  body_geomadr           () const { return val(typed_memory_view(m->nbody           * 1        , m->body_geomadr           )); }
  val  body_simple            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_simple            )); }
  val  body_sameframe         () const { return val(typed_memory_view(m->nbody           * 1        , m->body_sameframe         )); }
  val  body_pos               () const { return val(typed_memory_view(m->nbody           * 3        , m->body_pos               )); }
  val  body_quat              () const { return val(typed_memory_view(m->nbody           * 4        , m->body_quat              )); }
  val  body_ipos              () const { return val(typed_memory_view(m->nbody           * 3        , m->body_ipos              )); }
  val  body_iquat             () const { return val(typed_memory_view(m->nbody           * 4        , m->body_iquat             )); }
  val  body_mass              () const { return val(typed_memory_view(m->nbody           * 1        , m->body_mass              )); }
  val  body_subtreemass       () const { return val(typed_memory_view(m->nbody           * 1        , m->body_subtreemass       )); }
  val  body_inertia           () const { return val(typed_memory_view(m->nbody           * 3        , m->body_inertia           )); }
  val  body_invweight0        () const { return val(typed_memory_view(m->nbody           * 2        , m->body_invweight0        )); }
  val  body_gravcomp          () const { return val(typed_memory_view(m->nbody           * 1        , m->body_gravcomp          )); }
  val  body_user              () const { return val(typed_memory_view(m->nbody           * m->nuser_body, m->body_user              )); }
  val  body_plugin            () const { return val(typed_memory_view(m->nbody           * 1        , m->body_plugin            )); }
  val  jnt_type               () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_type               )); }
  val  jnt_qposadr            () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_qposadr            )); }
  val  jnt_dofadr             () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_dofadr             )); }
  val  jnt_bodyid             () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_bodyid             )); }
  val  jnt_group              () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_group              )); }
  val  jnt_limited            () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_limited            )); }
  val  jnt_solref             () const { return val(typed_memory_view(m->njnt            * mjNREF   , m->jnt_solref             )); }
  val  jnt_solimp             () const { return val(typed_memory_view(m->njnt            * mjNIMP   , m->jnt_solimp             )); }
  val  jnt_pos                () const { return val(typed_memory_view(m->njnt            * 3        , m->jnt_pos                )); }
  val  jnt_axis               () const { return val(typed_memory_view(m->njnt            * 3        , m->jnt_axis               )); }
  val  jnt_stiffness          () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_stiffness          )); }
  val  jnt_range              () const { return val(typed_memory_view(m->njnt            * 2        , m->jnt_range              )); }
  val  jnt_margin             () const { return val(typed_memory_view(m->njnt            * 1        , m->jnt_margin             )); }
  val  jnt_user               () const { return val(typed_memory_view(m->njnt            * m->nuser_jnt, m->jnt_user               )); }
  val  dof_bodyid             () const { return val(typed_memory_view(m->nv              * 1        , m->dof_bodyid             )); }
  val  dof_jntid              () const { return val(typed_memory_view(m->nv              * 1        , m->dof_jntid              )); }
  val  dof_parentid           () const { return val(typed_memory_view(m->nv              * 1        , m->dof_parentid           )); }
  val  dof_Madr               () const { return val(typed_memory_view(m->nv              * 1        , m->dof_Madr               )); }
  val  dof_simplenum          () const { return val(typed_memory_view(m->nv              * 1        , m->dof_simplenum          )); }
  val  dof_solref             () const { return val(typed_memory_view(m->nv              * mjNREF   , m->dof_solref             )); }
  val  dof_solimp             () const { return val(typed_memory_view(m->nv              * mjNIMP   , m->dof_solimp             )); }
  val  dof_frictionloss       () const { return val(typed_memory_view(m->nv              * 1        , m->dof_frictionloss       )); }
  val  dof_armature           () const { return val(typed_memory_view(m->nv              * 1        , m->dof_armature           )); }
  val  dof_damping            () const { return val(typed_memory_view(m->nv              * 1        , m->dof_damping            )); }
  val  dof_invweight0         () const { return val(typed_memory_view(m->nv              * 1        , m->dof_invweight0         )); }
  val  dof_M0                 () const { return val(typed_memory_view(m->nv              * 1        , m->dof_M0                 )); }
  val  geom_type              () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_type              )); }
  val  geom_contype           () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_contype           )); }
  val  geom_conaffinity       () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_conaffinity       )); }
  val  geom_condim            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_condim            )); }
  val  geom_bodyid            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_bodyid            )); }
  val  geom_dataid            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_dataid            )); }
  val  geom_matid             () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_matid             )); }
  val  geom_group             () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_group             )); }
  val  geom_priority          () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_priority          )); }
  val  geom_sameframe         () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_sameframe         )); }
  val  geom_solmix            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_solmix            )); }
  val  geom_solref            () const { return val(typed_memory_view(m->ngeom           * mjNREF   , m->geom_solref            )); }
  val  geom_solimp            () const { return val(typed_memory_view(m->ngeom           * mjNIMP   , m->geom_solimp            )); }
  val  geom_size              () const { return val(typed_memory_view(m->ngeom           * 3        , m->geom_size              )); }
  val  geom_rbound            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_rbound            )); }
  val  geom_pos               () const { return val(typed_memory_view(m->ngeom           * 3        , m->geom_pos               )); }
  val  geom_quat              () const { return val(typed_memory_view(m->ngeom           * 4        , m->geom_quat              )); }
  val  geom_friction          () const { return val(typed_memory_view(m->ngeom           * 3        , m->geom_friction          )); }
  val  geom_margin            () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_margin            )); }
  val  geom_gap               () const { return val(typed_memory_view(m->ngeom           * 1        , m->geom_gap               )); }
  val  geom_fluid             () const { return val(typed_memory_view(m->ngeom           * mjNFLUID , m->geom_fluid             )); }
  val  geom_user              () const { return val(typed_memory_view(m->ngeom           * m->nuser_geom, m->geom_user              )); }
  val  geom_rgba              () const { return val(typed_memory_view(m->ngeom           * 4        , m->geom_rgba              )); }
  val  site_type              () const { return val(typed_memory_view(m->nsite           * 1        , m->site_type              )); }
  val  site_bodyid            () const { return val(typed_memory_view(m->nsite           * 1        , m->site_bodyid            )); }
  val  site_matid             () const { return val(typed_memory_view(m->nsite           * 1        , m->site_matid             )); }
  val  site_group             () const { return val(typed_memory_view(m->nsite           * 1        , m->site_group             )); }
  val  site_sameframe         () const { return val(typed_memory_view(m->nsite           * 1        , m->site_sameframe         )); }
  val  site_size              () const { return val(typed_memory_view(m->nsite           * 3        , m->site_size              )); }
  val  site_pos               () const { return val(typed_memory_view(m->nsite           * 3        , m->site_pos               )); }
  val  site_quat              () const { return val(typed_memory_view(m->nsite           * 4        , m->site_quat              )); }
  val  site_user              () const { return val(typed_memory_view(m->nsite           * m->nuser_site, m->site_user              )); }
  val  site_rgba              () const { return val(typed_memory_view(m->nsite           * 4        , m->site_rgba              )); }
  val  cam_mode               () const { return val(typed_memory_view(m->ncam            * 1        , m->cam_mode               )); }
  val  cam_bodyid             () const { return val(typed_memory_view(m->ncam            * 1        , m->cam_bodyid             )); }
  val  cam_targetbodyid       () const { return val(typed_memory_view(m->ncam            * 1        , m->cam_targetbodyid       )); }
  val  cam_pos                () const { return val(typed_memory_view(m->ncam            * 3        , m->cam_pos                )); }
  val  cam_quat               () const { return val(typed_memory_view(m->ncam            * 4        , m->cam_quat               )); }
  val  cam_poscom0            () const { return val(typed_memory_view(m->ncam            * 3        , m->cam_poscom0            )); }
  val  cam_pos0               () const { return val(typed_memory_view(m->ncam            * 3        , m->cam_pos0               )); }
  val  cam_mat0               () const { return val(typed_memory_view(m->ncam            * 9        , m->cam_mat0               )); }
  val  cam_fovy               () const { return val(typed_memory_view(m->ncam            * 1        , m->cam_fovy               )); }
  val  cam_ipd                () const { return val(typed_memory_view(m->ncam            * 1        , m->cam_ipd                )); }
  val  cam_user               () const { return val(typed_memory_view(m->ncam            * m->nuser_cam, m->cam_user               )); }
  val  light_mode             () const { return val(typed_memory_view(m->nlight          * 1        , m->light_mode             )); }
  val  light_bodyid           () const { return val(typed_memory_view(m->nlight          * 1        , m->light_bodyid           )); }
  val  light_targetbodyid     () const { return val(typed_memory_view(m->nlight          * 1        , m->light_targetbodyid     )); }
  val  light_directional      () const { return val(typed_memory_view(m->nlight          * 1        , m->light_directional      )); }
  val  light_castshadow       () const { return val(typed_memory_view(m->nlight          * 1        , m->light_castshadow       )); }
  val  light_active           () const { return val(typed_memory_view(m->nlight          * 1        , m->light_active           )); }
  val  light_pos              () const { return val(typed_memory_view(m->nlight          * 3        , m->light_pos              )); }
  val  light_dir              () const { return val(typed_memory_view(m->nlight          * 3        , m->light_dir              )); }
  val  light_poscom0          () const { return val(typed_memory_view(m->nlight          * 3        , m->light_poscom0          )); }
  val  light_pos0             () const { return val(typed_memory_view(m->nlight          * 3        , m->light_pos0             )); }
  val  light_dir0             () const { return val(typed_memory_view(m->nlight          * 3        , m->light_dir0             )); }
  val  light_attenuation      () const { return val(typed_memory_view(m->nlight          * 3        , m->light_attenuation      )); }
  val  light_cutoff           () const { return val(typed_memory_view(m->nlight          * 1        , m->light_cutoff           )); }
  val  light_exponent         () const { return val(typed_memory_view(m->nlight          * 1        , m->light_exponent         )); }
  val  light_ambient          () const { return val(typed_memory_view(m->nlight          * 3        , m->light_ambient          )); }
  val  light_diffuse          () const { return val(typed_memory_view(m->nlight          * 3        , m->light_diffuse          )); }
  val  light_specular         () const { return val(typed_memory_view(m->nlight          * 3        , m->light_specular         )); }
  val  mesh_vertadr           () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_vertadr           )); }
  val  mesh_vertnum           () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_vertnum           )); }
  val  mesh_texcoordadr       () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_texcoordadr       )); }
  val  mesh_faceadr           () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_faceadr           )); }
  val  mesh_facenum           () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_facenum           )); }
  val  mesh_graphadr          () const { return val(typed_memory_view(m->nmesh           * 1        , m->mesh_graphadr          )); }
  val  mesh_vert              () const { return val(typed_memory_view(m->nmeshvert       * 3        , m->mesh_vert              )); }
  val  mesh_normal            () const { return val(typed_memory_view(m->nmeshvert       * 3        , m->mesh_normal            )); }
  val  mesh_texcoord          () const { return val(typed_memory_view(m->nmeshtexvert    * 2        , m->mesh_texcoord          )); }
  val  mesh_face              () const { return val(typed_memory_view(m->nmeshface       * 3        , m->mesh_face              )); }
  val  mesh_graph             () const { return val(typed_memory_view(m->nmeshgraph      * 1        , m->mesh_graph             )); }
  val  skin_matid             () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_matid             )); }
  val  skin_group             () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_group             )); }
  val  skin_rgba              () const { return val(typed_memory_view(m->nskin           * 4        , m->skin_rgba              )); }
  val  skin_inflate           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_inflate           )); }
  val  skin_vertadr           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_vertadr           )); }
  val  skin_vertnum           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_vertnum           )); }
  val  skin_texcoordadr       () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_texcoordadr       )); }
  val  skin_faceadr           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_faceadr           )); }
  val  skin_facenum           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_facenum           )); }
  val  skin_boneadr           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_boneadr           )); }
  val  skin_bonenum           () const { return val(typed_memory_view(m->nskin           * 1        , m->skin_bonenum           )); }
  val  skin_vert              () const { return val(typed_memory_view(m->nskinvert       * 3        , m->skin_vert              )); }
  val  skin_texcoord          () const { return val(typed_memory_view(m->nskintexvert    * 2        , m->skin_texcoord          )); }
  val  skin_face              () const { return val(typed_memory_view(m->nskinface       * 3        , m->skin_face              )); }
  val  skin_bonevertadr       () const { return val(typed_memory_view(m->nskinbone       * 1        , m->skin_bonevertadr       )); }
  val  skin_bonevertnum       () const { return val(typed_memory_view(m->nskinbone       * 1        , m->skin_bonevertnum       )); }
  val  skin_bonebindpos       () const { return val(typed_memory_view(m->nskinbone       * 3        , m->skin_bonebindpos       )); }
  val  skin_bonebindquat      () const { return val(typed_memory_view(m->nskinbone       * 4        , m->skin_bonebindquat      )); }
  val  skin_bonebodyid        () const { return val(typed_memory_view(m->nskinbone       * 1        , m->skin_bonebodyid        )); }
  val  skin_bonevertid        () const { return val(typed_memory_view(m->nskinbonevert   * 1        , m->skin_bonevertid        )); }
  val  skin_bonevertweight    () const { return val(typed_memory_view(m->nskinbonevert   * 1        , m->skin_bonevertweight    )); }
  val  hfield_size            () const { return val(typed_memory_view(m->nhfield         * 4        , m->hfield_size            )); }
  val  hfield_nrow            () const { return val(typed_memory_view(m->nhfield         * 1        , m->hfield_nrow            )); }
  val  hfield_ncol            () const { return val(typed_memory_view(m->nhfield         * 1        , m->hfield_ncol            )); }
  val  hfield_adr             () const { return val(typed_memory_view(m->nhfield         * 1        , m->hfield_adr             )); }
  val  hfield_data            () const { return val(typed_memory_view(m->nhfielddata     * 1        , m->hfield_data            )); }
  val  tex_type               () const { return val(typed_memory_view(m->ntex            * 1        , m->tex_type               )); }
  val  tex_height             () const { return val(typed_memory_view(m->ntex            * 1        , m->tex_height             )); }
  val  tex_width              () const { return val(typed_memory_view(m->ntex            * 1        , m->tex_width              )); }
  val  tex_adr                () const { return val(typed_memory_view(m->ntex            * 1        , m->tex_adr                )); }
  val  tex_rgb                () const { return val(typed_memory_view(m->ntexdata        * 1        , m->tex_rgb                )); }
  val  mat_texid              () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_texid              )); }
  val  mat_texuniform         () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_texuniform         )); }
  val  mat_texrepeat          () const { return val(typed_memory_view(m->nmat            * 2        , m->mat_texrepeat          )); }
  val  mat_emission           () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_emission           )); }
  val  mat_specular           () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_specular           )); }
  val  mat_shininess          () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_shininess          )); }
  val  mat_reflectance        () const { return val(typed_memory_view(m->nmat            * 1        , m->mat_reflectance        )); }
  val  mat_rgba               () const { return val(typed_memory_view(m->nmat            * 4        , m->mat_rgba               )); }
  val  pair_dim               () const { return val(typed_memory_view(m->npair           * 1        , m->pair_dim               )); }
  val  pair_geom1             () const { return val(typed_memory_view(m->npair           * 1        , m->pair_geom1             )); }
  val  pair_geom2             () const { return val(typed_memory_view(m->npair           * 1        , m->pair_geom2             )); }
  val  pair_signature         () const { return val(typed_memory_view(m->npair           * 1        , m->pair_signature         )); }
  val  pair_solref            () const { return val(typed_memory_view(m->npair           * mjNREF   , m->pair_solref            )); }
  val  pair_solimp            () const { return val(typed_memory_view(m->npair           * mjNIMP   , m->pair_solimp            )); }
  val  pair_margin            () const { return val(typed_memory_view(m->npair           * 1        , m->pair_margin            )); }
  val  pair_gap               () const { return val(typed_memory_view(m->npair           * 1        , m->pair_gap               )); }
  val  pair_friction          () const { return val(typed_memory_view(m->npair           * 5        , m->pair_friction          )); }
  val  exclude_signature      () const { return val(typed_memory_view(m->nexclude        * 1        , m->exclude_signature      )); }
  val  eq_type                () const { return val(typed_memory_view(m->neq             * 1        , m->eq_type                )); }
  val  eq_obj1id              () const { return val(typed_memory_view(m->neq             * 1        , m->eq_obj1id              )); }
  val  eq_obj2id              () const { return val(typed_memory_view(m->neq             * 1        , m->eq_obj2id              )); }
  val  eq_active              () const { return val(typed_memory_view(m->neq             * 1        , m->eq_active              )); }
  val  eq_solref              () const { return val(typed_memory_view(m->neq             * mjNREF   , m->eq_solref              )); }
  val  eq_solimp              () const { return val(typed_memory_view(m->neq             * mjNIMP   , m->eq_solimp              )); }
  val  eq_data                () const { return val(typed_memory_view(m->neq             * mjNEQDATA, m->eq_data                )); }
  val  tendon_adr             () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_adr             )); }
  val  tendon_num             () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_num             )); }
  val  tendon_matid           () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_matid           )); }
  val  tendon_group           () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_group           )); }
  val  tendon_limited         () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_limited         )); }
  val  tendon_width           () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_width           )); }
  val  tendon_solref_lim      () const { return val(typed_memory_view(m->ntendon         * mjNREF   , m->tendon_solref_lim      )); }
  val  tendon_solimp_lim      () const { return val(typed_memory_view(m->ntendon         * mjNIMP   , m->tendon_solimp_lim      )); }
  val  tendon_solref_fri      () const { return val(typed_memory_view(m->ntendon         * mjNREF   , m->tendon_solref_fri      )); }
  val  tendon_solimp_fri      () const { return val(typed_memory_view(m->ntendon         * mjNIMP   , m->tendon_solimp_fri      )); }
  val  tendon_range           () const { return val(typed_memory_view(m->ntendon         * 2        , m->tendon_range           )); }
  val  tendon_margin          () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_margin          )); }
  val  tendon_stiffness       () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_stiffness       )); }
  val  tendon_damping         () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_damping         )); }
  val  tendon_frictionloss    () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_frictionloss    )); }
  val  tendon_lengthspring    () const { return val(typed_memory_view(m->ntendon         * 2        , m->tendon_lengthspring    )); }
  val  tendon_length0         () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_length0         )); }
  val  tendon_invweight0      () const { return val(typed_memory_view(m->ntendon         * 1        , m->tendon_invweight0      )); }
  val  tendon_user            () const { return val(typed_memory_view(m->ntendon         * m->nuser_tendon, m->tendon_user            )); }
  val  tendon_rgba            () const { return val(typed_memory_view(m->ntendon         * 4        , m->tendon_rgba            )); }
  val  wrap_type              () const { return val(typed_memory_view(m->nwrap           * 1        , m->wrap_type              )); }
  val  wrap_objid             () const { return val(typed_memory_view(m->nwrap           * 1        , m->wrap_objid             )); }
  val  wrap_prm               () const { return val(typed_memory_view(m->nwrap           * 1        , m->wrap_prm               )); }
  val  actuator_trntype       () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_trntype       )); }
  val  actuator_dyntype       () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_dyntype       )); }
  val  actuator_gaintype      () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_gaintype      )); }
  val  actuator_biastype      () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_biastype      )); }
  val  actuator_trnid         () const { return val(typed_memory_view(m->nu              * 2        , m->actuator_trnid         )); }
  val  actuator_actadr        () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_actadr        )); }
  val  actuator_actnum        () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_actnum        )); }
  val  actuator_group         () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_group         )); }
  val  actuator_ctrllimited   () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_ctrllimited   )); }
  val  actuator_forcelimited  () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_forcelimited  )); }
  val  actuator_actlimited    () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_actlimited    )); }
  val  actuator_dynprm        () const { return val(typed_memory_view(m->nu              * mjNDYN   , m->actuator_dynprm        )); }
  val  actuator_gainprm       () const { return val(typed_memory_view(m->nu              * mjNGAIN  , m->actuator_gainprm       )); }
  val  actuator_biasprm       () const { return val(typed_memory_view(m->nu              * mjNBIAS  , m->actuator_biasprm       )); }
  val  actuator_ctrlrange     () const { return val(typed_memory_view(m->nu              * 2        , m->actuator_ctrlrange     )); }
  val  actuator_forcerange    () const { return val(typed_memory_view(m->nu              * 2        , m->actuator_forcerange    )); }
  val  actuator_actrange      () const { return val(typed_memory_view(m->nu              * 2        , m->actuator_actrange      )); }
  val  actuator_gear          () const { return val(typed_memory_view(m->nu              * 6        , m->actuator_gear          )); }
  val  actuator_cranklength   () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_cranklength   )); }
  val  actuator_acc0          () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_acc0          )); }
  val  actuator_length0       () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_length0       )); }
  val  actuator_lengthrange   () const { return val(typed_memory_view(m->nu              * 2        , m->actuator_lengthrange   )); }
  val  actuator_user          () const { return val(typed_memory_view(m->nu              * m->nuser_actuator, m->actuator_user          )); }
  val  actuator_plugin        () const { return val(typed_memory_view(m->nu              * 1        , m->actuator_plugin        )); }
  val  sensor_type            () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_type            )); }
  val  sensor_datatype        () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_datatype        )); }
  val  sensor_needstage       () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_needstage       )); }
  val  sensor_objtype         () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_objtype         )); }
  val  sensor_objid           () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_objid           )); }
  val  sensor_reftype         () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_reftype         )); }
  val  sensor_refid           () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_refid           )); }
  val  sensor_dim             () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_dim             )); }
  val  sensor_adr             () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_adr             )); }
  val  sensor_cutoff          () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_cutoff          )); }
  val  sensor_noise           () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_noise           )); }
  val  sensor_user            () const { return val(typed_memory_view(m->nsensor         * m->nuser_sensor, m->sensor_user            )); }
  val  sensor_plugin          () const { return val(typed_memory_view(m->nsensor         * 1        , m->sensor_plugin          )); }
  val  plugin                 () const { return val(typed_memory_view(m->nplugin         * 1        , m->plugin                 )); }
  val  plugin_stateadr        () const { return val(typed_memory_view(m->nplugin         * 1        , m->plugin_stateadr        )); }
  val  plugin_statenum        () const { return val(typed_memory_view(m->nplugin         * 1        , m->plugin_statenum        )); }
  val  plugin_attr            () const { return val(typed_memory_view(m->npluginattr     * 1        , m->plugin_attr            )); }
  val  plugin_attradr         () const { return val(typed_memory_view(m->nplugin         * 1        , m->plugin_attradr         )); }
  val  numeric_adr            () const { return val(typed_memory_view(m->nnumeric        * 1        , m->numeric_adr            )); }
  val  numeric_size           () const { return val(typed_memory_view(m->nnumeric        * 1        , m->numeric_size           )); }
  val  numeric_data           () const { return val(typed_memory_view(m->nnumericdata    * 1        , m->numeric_data           )); }
  val  text_adr               () const { return val(typed_memory_view(m->ntext           * 1        , m->text_adr               )); }
  val  text_size              () const { return val(typed_memory_view(m->ntext           * 1        , m->text_size              )); }
  val  text_data              () const { return val(typed_memory_view(m->ntextdata       * 1        , m->text_data              )); }
  val  tuple_adr              () const { return val(typed_memory_view(m->ntuple          * 1        , m->tuple_adr              )); }
  val  tuple_size             () const { return val(typed_memory_view(m->ntuple          * 1        , m->tuple_size             )); }
  val  tuple_objtype          () const { return val(typed_memory_view(m->ntupledata      * 1        , m->tuple_objtype          )); }
  val  tuple_objid            () const { return val(typed_memory_view(m->ntupledata      * 1        , m->tuple_objid            )); }
  val  tuple_objprm           () const { return val(typed_memory_view(m->ntupledata      * 1        , m->tuple_objprm           )); }
  val  key_time               () const { return val(typed_memory_view(m->nkey            * 1        , m->key_time               )); }
  val  key_qpos               () const { return val(typed_memory_view(m->nkey            * m->nq    , m->key_qpos               )); }
  val  key_qvel               () const { return val(typed_memory_view(m->nkey            * m->nv    , m->key_qvel               )); }
  val  key_act                () const { return val(typed_memory_view(m->nkey            * m->na    , m->key_act                )); }
  val  key_mpos               () const { return val(typed_memory_view(m->nkey            * m->nmocap, m->key_mpos               )); }
  val  key_mquat              () const { return val(typed_memory_view(m->nkey            * m->nmocap, m->key_mquat              )); }
  val  key_ctrl               () const { return val(typed_memory_view(m->nkey            * m->nu    , m->key_ctrl               )); }
  val  name_bodyadr           () const { return val(typed_memory_view(m->nbody           * 1        , m->name_bodyadr           )); }
  val  name_jntadr            () const { return val(typed_memory_view(m->njnt            * 1        , m->name_jntadr            )); }
  val  name_geomadr           () const { return val(typed_memory_view(m->ngeom           * 1        , m->name_geomadr           )); }
  val  name_siteadr           () const { return val(typed_memory_view(m->nsite           * 1        , m->name_siteadr           )); }
  val  name_camadr            () const { return val(typed_memory_view(m->ncam            * 1        , m->name_camadr            )); }
  val  name_lightadr          () const { return val(typed_memory_view(m->nlight          * 1        , m->name_lightadr          )); }
  val  name_meshadr           () const { return val(typed_memory_view(m->nmesh           * 1        , m->name_meshadr           )); }
  val  name_skinadr           () const { return val(typed_memory_view(m->nskin           * 1        , m->name_skinadr           )); }
  val  name_hfieldadr         () const { return val(typed_memory_view(m->nhfield         * 1        , m->name_hfieldadr         )); }
  val  name_texadr            () const { return val(typed_memory_view(m->ntex            * 1        , m->name_texadr            )); }
  val  name_matadr            () const { return val(typed_memory_view(m->nmat            * 1        , m->name_matadr            )); }
  val  name_pairadr           () const { return val(typed_memory_view(m->npair           * 1        , m->name_pairadr           )); }
  val  name_excludeadr        () const { return val(typed_memory_view(m->nexclude        * 1        , m->name_excludeadr        )); }
  val  name_eqadr             () const { return val(typed_memory_view(m->neq             * 1        , m->name_eqadr             )); }
  val  name_tendonadr         () const { return val(typed_memory_view(m->ntendon         * 1        , m->name_tendonadr         )); }
  val  name_actuatoradr       () const { return val(typed_memory_view(m->nu              * 1        , m->name_actuatoradr       )); }
  val  name_sensoradr         () const { return val(typed_memory_view(m->nsensor         * 1        , m->name_sensoradr         )); }
  val  name_numericadr        () const { return val(typed_memory_view(m->nnumeric        * 1        , m->name_numericadr        )); }
  val  name_textadr           () const { return val(typed_memory_view(m->ntext           * 1        , m->name_textadr           )); }
  val  name_tupleadr          () const { return val(typed_memory_view(m->ntuple          * 1        , m->name_tupleadr          )); }
  val  name_keyadr            () const { return val(typed_memory_view(m->nkey            * 1        , m->name_keyadr            )); }
  val  name_pluginadr         () const { return val(typed_memory_view(m->nplugin         * 1        , m->name_pluginadr         )); }
  val  names                  () const { return val(typed_memory_view(m->nnames          * 1        , m->names                  )); }

private:
  mjModel *m;
};

class State {
public:
  State(Model m)  { d = mj_makeData(m.ptr()); }
  mjData *ptr  () { return d; }
  mjData getVal() { return *d; }
  void free    () { return mju_free(d); }

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
  void    free() { mju_free(_state); mju_free(_model); }

  void applyForce(
    mjtNum fx, mjtNum fy, mjtNum fz, 
    mjtNum tx, mjtNum ty, mjtNum tz,  
    mjtNum px, mjtNum py, mjtNum pz, int body) {
    mjtNum force [3] = {fx, fy, fz};
    mjtNum torque[3] = {tx, ty, tz};
    mjtNum point [3] = {px, py, pz};
    mj_applyFT(_model->ptr(), _state->ptr(), 
               force, torque, point, body, 
               _state->ptr()->qfrc_applied);
  }

  // copied from the source of mjv_applyPerturbPose
  // sets perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
  //  d->qpos written only if flg_paused and subtree root for selected body has free joint
  void applyPose(int bodyID,
                 mjtNum refPosX,  mjtNum refPosY,  mjtNum refPosZ,
                 mjtNum refQuat1, mjtNum refQuat2, mjtNum refQuat3, mjtNum refQuat4,
                 int flg_paused) {
    int rootid = 0, sel = bodyID;//pert->select;
    mjtNum pos1[3], quat1[4], pos2[3], quat2[4], refpos[3], refquat[4];
    mjtNum *Rpos, *Rquat, *Cpos, *Cquat;
    mjtNum inrefpos [3] = { refPosX , refPosY , refPosZ };
    mjtNum inrefquat[4] = { refQuat1, refQuat2, refQuat3, refQuat4 };
    mjModel *m = _model->ptr();
    mjData  *d = _state->ptr();

    // exit if nothing to do
    //if (sel<=0 || sel>=m->nbody || !(pert->active | pert->active2)) { return; }

    // get rootid above selected body
    rootid = m->body_rootid[sel];

    // transform refpos,refquat from I-frame to X-frame of body[sel]
    mju_negPose(pos1, quat1, m->body_ipos+3*sel, m->body_iquat+4*sel);
    mju_mulPose(refpos, refquat, inrefpos, inrefquat, pos1, quat1);

    // mocap body
    if (m->body_mocapid[sel]>=0) {
      // copy ref pose into mocap pose
      mju_copy3(d->mocap_pos + 3*m->body_mocapid[sel], refpos);
      mju_copy4(d->mocap_quat + 4*m->body_mocapid[sel], refquat);
    }

    // floating body, paused
    else if (flg_paused && m->body_jntnum[sel]==1 &&
            m->jnt_type[m->body_jntadr[sel]]==mjJNT_FREE) {
      // copy ref pose into qpos
      mju_copy3(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]], refpos);
      mju_copy4(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]] + 3, refquat);
    }

    // child of floating body, paused
    else if (flg_paused && m->body_jntnum[rootid]==1 &&
            m->jnt_type[m->body_jntadr[rootid]]==mjJNT_FREE) {
      // get pointers to root
      Rpos = d->qpos + m->jnt_qposadr[m->body_jntadr[rootid]];
      Rquat = Rpos + 3;

      // get pointers to child
      Cpos = d->xpos + 3*sel;
      Cquat = d->xquat + 4*sel;

      // set root <- ref*neg(child)*root
      mju_negPose(pos1, quat1, Cpos, Cquat);                      // neg(child)
      mju_mulPose(pos2, quat2, pos1, quat1, Rpos, Rquat);         // neg(child)*root
      mju_mulPose(Rpos, Rquat, refpos, refquat, pos2, quat2);     // ref*neg(child)*root
    }
  }

  // MJDATA_DEFINITIONS
  val  qpos                   () const { return val(typed_memory_view(_model->ptr()->nq              * 1        , _state->ptr()->qpos                   )); }
  val  qvel                   () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qvel                   )); }
  val  act                    () const { return val(typed_memory_view(_model->ptr()->na              * 1        , _state->ptr()->act                    )); }
  val  qacc_warmstart         () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qacc_warmstart         )); }
  val  plugin_state           () const { return val(typed_memory_view(_model->ptr()->npluginstate    * 1        , _state->ptr()->plugin_state           )); }
  val  ctrl                   () const { return val(typed_memory_view(_model->ptr()->nu              * 1        , _state->ptr()->ctrl                   )); }
  val  qfrc_applied           () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_applied           )); }
  val  xfrc_applied           () const { return val(typed_memory_view(_model->ptr()->nbody           * 6        , _state->ptr()->xfrc_applied           )); }
  val  mocap_pos              () const { return val(typed_memory_view(_model->ptr()->nmocap          * 3        , _state->ptr()->mocap_pos              )); }
  val  mocap_quat             () const { return val(typed_memory_view(_model->ptr()->nmocap          * 4        , _state->ptr()->mocap_quat             )); }
  val  qacc                   () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qacc                   )); }
  val  act_dot                () const { return val(typed_memory_view(_model->ptr()->na              * 1        , _state->ptr()->act_dot                )); }
  val  userdata               () const { return val(typed_memory_view(_model->ptr()->nuserdata       * 1        , _state->ptr()->userdata               )); }
  val  sensordata             () const { return val(typed_memory_view(_model->ptr()->nsensordata     * 1        , _state->ptr()->sensordata             )); }
  val  plugin                 () const { return val(typed_memory_view(_model->ptr()->nplugin         * 1        , _state->ptr()->plugin                 )); }
  val  plugin_data            () const { return val(typed_memory_view(_model->ptr()->nplugin         * 1        , _state->ptr()->plugin_data            )); }
  val  xpos                   () const { return val(typed_memory_view(_model->ptr()->nbody           * 3        , _state->ptr()->xpos                   )); }
  val  xquat                  () const { return val(typed_memory_view(_model->ptr()->nbody           * 4        , _state->ptr()->xquat                  )); }
  val  xmat                   () const { return val(typed_memory_view(_model->ptr()->nbody           * 9        , _state->ptr()->xmat                   )); }
  val  xipos                  () const { return val(typed_memory_view(_model->ptr()->nbody           * 3        , _state->ptr()->xipos                  )); }
  val  ximat                  () const { return val(typed_memory_view(_model->ptr()->nbody           * 9        , _state->ptr()->ximat                  )); }
  val  xanchor                () const { return val(typed_memory_view(_model->ptr()->njnt            * 3        , _state->ptr()->xanchor                )); }
  val  xaxis                  () const { return val(typed_memory_view(_model->ptr()->njnt            * 3        , _state->ptr()->xaxis                  )); }
  val  geom_xpos              () const { return val(typed_memory_view(_model->ptr()->ngeom           * 3        , _state->ptr()->geom_xpos              )); }
  val  geom_xmat              () const { return val(typed_memory_view(_model->ptr()->ngeom           * 9        , _state->ptr()->geom_xmat              )); }
  val  site_xpos              () const { return val(typed_memory_view(_model->ptr()->nsite           * 3        , _state->ptr()->site_xpos              )); }
  val  site_xmat              () const { return val(typed_memory_view(_model->ptr()->nsite           * 9        , _state->ptr()->site_xmat              )); }
  val  cam_xpos               () const { return val(typed_memory_view(_model->ptr()->ncam            * 3        , _state->ptr()->cam_xpos               )); }
  val  cam_xmat               () const { return val(typed_memory_view(_model->ptr()->ncam            * 9        , _state->ptr()->cam_xmat               )); }
  val  light_xpos             () const { return val(typed_memory_view(_model->ptr()->nlight          * 3        , _state->ptr()->light_xpos             )); }
  val  light_xdir             () const { return val(typed_memory_view(_model->ptr()->nlight          * 3        , _state->ptr()->light_xdir             )); }
  val  subtree_com            () const { return val(typed_memory_view(_model->ptr()->nbody           * 3        , _state->ptr()->subtree_com            )); }
  val  cdof                   () const { return val(typed_memory_view(_model->ptr()->nv              * 6        , _state->ptr()->cdof                   )); }
  val  cinert                 () const { return val(typed_memory_view(_model->ptr()->nbody           * 10       , _state->ptr()->cinert                 )); }
  val  ten_wrapadr            () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_wrapadr            )); }
  val  ten_wrapnum            () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_wrapnum            )); }
  val  ten_J_rownnz           () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_J_rownnz           )); }
  val  ten_J_rowadr           () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_J_rowadr           )); }
  val  ten_J_colind           () const { return val(typed_memory_view(_model->ptr()->ntendon         * _model->ptr()->nv, _state->ptr()->ten_J_colind           )); }
  val  ten_length             () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_length             )); }
  val  ten_J                  () const { return val(typed_memory_view(_model->ptr()->ntendon         * _model->ptr()->nv, _state->ptr()->ten_J                  )); }
  val  wrap_obj               () const { return val(typed_memory_view(_model->ptr()->nwrap           * 2        , _state->ptr()->wrap_obj               )); }
  val  wrap_xpos              () const { return val(typed_memory_view(_model->ptr()->nwrap           * 6        , _state->ptr()->wrap_xpos              )); }
  val  actuator_length        () const { return val(typed_memory_view(_model->ptr()->nu              * 1        , _state->ptr()->actuator_length        )); }
  val  actuator_moment        () const { return val(typed_memory_view(_model->ptr()->nu              * _model->ptr()->nv, _state->ptr()->actuator_moment        )); }
  val  crb                    () const { return val(typed_memory_view(_model->ptr()->nbody           * 10       , _state->ptr()->crb                    )); }
  val  qM                     () const { return val(typed_memory_view(_model->ptr()->nM              * 1        , _state->ptr()->qM                     )); }
  val  qLD                    () const { return val(typed_memory_view(_model->ptr()->nM              * 1        , _state->ptr()->qLD                    )); }
  val  qLDiagInv              () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qLDiagInv              )); }
  val  qLDiagSqrtInv          () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qLDiagSqrtInv          )); }
  val  ten_velocity           () const { return val(typed_memory_view(_model->ptr()->ntendon         * 1        , _state->ptr()->ten_velocity           )); }
  val  actuator_velocity      () const { return val(typed_memory_view(_model->ptr()->nu              * 1        , _state->ptr()->actuator_velocity      )); }
  val  cvel                   () const { return val(typed_memory_view(_model->ptr()->nbody           * 6        , _state->ptr()->cvel                   )); }
  val  cdof_dot               () const { return val(typed_memory_view(_model->ptr()->nv              * 6        , _state->ptr()->cdof_dot               )); }
  val  qfrc_bias              () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_bias              )); }
  val  qfrc_passive           () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_passive           )); }
  val  subtree_linvel         () const { return val(typed_memory_view(_model->ptr()->nbody           * 3        , _state->ptr()->subtree_linvel         )); }
  val  subtree_angmom         () const { return val(typed_memory_view(_model->ptr()->nbody           * 3        , _state->ptr()->subtree_angmom         )); }
  val  qH                     () const { return val(typed_memory_view(_model->ptr()->nM              * 1        , _state->ptr()->qH                     )); }
  val  qHDiagInv              () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qHDiagInv              )); }
  val  D_rownnz               () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->D_rownnz               )); }
  val  D_rowadr               () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->D_rowadr               )); }
  val  D_colind               () const { return val(typed_memory_view(_model->ptr()->nD              * 1        , _state->ptr()->D_colind               )); }
  val  qDeriv                 () const { return val(typed_memory_view(_model->ptr()->nD              * 1        , _state->ptr()->qDeriv                 )); }
  val  qLU                    () const { return val(typed_memory_view(_model->ptr()->nD              * 1        , _state->ptr()->qLU                    )); }
  val  actuator_force         () const { return val(typed_memory_view(_model->ptr()->nu              * 1        , _state->ptr()->actuator_force         )); }
  val  qfrc_actuator          () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_actuator          )); }
  val  qfrc_smooth            () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_smooth            )); }
  val  qacc_smooth            () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qacc_smooth            )); }
  val  qfrc_constraint        () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_constraint        )); }
  val  qfrc_inverse           () const { return val(typed_memory_view(_model->ptr()->nv              * 1        , _state->ptr()->qfrc_inverse           )); }
  val  cacc                   () const { return val(typed_memory_view(_model->ptr()->nbody           * 6        , _state->ptr()->cacc                   )); }
  val  cfrc_int               () const { return val(typed_memory_view(_model->ptr()->nbody           * 6        , _state->ptr()->cfrc_int               )); }
  val  cfrc_ext               () const { return val(typed_memory_view(_model->ptr()->nbody           * 6        , _state->ptr()->cfrc_ext               )); }
  void   freeLastXML         (                    ) { return mj_freeLastXML              (                    ); }
  void   step                (                    ) { return mj_step                     (_model->ptr(), _state->ptr()); }
  void   step1               (                    ) { return mj_step1                    (_model->ptr(), _state->ptr()); }
  void   step2               (                    ) { return mj_step2                    (_model->ptr(), _state->ptr()); }
  void   forward             (                    ) { return mj_forward                  (_model->ptr(), _state->ptr()); }
  void   inverse             (                    ) { return mj_inverse                  (_model->ptr(), _state->ptr()); }
  void   forwardSkip         (int skipstage, int skipsensor) { return mj_forwardSkip              (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void   inverseSkip         (int skipstage, int skipsensor) { return mj_inverseSkip              (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void   defaultSolRefImp    (val solref, val solimp) { return mj_defaultSolRefImp         (reinterpret_cast<mjtNum*>(solref["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(solimp["byteOffset"].as<int>())); }
  int    sizeModel           (                    ) { return mj_sizeModel                (_model->ptr()       ); }
  void   resetData           (                    ) { return mj_resetData                (_model->ptr(), _state->ptr()); }
  void   resetDataDebug      (unsigned char debug_value) { return mj_resetDataDebug           (_model->ptr(), _state->ptr(), debug_value); }
  void   resetDataKeyframe   (int key             ) { return mj_resetDataKeyframe        (_model->ptr(), _state->ptr(), key); }
  void   deleteData          (                    ) { return mj_deleteData               (_state->ptr()       ); }
  void   resetCallbacks      (                    ) { return mj_resetCallbacks           (                    ); }
  void   printFormattedModel (std::string filename, std::string float_format) { return mj_printFormattedModel      (_model->ptr(), filename.c_str(), float_format.c_str()); }
  void   printModel          (std::string filename) { return mj_printModel               (_model->ptr(), filename.c_str()); }
  void   printFormattedData  (std::string filename, std::string float_format) { return mj_printFormattedData       (_model->ptr(), _state->ptr(), filename.c_str(), float_format.c_str()); }
  void   printData           (std::string filename) { return mj_printData                (_model->ptr(), _state->ptr(), filename.c_str()); }
  void   _printMat           (val mat, int nr, int nc) { return mju_printMat                (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
  void   fwdPosition         (                    ) { return mj_fwdPosition              (_model->ptr(), _state->ptr()); }
  void   fwdVelocity         (                    ) { return mj_fwdVelocity              (_model->ptr(), _state->ptr()); }
  void   fwdActuation        (                    ) { return mj_fwdActuation             (_model->ptr(), _state->ptr()); }
  void   fwdAcceleration     (                    ) { return mj_fwdAcceleration          (_model->ptr(), _state->ptr()); }
  void   fwdConstraint       (                    ) { return mj_fwdConstraint            (_model->ptr(), _state->ptr()); }
  void   Euler               (                    ) { return mj_Euler                    (_model->ptr(), _state->ptr()); }
  void   RungeKutta          (int N               ) { return mj_RungeKutta               (_model->ptr(), _state->ptr(), N); }
  void   invPosition         (                    ) { return mj_invPosition              (_model->ptr(), _state->ptr()); }
  void   invVelocity         (                    ) { return mj_invVelocity              (_model->ptr(), _state->ptr()); }
  void   invConstraint       (                    ) { return mj_invConstraint            (_model->ptr(), _state->ptr()); }
  void   compareFwdInv       (                    ) { return mj_compareFwdInv            (_model->ptr(), _state->ptr()); }
  void   sensorPos           (                    ) { return mj_sensorPos                (_model->ptr(), _state->ptr()); }
  void   sensorVel           (                    ) { return mj_sensorVel                (_model->ptr(), _state->ptr()); }
  void   sensorAcc           (                    ) { return mj_sensorAcc                (_model->ptr(), _state->ptr()); }
  void   energyPos           (                    ) { return mj_energyPos                (_model->ptr(), _state->ptr()); }
  void   energyVel           (                    ) { return mj_energyVel                (_model->ptr(), _state->ptr()); }
  void   checkPos            (                    ) { return mj_checkPos                 (_model->ptr(), _state->ptr()); }
  void   checkVel            (                    ) { return mj_checkVel                 (_model->ptr(), _state->ptr()); }
  void   checkAcc            (                    ) { return mj_checkAcc                 (_model->ptr(), _state->ptr()); }
  void   kinematics          (                    ) { return mj_kinematics               (_model->ptr(), _state->ptr()); }
  void   comPos              (                    ) { return mj_comPos                   (_model->ptr(), _state->ptr()); }
  void   camlight            (                    ) { return mj_camlight                 (_model->ptr(), _state->ptr()); }
  void   tendon              (                    ) { return mj_tendon                   (_model->ptr(), _state->ptr()); }
  void   transmission        (                    ) { return mj_transmission             (_model->ptr(), _state->ptr()); }
  void   crbCalculate        (                    ) { return mj_crb                      (_model->ptr(), _state->ptr()); }
  void   factorM             (                    ) { return mj_factorM                  (_model->ptr(), _state->ptr()); }
  void   solveM              (val x, val y, int n ) { return mj_solveM                   (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void   solveM2             (val x, val y, int n ) { return mj_solveM2                  (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void   comVel              (                    ) { return mj_comVel                   (_model->ptr(), _state->ptr()); }
  void   passive             (                    ) { return mj_passive                  (_model->ptr(), _state->ptr()); }
  void   subtreeVel          (                    ) { return mj_subtreeVel               (_model->ptr(), _state->ptr()); }
  void   rne                 (int flg_acc, val result) { return mj_rne                      (_model->ptr(), _state->ptr(), flg_acc, reinterpret_cast<mjtNum*>(result["byteOffset"].as<int>())); }
  void   rnePostConstraint   (                    ) { return mj_rnePostConstraint        (_model->ptr(), _state->ptr()); }
  void   collision           (                    ) { return mj_collision                (_model->ptr(), _state->ptr()); }
  void   makeConstraint      (                    ) { return mj_makeConstraint           (_model->ptr(), _state->ptr()); }
  void   projectConstraint   (                    ) { return mj_projectConstraint        (_model->ptr(), _state->ptr()); }
  void   referenceConstraint (                    ) { return mj_referenceConstraint      (_model->ptr(), _state->ptr()); }
  int    isPyramidal         (                    ) { return mj_isPyramidal              (_model->ptr()       ); }
  int    isSparse            (                    ) { return mj_isSparse                 (_model->ptr()       ); }
  int    isDual              (                    ) { return mj_isDual                   (_model->ptr()       ); }
  void   mulJacVec           (val res, val vec    ) { return mj_mulJacVec                (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>())); }
  void   mulJacTVec          (val res, val vec    ) { return mj_mulJacTVec               (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>())); }
  void   jacSubtreeCom       (val jacp, int body  ) { return mj_jacSubtreeCom            (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(jacp["byteOffset"].as<int>()), body); }
  int    name2id             (int type, std::string name) { return mj_name2id                  (_model->ptr(), type, name.c_str()); }
  std::string id2name             (int type, int id    ) { return std::string(mj_id2name                  (_model->ptr(), type, id)); }
  void   fullM               (val dst, val M      ) { return mj_fullM                    (_model->ptr(), reinterpret_cast<mjtNum*>(dst["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(M["byteOffset"].as<int>())); }
  void   differentiatePos    (val qvel, mjtNum dt, val qpos1, val qpos2) { return mj_differentiatePos         (_model->ptr(), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt, reinterpret_cast<mjtNum*>(qpos1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qpos2["byteOffset"].as<int>())); }
  void   integratePos        (val qpos, val qvel, mjtNum dt) { return mj_integratePos             (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt); }
  void   normalizeQuat       (val qpos            ) { return mj_normalizeQuat            (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>())); }
  mjtNum getTotalmass        (                    ) { return mj_getTotalmass             (_model->ptr()       ); }
  std::string getPluginConfig     (int plugin_id, std::string attrib) { return std::string(mj_getPluginConfig          (_model->ptr(), plugin_id, attrib.c_str())); }
  void   loadPluginLibrary   (std::string path    ) { return mj_loadPluginLibrary        (path.c_str()        ); }
  int    version             (                    ) { return mj_version                  (                    ); }
  std::string versionString       (                    ) { return std::string(mj_versionString            (                    )); }
  void   _rectangle          (mjrRect viewport, float r, float g, float b, float a) { return mjr_rectangle               (viewport, r, g, b, a); }
  void   _finish             (                    ) { return mjr_finish                  (                    ); }
  int    _getError           (                    ) { return mjr_getError                (                    ); }
  mjuiThemeSpacing i_themeSpacing      (int ind             ) { return mjui_themeSpacing           (ind                 ); }
  mjuiThemeColor i_themeColor        (int ind             ) { return mjui_themeColor             (ind                 ); }
  void   _error              (std::string msg     ) { return mju_error                   (msg.c_str()         ); }
  void   _error_i            (std::string msg, int i) { return mju_error_i                 (msg.c_str(), i      ); }
  void   _error_s            (std::string msg, std::string text) { return mju_error_s                 (msg.c_str(), text.c_str()); }
  void   _warning            (std::string msg     ) { return mju_warning                 (msg.c_str()         ); }
  void   _warning_i          (std::string msg, int i) { return mju_warning_i               (msg.c_str(), i      ); }
  void   _warning_s          (std::string msg, std::string text) { return mju_warning_s               (msg.c_str(), text.c_str()); }
  void   _clearHandlers      (                    ) { return mju_clearHandlers           (                    ); }
  void   warning             (int warning, int info) { return mj_warning                  (_state->ptr(), warning, info); }
  void   _writeLog           (std::string type, std::string msg) { return mju_writeLog                (type.c_str(), msg.c_str()); }
  int    activate            (std::string filename) { return mj_activate                 (filename.c_str()    ); }
  void   deactivate          (                    ) { return mj_deactivate               (                    ); }
  void   _zero               (val res, int n      ) { return mju_zero                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  void   _fill               (val res, mjtNum val, int n) { return mju_fill                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), val, n); }
  void   _copy               (val res, val data, int n) { return mju_copy                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(data["byteOffset"].as<int>()), n); }
  mjtNum _sum                (val vec, int n      ) { return mju_sum                     (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum _L1                 (val vec, int n      ) { return mju_L1                      (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _scl                (val res, val vec, mjtNum scl, int n) { return mju_scl                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void   _add                (val res, val vec1, val vec2, int n) { return mju_add                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _sub                (val res, val vec1, val vec2, int n) { return mju_sub                     (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _addTo              (val res, val vec, int n) { return mju_addTo                   (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _subFrom            (val res, val vec, int n) { return mju_subFrom                 (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void   _addToScl           (val res, val vec, mjtNum scl, int n) { return mju_addToScl                (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void   _addScl             (val res, val vec1, val vec2, mjtNum scl, int n) { return mju_addScl                  (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), scl, n); }
  mjtNum _normalize          (val res, int n      ) { return mju_normalize               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum _norm               (val res, int n      ) { return mju_norm                    (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum _dot                (val vec1, val vec2, int n) { return mju_dot                     (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _mulMatVec          (val res, val mat, val vec, int nr, int nc) { return mju_mulMatVec               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  void   _mulMatTVec         (val res, val mat, val vec, int nr, int nc) { return mju_mulMatTVec              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  mjtNum _mulVecMatVec       (val vec1, val mat, val vec2, int n) { return mju_mulVecMatVec            (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void   _transpose          (val res, val mat, int nr, int nc) { return mju_transpose               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
  void   _symmetrize         (val res, val mat, int n) { return mju_symmetrize              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void   _eye                (val mat, int n      ) { return mju_eye                     (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void   _mulMatMat          (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatMat               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void   _mulMatMatT         (val res, val mat1, val mat2, int r1, int c1, int r2) { return mju_mulMatMatT              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, r2); }
  void   _mulMatTMat         (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatTMat              (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void   _sqrMatTD           (val res, val mat, val diag, int nr, int nc) { return mju_sqrMatTD                (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(diag["byteOffset"].as<int>()), nr, nc); }
  int    _cholFactor         (val mat, int n, mjtNum mindiag) { return mju_cholFactor              (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n, mindiag); }
  void   _cholSolve          (val res, val mat, val vec, int n) { return mju_cholSolve               (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  int    _cholUpdate         (val mat, val x, int n, int flg_plus) { return mju_cholUpdate              (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), n, flg_plus); }
  void   _encodePyramid      (val pyramid, val force, val mu, int dim) { return mju_encodePyramid           (reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  void   _decodePyramid      (val force, val pyramid, val mu, int dim) { return mju_decodePyramid           (reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  mjtNum _springDamper       (mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt) { return mju_springDamper            (pos0, vel0, Kp, Kv, dt); }
  mjtNum _min                (mjtNum a, mjtNum b  ) { return mju_min                     (a, b                ); }
  mjtNum _max                (mjtNum a, mjtNum b  ) { return mju_max                     (a, b                ); }
  mjtNum _clip               (mjtNum x, mjtNum min, mjtNum max) { return mju_clip                    (x, min, max         ); }
  mjtNum _sign               (mjtNum x            ) { return mju_sign                    (x                   ); }
  int    _round              (mjtNum x            ) { return mju_round                   (x                   ); }
  std::string _type2Str           (int type            ) { return std::string(mju_type2Str                (type                )); }
  int    _str2Type           (std::string str     ) { return mju_str2Type                (str.c_str()         ); }
  std::string _writeNumBytes      (size_t nbytes       ) { return std::string(mju_writeNumBytes           (nbytes              )); }
  std::string _warningText        (int warning, size_t info) { return std::string(mju_warningText             (warning, info       )); }
  int    _isBad              (mjtNum x            ) { return mju_isBad                   (x                   ); }
  int    _isZero             (val vec, int n      ) { return mju_isZero                  (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum _standardNormal     (val num2            ) { return mju_standardNormal          (reinterpret_cast<mjtNum*>(num2["byteOffset"].as<int>())); }
  void   _insertionSort      (val list, int n     ) { return mju_insertionSort           (reinterpret_cast<mjtNum*>(list["byteOffset"].as<int>()), n); }
  mjtNum _Halton             (int index, int base ) { return mju_Halton                  (index, base         ); }
  mjtNum _sigmoid            (mjtNum x            ) { return mju_sigmoid                 (x                   ); }
  void   _transitionFD       (mjtNum eps, mjtByte centered, val A, val B, val C, val D) { return mjd_transitionFD            (_model->ptr(), _state->ptr(), eps, centered, reinterpret_cast<mjtNum*>(A["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(B["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(C["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(D["byteOffset"].as<int>())); }
  int    _pluginCount        (                    ) { return mjp_pluginCount             (                    ); }


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

  // MODEL_ENUMS
  enum_<mjtDisableBit>("mjtDisableBit")
      .value("mjDSBL_CONSTRAINT"      , mjtDisableBit            ::mjDSBL_CONSTRAINT        )
      .value("mjDSBL_EQUALITY"        , mjtDisableBit            ::mjDSBL_EQUALITY          )
      .value("mjDSBL_FRICTIONLOSS"    , mjtDisableBit            ::mjDSBL_FRICTIONLOSS      )
      .value("mjDSBL_LIMIT"           , mjtDisableBit            ::mjDSBL_LIMIT             )
      .value("mjDSBL_CONTACT"         , mjtDisableBit            ::mjDSBL_CONTACT           )
      .value("mjDSBL_PASSIVE"         , mjtDisableBit            ::mjDSBL_PASSIVE           )
      .value("mjDSBL_GRAVITY"         , mjtDisableBit            ::mjDSBL_GRAVITY           )
      .value("mjDSBL_CLAMPCTRL"       , mjtDisableBit            ::mjDSBL_CLAMPCTRL         )
      .value("mjDSBL_WARMSTART"       , mjtDisableBit            ::mjDSBL_WARMSTART         )
      .value("mjDSBL_FILTERPARENT"    , mjtDisableBit            ::mjDSBL_FILTERPARENT      )
      .value("mjDSBL_ACTUATION"       , mjtDisableBit            ::mjDSBL_ACTUATION         )
      .value("mjDSBL_REFSAFE"         , mjtDisableBit            ::mjDSBL_REFSAFE           )
      .value("mjDSBL_SENSOR"          , mjtDisableBit            ::mjDSBL_SENSOR            )
      .value("mjNDISABLE"             , mjtDisableBit            ::mjNDISABLE               )
  ;
  enum_<mjtEnableBit>("mjtEnableBit")
      .value("mjENBL_OVERRIDE"        , mjtEnableBit             ::mjENBL_OVERRIDE          )
      .value("mjENBL_ENERGY"          , mjtEnableBit             ::mjENBL_ENERGY            )
      .value("mjENBL_FWDINV"          , mjtEnableBit             ::mjENBL_FWDINV            )
      .value("mjENBL_SENSORNOISE"     , mjtEnableBit             ::mjENBL_SENSORNOISE       )
      .value("mjENBL_MULTICCD"        , mjtEnableBit             ::mjENBL_MULTICCD          )
      .value("mjNENABLE"              , mjtEnableBit             ::mjNENABLE                )
  ;
  enum_<mjtJoint>("mjtJoint")
      .value("mjJNT_FREE"             , mjtJoint                 ::mjJNT_FREE               )
      .value("mjJNT_BALL"             , mjtJoint                 ::mjJNT_BALL               )
      .value("mjJNT_SLIDE"            , mjtJoint                 ::mjJNT_SLIDE              )
      .value("mjJNT_HINGE"            , mjtJoint                 ::mjJNT_HINGE              )
  ;
  enum_<mjtGeom>("mjtGeom")
      .value("mjGEOM_PLANE"           , mjtGeom                  ::mjGEOM_PLANE             )
      .value("mjGEOM_HFIELD"          , mjtGeom                  ::mjGEOM_HFIELD            )
      .value("mjGEOM_SPHERE"          , mjtGeom                  ::mjGEOM_SPHERE            )
      .value("mjGEOM_CAPSULE"         , mjtGeom                  ::mjGEOM_CAPSULE           )
      .value("mjGEOM_ELLIPSOID"       , mjtGeom                  ::mjGEOM_ELLIPSOID         )
      .value("mjGEOM_CYLINDER"        , mjtGeom                  ::mjGEOM_CYLINDER          )
      .value("mjGEOM_BOX"             , mjtGeom                  ::mjGEOM_BOX               )
      .value("mjGEOM_MESH"            , mjtGeom                  ::mjGEOM_MESH              )
      .value("mjNGEOMTYPES"           , mjtGeom                  ::mjNGEOMTYPES             )
      .value("mjGEOM_ARROW"           , mjtGeom                  ::mjGEOM_ARROW             )
      .value("mjGEOM_ARROW1"          , mjtGeom                  ::mjGEOM_ARROW1            )
      .value("mjGEOM_ARROW2"          , mjtGeom                  ::mjGEOM_ARROW2            )
      .value("mjGEOM_LINE"            , mjtGeom                  ::mjGEOM_LINE              )
      .value("mjGEOM_SKIN"            , mjtGeom                  ::mjGEOM_SKIN              )
      .value("mjGEOM_LABEL"           , mjtGeom                  ::mjGEOM_LABEL             )
      .value("mjGEOM_NONE"            , mjtGeom                  ::mjGEOM_NONE              )
  ;
  enum_<mjtCamLight>("mjtCamLight")
      .value("mjCAMLIGHT_FIXED"       , mjtCamLight              ::mjCAMLIGHT_FIXED         )
      .value("mjCAMLIGHT_TRACK"       , mjtCamLight              ::mjCAMLIGHT_TRACK         )
      .value("mjCAMLIGHT_TRACKCOM"    , mjtCamLight              ::mjCAMLIGHT_TRACKCOM      )
      .value("mjCAMLIGHT_TARGETBODY"  , mjtCamLight              ::mjCAMLIGHT_TARGETBODY    )
      .value("mjCAMLIGHT_TARGETBODYCOM", mjtCamLight              ::mjCAMLIGHT_TARGETBODYCOM )
  ;
  enum_<mjtTexture>("mjtTexture")
      .value("mjTEXTURE_2D"           , mjtTexture               ::mjTEXTURE_2D             )
      .value("mjTEXTURE_CUBE"         , mjtTexture               ::mjTEXTURE_CUBE           )
      .value("mjTEXTURE_SKYBOX"       , mjtTexture               ::mjTEXTURE_SKYBOX         )
  ;
  enum_<mjtIntegrator>("mjtIntegrator")
      .value("mjINT_EULER"            , mjtIntegrator            ::mjINT_EULER              )
      .value("mjINT_RK4"              , mjtIntegrator            ::mjINT_RK4                )
      .value("mjINT_IMPLICIT"         , mjtIntegrator            ::mjINT_IMPLICIT           )
  ;
  enum_<mjtCollision>("mjtCollision")
      .value("mjCOL_ALL"              , mjtCollision             ::mjCOL_ALL                )
      .value("mjCOL_PAIR"             , mjtCollision             ::mjCOL_PAIR               )
      .value("mjCOL_DYNAMIC"          , mjtCollision             ::mjCOL_DYNAMIC            )
  ;
  enum_<mjtCone>("mjtCone")
      .value("mjCONE_PYRAMIDAL"       , mjtCone                  ::mjCONE_PYRAMIDAL         )
      .value("mjCONE_ELLIPTIC"        , mjtCone                  ::mjCONE_ELLIPTIC          )
  ;
  enum_<mjtJacobian>("mjtJacobian")
      .value("mjJAC_DENSE"            , mjtJacobian              ::mjJAC_DENSE              )
      .value("mjJAC_SPARSE"           , mjtJacobian              ::mjJAC_SPARSE             )
      .value("mjJAC_AUTO"             , mjtJacobian              ::mjJAC_AUTO               )
  ;
  enum_<mjtSolver>("mjtSolver")
      .value("mjSOL_PGS"              , mjtSolver                ::mjSOL_PGS                )
      .value("mjSOL_CG"               , mjtSolver                ::mjSOL_CG                 )
      .value("mjSOL_NEWTON"           , mjtSolver                ::mjSOL_NEWTON             )
  ;
  enum_<mjtEq>("mjtEq")
      .value("mjEQ_CONNECT"           , mjtEq                    ::mjEQ_CONNECT             )
      .value("mjEQ_WELD"              , mjtEq                    ::mjEQ_WELD                )
      .value("mjEQ_JOINT"             , mjtEq                    ::mjEQ_JOINT               )
      .value("mjEQ_TENDON"            , mjtEq                    ::mjEQ_TENDON              )
      .value("mjEQ_DISTANCE"          , mjtEq                    ::mjEQ_DISTANCE            )
  ;
  enum_<mjtWrap>("mjtWrap")
      .value("mjWRAP_NONE"            , mjtWrap                  ::mjWRAP_NONE              )
      .value("mjWRAP_JOINT"           , mjtWrap                  ::mjWRAP_JOINT             )
      .value("mjWRAP_PULLEY"          , mjtWrap                  ::mjWRAP_PULLEY            )
      .value("mjWRAP_SITE"            , mjtWrap                  ::mjWRAP_SITE              )
      .value("mjWRAP_SPHERE"          , mjtWrap                  ::mjWRAP_SPHERE            )
      .value("mjWRAP_CYLINDER"        , mjtWrap                  ::mjWRAP_CYLINDER          )
  ;
  enum_<mjtTrn>("mjtTrn")
      .value("mjTRN_JOINT"            , mjtTrn                   ::mjTRN_JOINT              )
      .value("mjTRN_JOINTINPARENT"    , mjtTrn                   ::mjTRN_JOINTINPARENT      )
      .value("mjTRN_SLIDERCRANK"      , mjtTrn                   ::mjTRN_SLIDERCRANK        )
      .value("mjTRN_TENDON"           , mjtTrn                   ::mjTRN_TENDON             )
      .value("mjTRN_SITE"             , mjtTrn                   ::mjTRN_SITE               )
      .value("mjTRN_BODY"             , mjtTrn                   ::mjTRN_BODY               )
      .value("mjTRN_UNDEFINED"        , mjtTrn                   ::mjTRN_UNDEFINED          )
  ;
  enum_<mjtDyn>("mjtDyn")
      .value("mjDYN_NONE"             , mjtDyn                   ::mjDYN_NONE               )
      .value("mjDYN_INTEGRATOR"       , mjtDyn                   ::mjDYN_INTEGRATOR         )
      .value("mjDYN_FILTER"           , mjtDyn                   ::mjDYN_FILTER             )
      .value("mjDYN_MUSCLE"           , mjtDyn                   ::mjDYN_MUSCLE             )
      .value("mjDYN_USER"             , mjtDyn                   ::mjDYN_USER               )
  ;
  enum_<mjtGain>("mjtGain")
      .value("mjGAIN_FIXED"           , mjtGain                  ::mjGAIN_FIXED             )
      .value("mjGAIN_AFFINE"          , mjtGain                  ::mjGAIN_AFFINE            )
      .value("mjGAIN_MUSCLE"          , mjtGain                  ::mjGAIN_MUSCLE            )
      .value("mjGAIN_USER"            , mjtGain                  ::mjGAIN_USER              )
  ;
  enum_<mjtBias>("mjtBias")
      .value("mjBIAS_NONE"            , mjtBias                  ::mjBIAS_NONE              )
      .value("mjBIAS_AFFINE"          , mjtBias                  ::mjBIAS_AFFINE            )
      .value("mjBIAS_MUSCLE"          , mjtBias                  ::mjBIAS_MUSCLE            )
      .value("mjBIAS_USER"            , mjtBias                  ::mjBIAS_USER              )
  ;
  enum_<mjtObj>("mjtObj")
      .value("mjOBJ_UNKNOWN"          , mjtObj                   ::mjOBJ_UNKNOWN            )
      .value("mjOBJ_BODY"             , mjtObj                   ::mjOBJ_BODY               )
      .value("mjOBJ_XBODY"            , mjtObj                   ::mjOBJ_XBODY              )
      .value("mjOBJ_JOINT"            , mjtObj                   ::mjOBJ_JOINT              )
      .value("mjOBJ_DOF"              , mjtObj                   ::mjOBJ_DOF                )
      .value("mjOBJ_GEOM"             , mjtObj                   ::mjOBJ_GEOM               )
      .value("mjOBJ_SITE"             , mjtObj                   ::mjOBJ_SITE               )
      .value("mjOBJ_CAMERA"           , mjtObj                   ::mjOBJ_CAMERA             )
      .value("mjOBJ_LIGHT"            , mjtObj                   ::mjOBJ_LIGHT              )
      .value("mjOBJ_MESH"             , mjtObj                   ::mjOBJ_MESH               )
      .value("mjOBJ_SKIN"             , mjtObj                   ::mjOBJ_SKIN               )
      .value("mjOBJ_HFIELD"           , mjtObj                   ::mjOBJ_HFIELD             )
      .value("mjOBJ_TEXTURE"          , mjtObj                   ::mjOBJ_TEXTURE            )
      .value("mjOBJ_MATERIAL"         , mjtObj                   ::mjOBJ_MATERIAL           )
      .value("mjOBJ_PAIR"             , mjtObj                   ::mjOBJ_PAIR               )
      .value("mjOBJ_EXCLUDE"          , mjtObj                   ::mjOBJ_EXCLUDE            )
      .value("mjOBJ_EQUALITY"         , mjtObj                   ::mjOBJ_EQUALITY           )
      .value("mjOBJ_TENDON"           , mjtObj                   ::mjOBJ_TENDON             )
      .value("mjOBJ_ACTUATOR"         , mjtObj                   ::mjOBJ_ACTUATOR           )
      .value("mjOBJ_SENSOR"           , mjtObj                   ::mjOBJ_SENSOR             )
      .value("mjOBJ_NUMERIC"          , mjtObj                   ::mjOBJ_NUMERIC            )
      .value("mjOBJ_TEXT"             , mjtObj                   ::mjOBJ_TEXT               )
      .value("mjOBJ_TUPLE"            , mjtObj                   ::mjOBJ_TUPLE              )
      .value("mjOBJ_KEY"              , mjtObj                   ::mjOBJ_KEY                )
      .value("mjOBJ_PLUGIN"           , mjtObj                   ::mjOBJ_PLUGIN             )
  ;
  enum_<mjtConstraint>("mjtConstraint")
      .value("mjCNSTR_EQUALITY"       , mjtConstraint            ::mjCNSTR_EQUALITY         )
      .value("mjCNSTR_FRICTION_DOF"   , mjtConstraint            ::mjCNSTR_FRICTION_DOF     )
      .value("mjCNSTR_FRICTION_TENDON", mjtConstraint            ::mjCNSTR_FRICTION_TENDON  )
      .value("mjCNSTR_LIMIT_JOINT"    , mjtConstraint            ::mjCNSTR_LIMIT_JOINT      )
      .value("mjCNSTR_LIMIT_TENDON"   , mjtConstraint            ::mjCNSTR_LIMIT_TENDON     )
      .value("mjCNSTR_CONTACT_FRICTIONLESS", mjtConstraint            ::mjCNSTR_CONTACT_FRICTIONLESS)
      .value("mjCNSTR_CONTACT_PYRAMIDAL", mjtConstraint            ::mjCNSTR_CONTACT_PYRAMIDAL)
      .value("mjCNSTR_CONTACT_ELLIPTIC", mjtConstraint            ::mjCNSTR_CONTACT_ELLIPTIC )
  ;
  enum_<mjtConstraintState>("mjtConstraintState")
      .value("mjCNSTRSTATE_SATISFIED" , mjtConstraintState       ::mjCNSTRSTATE_SATISFIED   )
      .value("mjCNSTRSTATE_QUADRATIC" , mjtConstraintState       ::mjCNSTRSTATE_QUADRATIC   )
      .value("mjCNSTRSTATE_LINEARNEG" , mjtConstraintState       ::mjCNSTRSTATE_LINEARNEG   )
      .value("mjCNSTRSTATE_LINEARPOS" , mjtConstraintState       ::mjCNSTRSTATE_LINEARPOS   )
      .value("mjCNSTRSTATE_CONE"      , mjtConstraintState       ::mjCNSTRSTATE_CONE        )
  ;
  enum_<mjtSensor>("mjtSensor")
      .value("mjSENS_TOUCH"           , mjtSensor                ::mjSENS_TOUCH             )
      .value("mjSENS_ACCELEROMETER"   , mjtSensor                ::mjSENS_ACCELEROMETER     )
      .value("mjSENS_VELOCIMETER"     , mjtSensor                ::mjSENS_VELOCIMETER       )
      .value("mjSENS_GYRO"            , mjtSensor                ::mjSENS_GYRO              )
      .value("mjSENS_FORCE"           , mjtSensor                ::mjSENS_FORCE             )
      .value("mjSENS_TORQUE"          , mjtSensor                ::mjSENS_TORQUE            )
      .value("mjSENS_MAGNETOMETER"    , mjtSensor                ::mjSENS_MAGNETOMETER      )
      .value("mjSENS_RANGEFINDER"     , mjtSensor                ::mjSENS_RANGEFINDER       )
      .value("mjSENS_JOINTPOS"        , mjtSensor                ::mjSENS_JOINTPOS          )
      .value("mjSENS_JOINTVEL"        , mjtSensor                ::mjSENS_JOINTVEL          )
      .value("mjSENS_TENDONPOS"       , mjtSensor                ::mjSENS_TENDONPOS         )
      .value("mjSENS_TENDONVEL"       , mjtSensor                ::mjSENS_TENDONVEL         )
      .value("mjSENS_ACTUATORPOS"     , mjtSensor                ::mjSENS_ACTUATORPOS       )
      .value("mjSENS_ACTUATORVEL"     , mjtSensor                ::mjSENS_ACTUATORVEL       )
      .value("mjSENS_ACTUATORFRC"     , mjtSensor                ::mjSENS_ACTUATORFRC       )
      .value("mjSENS_BALLQUAT"        , mjtSensor                ::mjSENS_BALLQUAT          )
      .value("mjSENS_BALLANGVEL"      , mjtSensor                ::mjSENS_BALLANGVEL        )
      .value("mjSENS_JOINTLIMITPOS"   , mjtSensor                ::mjSENS_JOINTLIMITPOS     )
      .value("mjSENS_JOINTLIMITVEL"   , mjtSensor                ::mjSENS_JOINTLIMITVEL     )
      .value("mjSENS_JOINTLIMITFRC"   , mjtSensor                ::mjSENS_JOINTLIMITFRC     )
      .value("mjSENS_TENDONLIMITPOS"  , mjtSensor                ::mjSENS_TENDONLIMITPOS    )
      .value("mjSENS_TENDONLIMITVEL"  , mjtSensor                ::mjSENS_TENDONLIMITVEL    )
      .value("mjSENS_TENDONLIMITFRC"  , mjtSensor                ::mjSENS_TENDONLIMITFRC    )
      .value("mjSENS_FRAMEPOS"        , mjtSensor                ::mjSENS_FRAMEPOS          )
      .value("mjSENS_FRAMEQUAT"       , mjtSensor                ::mjSENS_FRAMEQUAT         )
      .value("mjSENS_FRAMEXAXIS"      , mjtSensor                ::mjSENS_FRAMEXAXIS        )
      .value("mjSENS_FRAMEYAXIS"      , mjtSensor                ::mjSENS_FRAMEYAXIS        )
      .value("mjSENS_FRAMEZAXIS"      , mjtSensor                ::mjSENS_FRAMEZAXIS        )
      .value("mjSENS_FRAMELINVEL"     , mjtSensor                ::mjSENS_FRAMELINVEL       )
      .value("mjSENS_FRAMEANGVEL"     , mjtSensor                ::mjSENS_FRAMEANGVEL       )
      .value("mjSENS_FRAMELINACC"     , mjtSensor                ::mjSENS_FRAMELINACC       )
      .value("mjSENS_FRAMEANGACC"     , mjtSensor                ::mjSENS_FRAMEANGACC       )
      .value("mjSENS_SUBTREECOM"      , mjtSensor                ::mjSENS_SUBTREECOM        )
      .value("mjSENS_SUBTREELINVEL"   , mjtSensor                ::mjSENS_SUBTREELINVEL     )
      .value("mjSENS_SUBTREEANGMOM"   , mjtSensor                ::mjSENS_SUBTREEANGMOM     )
      .value("mjSENS_CLOCK"           , mjtSensor                ::mjSENS_CLOCK             )
      .value("mjSENS_PLUGIN"          , mjtSensor                ::mjSENS_PLUGIN            )
      .value("mjSENS_USER"            , mjtSensor                ::mjSENS_USER              )
  ;
  enum_<mjtStage>("mjtStage")
      .value("mjSTAGE_NONE"           , mjtStage                 ::mjSTAGE_NONE             )
      .value("mjSTAGE_POS"            , mjtStage                 ::mjSTAGE_POS              )
      .value("mjSTAGE_VEL"            , mjtStage                 ::mjSTAGE_VEL              )
      .value("mjSTAGE_ACC"            , mjtStage                 ::mjSTAGE_ACC              )
  ;
  enum_<mjtDataType>("mjtDataType")
      .value("mjDATATYPE_REAL"        , mjtDataType              ::mjDATATYPE_REAL          )
      .value("mjDATATYPE_POSITIVE"    , mjtDataType              ::mjDATATYPE_POSITIVE      )
      .value("mjDATATYPE_AXIS"        , mjtDataType              ::mjDATATYPE_AXIS          )
      .value("mjDATATYPE_QUATERNION"  , mjtDataType              ::mjDATATYPE_QUATERNION    )
  ;
  enum_<mjtLRMode>("mjtLRMode")
      .value("mjLRMODE_NONE"          , mjtLRMode                ::mjLRMODE_NONE            )
      .value("mjLRMODE_MUSCLE"        , mjtLRMode                ::mjLRMODE_MUSCLE          )
      .value("mjLRMODE_MUSCLEUSER"    , mjtLRMode                ::mjLRMODE_MUSCLEUSER      )
      .value("mjLRMODE_ALL"           , mjtLRMode                ::mjLRMODE_ALL             )
  ;


  class_<Model>("Model")
      .constructor<>(&Model::load_from_xml)
      .class_function("load_from_xml", &Model::load_from_xml)
      .class_function("load_from_mjb", &Model::load_from_mjb)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("free"            , &Model::free        )
      .function("getVal"          , &Model::getVal      )
      .function("getOptions"      , &Model::getOptions  )
      // MJMODEL_BINDINGS
      .property("nq"                    , &Model::nq                    )
      .property("nv"                    , &Model::nv                    )
      .property("nu"                    , &Model::nu                    )
      .property("na"                    , &Model::na                    )
      .property("nbody"                 , &Model::nbody                 )
      .property("njnt"                  , &Model::njnt                  )
      .property("ngeom"                 , &Model::ngeom                 )
      .property("nsite"                 , &Model::nsite                 )
      .property("ncam"                  , &Model::ncam                  )
      .property("nlight"                , &Model::nlight                )
      .property("nmesh"                 , &Model::nmesh                 )
      .property("nmeshvert"             , &Model::nmeshvert             )
      .property("nmeshtexvert"          , &Model::nmeshtexvert          )
      .property("nmeshface"             , &Model::nmeshface             )
      .property("nmeshgraph"            , &Model::nmeshgraph            )
      .property("nskin"                 , &Model::nskin                 )
      .property("nskinvert"             , &Model::nskinvert             )
      .property("nskintexvert"          , &Model::nskintexvert          )
      .property("nskinface"             , &Model::nskinface             )
      .property("nskinbone"             , &Model::nskinbone             )
      .property("nskinbonevert"         , &Model::nskinbonevert         )
      .property("nhfield"               , &Model::nhfield               )
      .property("nhfielddata"           , &Model::nhfielddata           )
      .property("ntex"                  , &Model::ntex                  )
      .property("ntexdata"              , &Model::ntexdata              )
      .property("nmat"                  , &Model::nmat                  )
      .property("npair"                 , &Model::npair                 )
      .property("nexclude"              , &Model::nexclude              )
      .property("neq"                   , &Model::neq                   )
      .property("ntendon"               , &Model::ntendon               )
      .property("nwrap"                 , &Model::nwrap                 )
      .property("nsensor"               , &Model::nsensor               )
      .property("nnumeric"              , &Model::nnumeric              )
      .property("nnumericdata"          , &Model::nnumericdata          )
      .property("ntext"                 , &Model::ntext                 )
      .property("ntextdata"             , &Model::ntextdata             )
      .property("ntuple"                , &Model::ntuple                )
      .property("ntupledata"            , &Model::ntupledata            )
      .property("nkey"                  , &Model::nkey                  )
      .property("nmocap"                , &Model::nmocap                )
      .property("nplugin"               , &Model::nplugin               )
      .property("npluginattr"           , &Model::npluginattr           )
      .property("nuser_body"            , &Model::nuser_body            )
      .property("nuser_jnt"             , &Model::nuser_jnt             )
      .property("nuser_geom"            , &Model::nuser_geom            )
      .property("nuser_site"            , &Model::nuser_site            )
      .property("nuser_cam"             , &Model::nuser_cam             )
      .property("nuser_tendon"          , &Model::nuser_tendon          )
      .property("nuser_actuator"        , &Model::nuser_actuator        )
      .property("nuser_sensor"          , &Model::nuser_sensor          )
      .property("nnames"                , &Model::nnames                )
      .property("nM"                    , &Model::nM                    )
      .property("nD"                    , &Model::nD                    )
      .property("nemax"                 , &Model::nemax                 )
      .property("njmax"                 , &Model::njmax                 )
      .property("nconmax"               , &Model::nconmax               )
      .property("nstack"                , &Model::nstack                )
      .property("nuserdata"             , &Model::nuserdata             )
      .property("nsensordata"           , &Model::nsensordata           )
      .property("npluginstate"          , &Model::npluginstate          )
      .property("nbuffer"               , &Model::nbuffer               )
      .property("qpos0"                 , &Model::qpos0                 )
      .property("qpos_spring"           , &Model::qpos_spring           )
      .property("body_parentid"         , &Model::body_parentid         )
      .property("body_rootid"           , &Model::body_rootid           )
      .property("body_weldid"           , &Model::body_weldid           )
      .property("body_mocapid"          , &Model::body_mocapid          )
      .property("body_jntnum"           , &Model::body_jntnum           )
      .property("body_jntadr"           , &Model::body_jntadr           )
      .property("body_dofnum"           , &Model::body_dofnum           )
      .property("body_dofadr"           , &Model::body_dofadr           )
      .property("body_geomnum"          , &Model::body_geomnum          )
      .property("body_geomadr"          , &Model::body_geomadr          )
      .property("body_simple"           , &Model::body_simple           )
      .property("body_sameframe"        , &Model::body_sameframe        )
      .property("body_pos"              , &Model::body_pos              )
      .property("body_quat"             , &Model::body_quat             )
      .property("body_ipos"             , &Model::body_ipos             )
      .property("body_iquat"            , &Model::body_iquat            )
      .property("body_mass"             , &Model::body_mass             )
      .property("body_subtreemass"      , &Model::body_subtreemass      )
      .property("body_inertia"          , &Model::body_inertia          )
      .property("body_invweight0"       , &Model::body_invweight0       )
      .property("body_gravcomp"         , &Model::body_gravcomp         )
      .property("body_user"             , &Model::body_user             )
      .property("body_plugin"           , &Model::body_plugin           )
      .property("jnt_type"              , &Model::jnt_type              )
      .property("jnt_qposadr"           , &Model::jnt_qposadr           )
      .property("jnt_dofadr"            , &Model::jnt_dofadr            )
      .property("jnt_bodyid"            , &Model::jnt_bodyid            )
      .property("jnt_group"             , &Model::jnt_group             )
      .property("jnt_limited"           , &Model::jnt_limited           )
      .property("jnt_solref"            , &Model::jnt_solref            )
      .property("jnt_solimp"            , &Model::jnt_solimp            )
      .property("jnt_pos"               , &Model::jnt_pos               )
      .property("jnt_axis"              , &Model::jnt_axis              )
      .property("jnt_stiffness"         , &Model::jnt_stiffness         )
      .property("jnt_range"             , &Model::jnt_range             )
      .property("jnt_margin"            , &Model::jnt_margin            )
      .property("jnt_user"              , &Model::jnt_user              )
      .property("dof_bodyid"            , &Model::dof_bodyid            )
      .property("dof_jntid"             , &Model::dof_jntid             )
      .property("dof_parentid"          , &Model::dof_parentid          )
      .property("dof_Madr"              , &Model::dof_Madr              )
      .property("dof_simplenum"         , &Model::dof_simplenum         )
      .property("dof_solref"            , &Model::dof_solref            )
      .property("dof_solimp"            , &Model::dof_solimp            )
      .property("dof_frictionloss"      , &Model::dof_frictionloss      )
      .property("dof_armature"          , &Model::dof_armature          )
      .property("dof_damping"           , &Model::dof_damping           )
      .property("dof_invweight0"        , &Model::dof_invweight0        )
      .property("dof_M0"                , &Model::dof_M0                )
      .property("geom_type"             , &Model::geom_type             )
      .property("geom_contype"          , &Model::geom_contype          )
      .property("geom_conaffinity"      , &Model::geom_conaffinity      )
      .property("geom_condim"           , &Model::geom_condim           )
      .property("geom_bodyid"           , &Model::geom_bodyid           )
      .property("geom_dataid"           , &Model::geom_dataid           )
      .property("geom_matid"            , &Model::geom_matid            )
      .property("geom_group"            , &Model::geom_group            )
      .property("geom_priority"         , &Model::geom_priority         )
      .property("geom_sameframe"        , &Model::geom_sameframe        )
      .property("geom_solmix"           , &Model::geom_solmix           )
      .property("geom_solref"           , &Model::geom_solref           )
      .property("geom_solimp"           , &Model::geom_solimp           )
      .property("geom_size"             , &Model::geom_size             )
      .property("geom_rbound"           , &Model::geom_rbound           )
      .property("geom_pos"              , &Model::geom_pos              )
      .property("geom_quat"             , &Model::geom_quat             )
      .property("geom_friction"         , &Model::geom_friction         )
      .property("geom_margin"           , &Model::geom_margin           )
      .property("geom_gap"              , &Model::geom_gap              )
      .property("geom_fluid"            , &Model::geom_fluid            )
      .property("geom_user"             , &Model::geom_user             )
      .property("geom_rgba"             , &Model::geom_rgba             )
      .property("site_type"             , &Model::site_type             )
      .property("site_bodyid"           , &Model::site_bodyid           )
      .property("site_matid"            , &Model::site_matid            )
      .property("site_group"            , &Model::site_group            )
      .property("site_sameframe"        , &Model::site_sameframe        )
      .property("site_size"             , &Model::site_size             )
      .property("site_pos"              , &Model::site_pos              )
      .property("site_quat"             , &Model::site_quat             )
      .property("site_user"             , &Model::site_user             )
      .property("site_rgba"             , &Model::site_rgba             )
      .property("cam_mode"              , &Model::cam_mode              )
      .property("cam_bodyid"            , &Model::cam_bodyid            )
      .property("cam_targetbodyid"      , &Model::cam_targetbodyid      )
      .property("cam_pos"               , &Model::cam_pos               )
      .property("cam_quat"              , &Model::cam_quat              )
      .property("cam_poscom0"           , &Model::cam_poscom0           )
      .property("cam_pos0"              , &Model::cam_pos0              )
      .property("cam_mat0"              , &Model::cam_mat0              )
      .property("cam_fovy"              , &Model::cam_fovy              )
      .property("cam_ipd"               , &Model::cam_ipd               )
      .property("cam_user"              , &Model::cam_user              )
      .property("light_mode"            , &Model::light_mode            )
      .property("light_bodyid"          , &Model::light_bodyid          )
      .property("light_targetbodyid"    , &Model::light_targetbodyid    )
      .property("light_directional"     , &Model::light_directional     )
      .property("light_castshadow"      , &Model::light_castshadow      )
      .property("light_active"          , &Model::light_active          )
      .property("light_pos"             , &Model::light_pos             )
      .property("light_dir"             , &Model::light_dir             )
      .property("light_poscom0"         , &Model::light_poscom0         )
      .property("light_pos0"            , &Model::light_pos0            )
      .property("light_dir0"            , &Model::light_dir0            )
      .property("light_attenuation"     , &Model::light_attenuation     )
      .property("light_cutoff"          , &Model::light_cutoff          )
      .property("light_exponent"        , &Model::light_exponent        )
      .property("light_ambient"         , &Model::light_ambient         )
      .property("light_diffuse"         , &Model::light_diffuse         )
      .property("light_specular"        , &Model::light_specular        )
      .property("mesh_vertadr"          , &Model::mesh_vertadr          )
      .property("mesh_vertnum"          , &Model::mesh_vertnum          )
      .property("mesh_texcoordadr"      , &Model::mesh_texcoordadr      )
      .property("mesh_faceadr"          , &Model::mesh_faceadr          )
      .property("mesh_facenum"          , &Model::mesh_facenum          )
      .property("mesh_graphadr"         , &Model::mesh_graphadr         )
      .property("mesh_vert"             , &Model::mesh_vert             )
      .property("mesh_normal"           , &Model::mesh_normal           )
      .property("mesh_texcoord"         , &Model::mesh_texcoord         )
      .property("mesh_face"             , &Model::mesh_face             )
      .property("mesh_graph"            , &Model::mesh_graph            )
      .property("skin_matid"            , &Model::skin_matid            )
      .property("skin_group"            , &Model::skin_group            )
      .property("skin_rgba"             , &Model::skin_rgba             )
      .property("skin_inflate"          , &Model::skin_inflate          )
      .property("skin_vertadr"          , &Model::skin_vertadr          )
      .property("skin_vertnum"          , &Model::skin_vertnum          )
      .property("skin_texcoordadr"      , &Model::skin_texcoordadr      )
      .property("skin_faceadr"          , &Model::skin_faceadr          )
      .property("skin_facenum"          , &Model::skin_facenum          )
      .property("skin_boneadr"          , &Model::skin_boneadr          )
      .property("skin_bonenum"          , &Model::skin_bonenum          )
      .property("skin_vert"             , &Model::skin_vert             )
      .property("skin_texcoord"         , &Model::skin_texcoord         )
      .property("skin_face"             , &Model::skin_face             )
      .property("skin_bonevertadr"      , &Model::skin_bonevertadr      )
      .property("skin_bonevertnum"      , &Model::skin_bonevertnum      )
      .property("skin_bonebindpos"      , &Model::skin_bonebindpos      )
      .property("skin_bonebindquat"     , &Model::skin_bonebindquat     )
      .property("skin_bonebodyid"       , &Model::skin_bonebodyid       )
      .property("skin_bonevertid"       , &Model::skin_bonevertid       )
      .property("skin_bonevertweight"   , &Model::skin_bonevertweight   )
      .property("hfield_size"           , &Model::hfield_size           )
      .property("hfield_nrow"           , &Model::hfield_nrow           )
      .property("hfield_ncol"           , &Model::hfield_ncol           )
      .property("hfield_adr"            , &Model::hfield_adr            )
      .property("hfield_data"           , &Model::hfield_data           )
      .property("tex_type"              , &Model::tex_type              )
      .property("tex_height"            , &Model::tex_height            )
      .property("tex_width"             , &Model::tex_width             )
      .property("tex_adr"               , &Model::tex_adr               )
      .property("tex_rgb"               , &Model::tex_rgb               )
      .property("mat_texid"             , &Model::mat_texid             )
      .property("mat_texuniform"        , &Model::mat_texuniform        )
      .property("mat_texrepeat"         , &Model::mat_texrepeat         )
      .property("mat_emission"          , &Model::mat_emission          )
      .property("mat_specular"          , &Model::mat_specular          )
      .property("mat_shininess"         , &Model::mat_shininess         )
      .property("mat_reflectance"       , &Model::mat_reflectance       )
      .property("mat_rgba"              , &Model::mat_rgba              )
      .property("pair_dim"              , &Model::pair_dim              )
      .property("pair_geom1"            , &Model::pair_geom1            )
      .property("pair_geom2"            , &Model::pair_geom2            )
      .property("pair_signature"        , &Model::pair_signature        )
      .property("pair_solref"           , &Model::pair_solref           )
      .property("pair_solimp"           , &Model::pair_solimp           )
      .property("pair_margin"           , &Model::pair_margin           )
      .property("pair_gap"              , &Model::pair_gap              )
      .property("pair_friction"         , &Model::pair_friction         )
      .property("exclude_signature"     , &Model::exclude_signature     )
      .property("eq_type"               , &Model::eq_type               )
      .property("eq_obj1id"             , &Model::eq_obj1id             )
      .property("eq_obj2id"             , &Model::eq_obj2id             )
      .property("eq_active"             , &Model::eq_active             )
      .property("eq_solref"             , &Model::eq_solref             )
      .property("eq_solimp"             , &Model::eq_solimp             )
      .property("eq_data"               , &Model::eq_data               )
      .property("tendon_adr"            , &Model::tendon_adr            )
      .property("tendon_num"            , &Model::tendon_num            )
      .property("tendon_matid"          , &Model::tendon_matid          )
      .property("tendon_group"          , &Model::tendon_group          )
      .property("tendon_limited"        , &Model::tendon_limited        )
      .property("tendon_width"          , &Model::tendon_width          )
      .property("tendon_solref_lim"     , &Model::tendon_solref_lim     )
      .property("tendon_solimp_lim"     , &Model::tendon_solimp_lim     )
      .property("tendon_solref_fri"     , &Model::tendon_solref_fri     )
      .property("tendon_solimp_fri"     , &Model::tendon_solimp_fri     )
      .property("tendon_range"          , &Model::tendon_range          )
      .property("tendon_margin"         , &Model::tendon_margin         )
      .property("tendon_stiffness"      , &Model::tendon_stiffness      )
      .property("tendon_damping"        , &Model::tendon_damping        )
      .property("tendon_frictionloss"   , &Model::tendon_frictionloss   )
      .property("tendon_lengthspring"   , &Model::tendon_lengthspring   )
      .property("tendon_length0"        , &Model::tendon_length0        )
      .property("tendon_invweight0"     , &Model::tendon_invweight0     )
      .property("tendon_user"           , &Model::tendon_user           )
      .property("tendon_rgba"           , &Model::tendon_rgba           )
      .property("wrap_type"             , &Model::wrap_type             )
      .property("wrap_objid"            , &Model::wrap_objid            )
      .property("wrap_prm"              , &Model::wrap_prm              )
      .property("actuator_trntype"      , &Model::actuator_trntype      )
      .property("actuator_dyntype"      , &Model::actuator_dyntype      )
      .property("actuator_gaintype"     , &Model::actuator_gaintype     )
      .property("actuator_biastype"     , &Model::actuator_biastype     )
      .property("actuator_trnid"        , &Model::actuator_trnid        )
      .property("actuator_actadr"       , &Model::actuator_actadr       )
      .property("actuator_actnum"       , &Model::actuator_actnum       )
      .property("actuator_group"        , &Model::actuator_group        )
      .property("actuator_ctrllimited"  , &Model::actuator_ctrllimited  )
      .property("actuator_forcelimited" , &Model::actuator_forcelimited )
      .property("actuator_actlimited"   , &Model::actuator_actlimited   )
      .property("actuator_dynprm"       , &Model::actuator_dynprm       )
      .property("actuator_gainprm"      , &Model::actuator_gainprm      )
      .property("actuator_biasprm"      , &Model::actuator_biasprm      )
      .property("actuator_ctrlrange"    , &Model::actuator_ctrlrange    )
      .property("actuator_forcerange"   , &Model::actuator_forcerange   )
      .property("actuator_actrange"     , &Model::actuator_actrange     )
      .property("actuator_gear"         , &Model::actuator_gear         )
      .property("actuator_cranklength"  , &Model::actuator_cranklength  )
      .property("actuator_acc0"         , &Model::actuator_acc0         )
      .property("actuator_length0"      , &Model::actuator_length0      )
      .property("actuator_lengthrange"  , &Model::actuator_lengthrange  )
      .property("actuator_user"         , &Model::actuator_user         )
      .property("actuator_plugin"       , &Model::actuator_plugin       )
      .property("sensor_type"           , &Model::sensor_type           )
      .property("sensor_datatype"       , &Model::sensor_datatype       )
      .property("sensor_needstage"      , &Model::sensor_needstage      )
      .property("sensor_objtype"        , &Model::sensor_objtype        )
      .property("sensor_objid"          , &Model::sensor_objid          )
      .property("sensor_reftype"        , &Model::sensor_reftype        )
      .property("sensor_refid"          , &Model::sensor_refid          )
      .property("sensor_dim"            , &Model::sensor_dim            )
      .property("sensor_adr"            , &Model::sensor_adr            )
      .property("sensor_cutoff"         , &Model::sensor_cutoff         )
      .property("sensor_noise"          , &Model::sensor_noise          )
      .property("sensor_user"           , &Model::sensor_user           )
      .property("sensor_plugin"         , &Model::sensor_plugin         )
      .property("plugin"                , &Model::plugin                )
      .property("plugin_stateadr"       , &Model::plugin_stateadr       )
      .property("plugin_statenum"       , &Model::plugin_statenum       )
      .property("plugin_attr"           , &Model::plugin_attr           )
      .property("plugin_attradr"        , &Model::plugin_attradr        )
      .property("numeric_adr"           , &Model::numeric_adr           )
      .property("numeric_size"          , &Model::numeric_size          )
      .property("numeric_data"          , &Model::numeric_data          )
      .property("text_adr"              , &Model::text_adr              )
      .property("text_size"             , &Model::text_size             )
      .property("text_data"             , &Model::text_data             )
      .property("tuple_adr"             , &Model::tuple_adr             )
      .property("tuple_size"            , &Model::tuple_size            )
      .property("tuple_objtype"         , &Model::tuple_objtype         )
      .property("tuple_objid"           , &Model::tuple_objid           )
      .property("tuple_objprm"          , &Model::tuple_objprm          )
      .property("key_time"              , &Model::key_time              )
      .property("key_qpos"              , &Model::key_qpos              )
      .property("key_qvel"              , &Model::key_qvel              )
      .property("key_act"               , &Model::key_act               )
      .property("key_mpos"              , &Model::key_mpos              )
      .property("key_mquat"             , &Model::key_mquat             )
      .property("key_ctrl"              , &Model::key_ctrl              )
      .property("name_bodyadr"          , &Model::name_bodyadr          )
      .property("name_jntadr"           , &Model::name_jntadr           )
      .property("name_geomadr"          , &Model::name_geomadr          )
      .property("name_siteadr"          , &Model::name_siteadr          )
      .property("name_camadr"           , &Model::name_camadr           )
      .property("name_lightadr"         , &Model::name_lightadr         )
      .property("name_meshadr"          , &Model::name_meshadr          )
      .property("name_skinadr"          , &Model::name_skinadr          )
      .property("name_hfieldadr"        , &Model::name_hfieldadr        )
      .property("name_texadr"           , &Model::name_texadr           )
      .property("name_matadr"           , &Model::name_matadr           )
      .property("name_pairadr"          , &Model::name_pairadr          )
      .property("name_excludeadr"       , &Model::name_excludeadr       )
      .property("name_eqadr"            , &Model::name_eqadr            )
      .property("name_tendonadr"        , &Model::name_tendonadr        )
      .property("name_actuatoradr"      , &Model::name_actuatoradr      )
      .property("name_sensoradr"        , &Model::name_sensoradr        )
      .property("name_numericadr"       , &Model::name_numericadr       )
      .property("name_textadr"          , &Model::name_textadr          )
      .property("name_tupleadr"         , &Model::name_tupleadr         )
      .property("name_keyadr"           , &Model::name_keyadr           )
      .property("name_pluginadr"        , &Model::name_pluginadr        )
      .property("names"                 , &Model::names                 )
;

  class_<State>("State")
      .constructor<Model>()
      .function("ptr"   , &State::ptr, allow_raw_pointers())
      .function("free"  , &State::free  )
      .function("getVal", &State::getVal);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("state"     , &Simulation::state, allow_raw_pointers())
      .function("model"     , &Simulation::model, allow_raw_pointers())
      .function("free"      , &Simulation::free      )
      .function("applyForce", &Simulation::applyForce)
      .function("applyPose" , &Simulation::applyPose )
      // MJDATA_BINDINGS
      .property("qpos"                  , &Simulation::qpos                  )
      .property("qvel"                  , &Simulation::qvel                  )
      .property("act"                   , &Simulation::act                   )
      .property("qacc_warmstart"        , &Simulation::qacc_warmstart        )
      .property("plugin_state"          , &Simulation::plugin_state          )
      .property("ctrl"                  , &Simulation::ctrl                  )
      .property("qfrc_applied"          , &Simulation::qfrc_applied          )
      .property("xfrc_applied"          , &Simulation::xfrc_applied          )
      .property("mocap_pos"             , &Simulation::mocap_pos             )
      .property("mocap_quat"            , &Simulation::mocap_quat            )
      .property("qacc"                  , &Simulation::qacc                  )
      .property("act_dot"               , &Simulation::act_dot               )
      .property("userdata"              , &Simulation::userdata              )
      .property("sensordata"            , &Simulation::sensordata            )
      .property("plugin"                , &Simulation::plugin                )
      .property("plugin_data"           , &Simulation::plugin_data           )
      .property("xpos"                  , &Simulation::xpos                  )
      .property("xquat"                 , &Simulation::xquat                 )
      .property("xmat"                  , &Simulation::xmat                  )
      .property("xipos"                 , &Simulation::xipos                 )
      .property("ximat"                 , &Simulation::ximat                 )
      .property("xanchor"               , &Simulation::xanchor               )
      .property("xaxis"                 , &Simulation::xaxis                 )
      .property("geom_xpos"             , &Simulation::geom_xpos             )
      .property("geom_xmat"             , &Simulation::geom_xmat             )
      .property("site_xpos"             , &Simulation::site_xpos             )
      .property("site_xmat"             , &Simulation::site_xmat             )
      .property("cam_xpos"              , &Simulation::cam_xpos              )
      .property("cam_xmat"              , &Simulation::cam_xmat              )
      .property("light_xpos"            , &Simulation::light_xpos            )
      .property("light_xdir"            , &Simulation::light_xdir            )
      .property("subtree_com"           , &Simulation::subtree_com           )
      .property("cdof"                  , &Simulation::cdof                  )
      .property("cinert"                , &Simulation::cinert                )
      .property("ten_wrapadr"           , &Simulation::ten_wrapadr           )
      .property("ten_wrapnum"           , &Simulation::ten_wrapnum           )
      .property("ten_J_rownnz"          , &Simulation::ten_J_rownnz          )
      .property("ten_J_rowadr"          , &Simulation::ten_J_rowadr          )
      .property("ten_J_colind"          , &Simulation::ten_J_colind          )
      .property("ten_length"            , &Simulation::ten_length            )
      .property("ten_J"                 , &Simulation::ten_J                 )
      .property("wrap_obj"              , &Simulation::wrap_obj              )
      .property("wrap_xpos"             , &Simulation::wrap_xpos             )
      .property("actuator_length"       , &Simulation::actuator_length       )
      .property("actuator_moment"       , &Simulation::actuator_moment       )
      .property("crb"                   , &Simulation::crb                   )
      .property("qM"                    , &Simulation::qM                    )
      .property("qLD"                   , &Simulation::qLD                   )
      .property("qLDiagInv"             , &Simulation::qLDiagInv             )
      .property("qLDiagSqrtInv"         , &Simulation::qLDiagSqrtInv         )
      .property("ten_velocity"          , &Simulation::ten_velocity          )
      .property("actuator_velocity"     , &Simulation::actuator_velocity     )
      .property("cvel"                  , &Simulation::cvel                  )
      .property("cdof_dot"              , &Simulation::cdof_dot              )
      .property("qfrc_bias"             , &Simulation::qfrc_bias             )
      .property("qfrc_passive"          , &Simulation::qfrc_passive          )
      .property("subtree_linvel"        , &Simulation::subtree_linvel        )
      .property("subtree_angmom"        , &Simulation::subtree_angmom        )
      .property("qH"                    , &Simulation::qH                    )
      .property("qHDiagInv"             , &Simulation::qHDiagInv             )
      .property("D_rownnz"              , &Simulation::D_rownnz              )
      .property("D_rowadr"              , &Simulation::D_rowadr              )
      .property("D_colind"              , &Simulation::D_colind              )
      .property("qDeriv"                , &Simulation::qDeriv                )
      .property("qLU"                   , &Simulation::qLU                   )
      .property("actuator_force"        , &Simulation::actuator_force        )
      .property("qfrc_actuator"         , &Simulation::qfrc_actuator         )
      .property("qfrc_smooth"           , &Simulation::qfrc_smooth           )
      .property("qacc_smooth"           , &Simulation::qacc_smooth           )
      .property("qfrc_constraint"       , &Simulation::qfrc_constraint       )
      .property("qfrc_inverse"          , &Simulation::qfrc_inverse          )
      .property("cacc"                  , &Simulation::cacc                  )
      .property("cfrc_int"              , &Simulation::cfrc_int              )
      .property("cfrc_ext"              , &Simulation::cfrc_ext              )
      .function("freeLastXML"           , &Simulation::freeLastXML           )
      .function("step"                  , &Simulation::step                  )
      .function("step1"                 , &Simulation::step1                 )
      .function("step2"                 , &Simulation::step2                 )
      .function("forward"               , &Simulation::forward               )
      .function("inverse"               , &Simulation::inverse               )
      .function("forwardSkip"           , &Simulation::forwardSkip           )
      .function("inverseSkip"           , &Simulation::inverseSkip           )
      .function("defaultSolRefImp"      , &Simulation::defaultSolRefImp      , allow_raw_pointers())
      .function("sizeModel"             , &Simulation::sizeModel             )
      .function("resetData"             , &Simulation::resetData             )
      .function("resetDataDebug"        , &Simulation::resetDataDebug        )
      .function("resetDataKeyframe"     , &Simulation::resetDataKeyframe     )
      .function("deleteData"            , &Simulation::deleteData            )
      .function("resetCallbacks"        , &Simulation::resetCallbacks        )
      .function("printFormattedModel"   , &Simulation::printFormattedModel   )
      .function("printModel"            , &Simulation::printModel            )
      .function("printFormattedData"    , &Simulation::printFormattedData    )
      .function("printData"             , &Simulation::printData             )
      .function("_printMat"             , &Simulation::_printMat             , allow_raw_pointers())
      .function("fwdPosition"           , &Simulation::fwdPosition           )
      .function("fwdVelocity"           , &Simulation::fwdVelocity           )
      .function("fwdActuation"          , &Simulation::fwdActuation          )
      .function("fwdAcceleration"       , &Simulation::fwdAcceleration       )
      .function("fwdConstraint"         , &Simulation::fwdConstraint         )
      .function("Euler"                 , &Simulation::Euler                 )
      .function("RungeKutta"            , &Simulation::RungeKutta            )
      .function("invPosition"           , &Simulation::invPosition           )
      .function("invVelocity"           , &Simulation::invVelocity           )
      .function("invConstraint"         , &Simulation::invConstraint         )
      .function("compareFwdInv"         , &Simulation::compareFwdInv         )
      .function("sensorPos"             , &Simulation::sensorPos             )
      .function("sensorVel"             , &Simulation::sensorVel             )
      .function("sensorAcc"             , &Simulation::sensorAcc             )
      .function("energyPos"             , &Simulation::energyPos             )
      .function("energyVel"             , &Simulation::energyVel             )
      .function("checkPos"              , &Simulation::checkPos              )
      .function("checkVel"              , &Simulation::checkVel              )
      .function("checkAcc"              , &Simulation::checkAcc              )
      .function("kinematics"            , &Simulation::kinematics            )
      .function("comPos"                , &Simulation::comPos                )
      .function("camlight"              , &Simulation::camlight              )
      .function("tendon"                , &Simulation::tendon                )
      .function("transmission"          , &Simulation::transmission          )
      .function("crbCalculate"          , &Simulation::crbCalculate          )
      .function("factorM"               , &Simulation::factorM               )
      .function("solveM"                , &Simulation::solveM                , allow_raw_pointers())
      .function("solveM2"               , &Simulation::solveM2               , allow_raw_pointers())
      .function("comVel"                , &Simulation::comVel                )
      .function("passive"               , &Simulation::passive               )
      .function("subtreeVel"            , &Simulation::subtreeVel            )
      .function("rne"                   , &Simulation::rne                   , allow_raw_pointers())
      .function("rnePostConstraint"     , &Simulation::rnePostConstraint     )
      .function("collision"             , &Simulation::collision             )
      .function("makeConstraint"        , &Simulation::makeConstraint        )
      .function("projectConstraint"     , &Simulation::projectConstraint     )
      .function("referenceConstraint"   , &Simulation::referenceConstraint   )
      .function("isPyramidal"           , &Simulation::isPyramidal           )
      .function("isSparse"              , &Simulation::isSparse              )
      .function("isDual"                , &Simulation::isDual                )
      .function("mulJacVec"             , &Simulation::mulJacVec             , allow_raw_pointers())
      .function("mulJacTVec"            , &Simulation::mulJacTVec            , allow_raw_pointers())
      .function("jacSubtreeCom"         , &Simulation::jacSubtreeCom         , allow_raw_pointers())
      .function("name2id"               , &Simulation::name2id               )
      .function("id2name"               , &Simulation::id2name               )
      .function("fullM"                 , &Simulation::fullM                 , allow_raw_pointers())
      .function("differentiatePos"      , &Simulation::differentiatePos      , allow_raw_pointers())
      .function("integratePos"          , &Simulation::integratePos          , allow_raw_pointers())
      .function("normalizeQuat"         , &Simulation::normalizeQuat         , allow_raw_pointers())
      .function("getTotalmass"          , &Simulation::getTotalmass          )
      .function("getPluginConfig"       , &Simulation::getPluginConfig       )
      .function("loadPluginLibrary"     , &Simulation::loadPluginLibrary     )
      .function("version"               , &Simulation::version               )
      .function("versionString"         , &Simulation::versionString         )
      .function("_rectangle"            , &Simulation::_rectangle            )
      .function("_finish"               , &Simulation::_finish               )
      .function("_getError"             , &Simulation::_getError             )
      .function("i_themeSpacing"        , &Simulation::i_themeSpacing        )
      .function("i_themeColor"          , &Simulation::i_themeColor          )
      .function("_error"                , &Simulation::_error                )
      .function("_error_i"              , &Simulation::_error_i              )
      .function("_error_s"              , &Simulation::_error_s              )
      .function("_warning"              , &Simulation::_warning              )
      .function("_warning_i"            , &Simulation::_warning_i            )
      .function("_warning_s"            , &Simulation::_warning_s            )
      .function("_clearHandlers"        , &Simulation::_clearHandlers        )
      .function("warning"               , &Simulation::warning               )
      .function("_writeLog"             , &Simulation::_writeLog             )
      .function("activate"              , &Simulation::activate              )
      .function("deactivate"            , &Simulation::deactivate            )
      .function("_zero"                 , &Simulation::_zero                 , allow_raw_pointers())
      .function("_fill"                 , &Simulation::_fill                 , allow_raw_pointers())
      .function("_copy"                 , &Simulation::_copy                 , allow_raw_pointers())
      .function("_sum"                  , &Simulation::_sum                  , allow_raw_pointers())
      .function("_L1"                   , &Simulation::_L1                   , allow_raw_pointers())
      .function("_scl"                  , &Simulation::_scl                  , allow_raw_pointers())
      .function("_add"                  , &Simulation::_add                  , allow_raw_pointers())
      .function("_sub"                  , &Simulation::_sub                  , allow_raw_pointers())
      .function("_addTo"                , &Simulation::_addTo                , allow_raw_pointers())
      .function("_subFrom"              , &Simulation::_subFrom              , allow_raw_pointers())
      .function("_addToScl"             , &Simulation::_addToScl             , allow_raw_pointers())
      .function("_addScl"               , &Simulation::_addScl               , allow_raw_pointers())
      .function("_normalize"            , &Simulation::_normalize            , allow_raw_pointers())
      .function("_norm"                 , &Simulation::_norm                 , allow_raw_pointers())
      .function("_dot"                  , &Simulation::_dot                  , allow_raw_pointers())
      .function("_mulMatVec"            , &Simulation::_mulMatVec            , allow_raw_pointers())
      .function("_mulMatTVec"           , &Simulation::_mulMatTVec           , allow_raw_pointers())
      .function("_mulVecMatVec"         , &Simulation::_mulVecMatVec         , allow_raw_pointers())
      .function("_transpose"            , &Simulation::_transpose            , allow_raw_pointers())
      .function("_symmetrize"           , &Simulation::_symmetrize           , allow_raw_pointers())
      .function("_eye"                  , &Simulation::_eye                  , allow_raw_pointers())
      .function("_mulMatMat"            , &Simulation::_mulMatMat            , allow_raw_pointers())
      .function("_mulMatMatT"           , &Simulation::_mulMatMatT           , allow_raw_pointers())
      .function("_mulMatTMat"           , &Simulation::_mulMatTMat           , allow_raw_pointers())
      .function("_sqrMatTD"             , &Simulation::_sqrMatTD             , allow_raw_pointers())
      .function("_cholFactor"           , &Simulation::_cholFactor           , allow_raw_pointers())
      .function("_cholSolve"            , &Simulation::_cholSolve            , allow_raw_pointers())
      .function("_cholUpdate"           , &Simulation::_cholUpdate           , allow_raw_pointers())
      .function("_encodePyramid"        , &Simulation::_encodePyramid        , allow_raw_pointers())
      .function("_decodePyramid"        , &Simulation::_decodePyramid        , allow_raw_pointers())
      .function("_springDamper"         , &Simulation::_springDamper         )
      .function("_min"                  , &Simulation::_min                  )
      .function("_max"                  , &Simulation::_max                  )
      .function("_clip"                 , &Simulation::_clip                 )
      .function("_sign"                 , &Simulation::_sign                 )
      .function("_round"                , &Simulation::_round                )
      .function("_type2Str"             , &Simulation::_type2Str             )
      .function("_str2Type"             , &Simulation::_str2Type             )
      .function("_writeNumBytes"        , &Simulation::_writeNumBytes        )
      .function("_warningText"          , &Simulation::_warningText          )
      .function("_isBad"                , &Simulation::_isBad                )
      .function("_isZero"               , &Simulation::_isZero               , allow_raw_pointers())
      .function("_standardNormal"       , &Simulation::_standardNormal       , allow_raw_pointers())
      .function("_insertionSort"        , &Simulation::_insertionSort        , allow_raw_pointers())
      .function("_Halton"               , &Simulation::_Halton               )
      .function("_sigmoid"              , &Simulation::_sigmoid              )
      .function("_transitionFD"         , &Simulation::_transitionFD         , allow_raw_pointers())
      .function("_pluginCount"          , &Simulation::_pluginCount          )
      ;

  value_object<mjModel>("mjModel")
      .field("ngeom"      , &mjModel::ngeom)
      .field("nq"         , &mjModel::nq)
      .field("na"         , &mjModel::na)
      .field("nv"         , &mjModel::nv)
      .field("nu"         , &mjModel::nu)
      .field("nbody"      , &mjModel::nbody)
      .field("nsensordata", &mjModel::nsensordata)
      //.field("body_rootid", &mjModel::body_rootid, allow_raw_pointers())
      .field("nmesh"      , &mjModel::nmesh)
      .field("nmeshvert"  , &mjModel::nmeshvert)
      .field("nmeshface"  , &mjModel::nmeshface);

  value_object<mjvPerturb>("mjvPerturb")
      .field("select"    , &mjvPerturb::select)     // selected body id; non-positive: none
      .field("skinselect", &mjvPerturb::skinselect) // selected skin id; negative: none
      .field("active"    , &mjvPerturb::active)     // perturbation bitmask (mjtPertBit)
      .field("active2"   , &mjvPerturb::active2)    // secondary perturbation bitmask (mjtPertBit)
      .field("refpos"    , &mjvPerturb::refpos)     // desired position for selected object
      .field("refquat"   , &mjvPerturb::refquat)    // desired orientation for selected object
      .field("localpos"  , &mjvPerturb::localpos)   // selection point in object coordinates
      .field("scale"     , &mjvPerturb::scale)      // relative mouse motion-to-space scaling (set by initPerturb)
      ;

  value_object<mjContact>("mjContact")
      .field("dist"         , &mjContact::dist)             // distance between nearest points; neg: penetration
      .field("pos"          , &mjContact::pos)              // position of contact point: midpoint between geoms
      .field("frame"        , &mjContact::frame)            // normal is in [0-2]
      .field("includemargin", &mjContact::includemargin)    // include if dist<includemargin=margin-gap
      .field("friction"     , &mjContact::friction)         // tangent1, 2, spin, roll1, 2
      .field("solref"       , &mjContact::solref)           // constraint solver reference
      .field("solimp"       , &mjContact::solimp)           // constraint solver impedance
      .field("mu"           , &mjContact::mu)               // friction of regularized cone, set by mj_makeConstraint
      .field("H"            , &mjContact::H)                // cone Hessian, set by mj_updateConstraint
      .field("dim"          , &mjContact::H)                // contact space dimensionality: 1, 3, 4 or 6
      .field("geom1"        , &mjContact::H)                // id of geom 1
      .field("geom2"        , &mjContact::H)                // id of geom 2
      .field("exclude"      , &mjContact::exclude)          // 0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs
      .field("efc_address"  , &mjContact::efc_address);     // address in efc; -1: not included, -2-i: distance constraint i

  value_object<mjLROpt>("mjLROpt")
      .field("mode"       , &mjLROpt::mode)
      .field("useexisting", &mjLROpt::useexisting)
      .field("uselimit"   , &mjLROpt::uselimit)
      .field("accel"      , &mjLROpt::accel)      // target acceleration used to compute force
      .field("maxforce"   , &mjLROpt::maxforce)   // maximum force; 0: no limit
      .field("timeconst"  , &mjLROpt::timeconst)  // time constant for velocity reduction; min 0.01
      .field("timestep"   , &mjLROpt::timestep)   // simulation timestep; 0: use mjOption.timestep
      .field("inttotal"   , &mjLROpt::inttotal)   // total simulation time interval
      .field("inteval"    , &mjLROpt::inteval)    // evaluation time interval (at the end)
      .field("tolrange"   , &mjLROpt::tolrange);  // convergence tolerance (relative to range)

  value_object<mjOption>("mjOption")
      .field("timestep"            , &mjOption::timestep)          // timestep
      .field("apirate"             , &mjOption::apirate)           // update rate for remote API (Hz)
      .field("impratio"            , &mjOption::impratio)          // ratio of friction-to-normal contact impedance
      .field("tolerance"           , &mjOption::tolerance)         // main solver tolerance
      .field("noslip_tolerance"    , &mjOption::noslip_tolerance)  // noslip solver tolerance
      .field("mpr_tolerance"       , &mjOption::mpr_tolerance)     // MPR solver tolerance
      //.field("gravity"           , &mjOption::gravity)           // gravitational acceleration
      //.field("wind"              , &mjOption::wind)              // wind (for lift, drag and viscosity)
      //.field("magnetic"          , &mjOption::magnetic)          // global magnetic flux
      .field("density"             , &mjOption::density)           // density of medium
      .field("viscosity"           , &mjOption::viscosity)         // viscosity of medium
      .field("o_margin"            , &mjOption::o_margin)          // margin
      //.field("o_solref"          , &mjOption::o_solref)          // solref
      //.field("o_solimp"          , &mjOption::o_solimp)          // solimp
      .field("integrator"          , &mjOption::integrator)        // integration mode (mjtIntegrator)
      .field("collision"           , &mjOption::collision)         // collision mode (mjtCollision)
      .field("cone"                , &mjOption::cone)              // type of friction cone (mjtCone)
      .field("jacobian"            , &mjOption::jacobian)          // type of Jacobian (mjtJacobian)
      .field("solver"              , &mjOption::solver)            // solver algorithm (mjtSolver)
      .field("iterations"          , &mjOption::iterations)        // maximum number of main solver iterations
      .field("noslip_iterations"   , &mjOption::noslip_iterations) // maximum number of noslip solver iterations
      .field("mpr_iterations"      , &mjOption::mpr_iterations)    // maximum number of MPR solver iterations
      .field("disableflags"        , &mjOption::disableflags)      // bit flags for disabling standard features
      .field("enableflags"         , &mjOption::enableflags);      // bit flags for enabling optional features

  register_vector<mjContact>("vector<mjContact>");
}
