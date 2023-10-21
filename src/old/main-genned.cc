// THIS FILE IS AUTO GENERATED - SEE parse_headers.py FOR HOW IT GETS GENERATED!
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

class LROpt {
public:
  LROpt() { m = NULL; }

  mjLROpt *ptr  () { return m; }
  mjLROpt getVal() { return *m; }

  int mode                  () { return m->mode                  ; } // options for mj_setLengthRange()
// flags
  int useexisting           () { return m->useexisting           ; } // which actuators to process (mjtLRMode)
  int uselimit              () { return m->uselimit              ; } // use existing length range if available
  mjtNum accel                 () { return m->accel                 ; } // algorithm parameters
  mjtNum maxforce              () { return m->maxforce              ; } // target acceleration used to compute force
  mjtNum timeconst             () { return m->timeconst             ; } // maximum force; 0: no limit
  mjtNum timestep              () { return m->timestep              ; } // time constant for velocity reduction; min 0.01
  mjtNum inttotal              () { return m->inttotal              ; } // simulation timestep; 0: use mjOption.timestep
  mjtNum inteval               () { return m->inteval               ; } // total simulation time interval
  mjtNum tolrange              () { return m->tolrange              ; } // evaluation time interval (at the end)
private:
  mjLROpt *m;
};
class VFS {
public:
  VFS() { m = NULL; }

  mjVFS *ptr  () { return m; }
  mjVFS getVal() { return *m; }

  int nfile                 () { return m->nfile                 ; } // virtual file system for loading from memory
private:
  mjVFS *m;
};
class Option {
public:
  Option() { m = NULL; }

  mjOption *ptr  () { return m; }
  mjOption getVal() { return *m; }

  mjtNum timestep              () { return m->timestep              ; } // physics options
// timing parameters
  mjtNum apirate               () { return m->apirate               ; } // timestep
  mjtNum impratio              () { return m->impratio              ; } // solver parameters
  mjtNum tolerance             () { return m->tolerance             ; } // ratio of friction-to-normal contact impedance
  mjtNum noslip_tolerance      () { return m->noslip_tolerance      ; } // main solver tolerance
  mjtNum mpr_tolerance         () { return m->mpr_tolerance         ; } // noslip solver tolerance
  mjtNum density               () { return m->density               ; } // global magnetic flux
  mjtNum viscosity             () { return m->viscosity             ; } // density of medium
  mjtNum o_margin              () { return m->o_margin              ; } // override contact solver parameters (if enabled)
  int integrator            () { return m->integrator            ; } // discrete settings
  int collision             () { return m->collision             ; } // integration mode (mjtIntegrator)
  int cone                  () { return m->cone                  ; } // collision mode (mjtCollision)
  int jacobian              () { return m->jacobian              ; } // type of friction cone (mjtCone)
  int solver                () { return m->solver                ; } // type of Jacobian (mjtJacobian)
  int iterations            () { return m->iterations            ; } // solver algorithm (mjtSolver)
  int noslip_iterations     () { return m->noslip_iterations     ; } // maximum number of main solver iterations
  int mpr_iterations        () { return m->mpr_iterations        ; } // maximum number of noslip solver iterations
  int disableflags          () { return m->disableflags          ; } // maximum number of MPR solver iterations
  int enableflags           () { return m->enableflags           ; } // bit flags for disabling standard features
private:
  mjOption *m;
};
class Visual {
public:
  Visual() { m = NULL; }

  mjVisual *ptr  () { return m; }
  mjVisual getVal() { return *m; }

private:
  mjVisual *m;
};
class Statistic {
public:
  Statistic() { m = NULL; }

  mjStatistic *ptr  () { return m; }
  mjStatistic getVal() { return *m; }

  mjtNum meaninertia           () { return m->meaninertia           ; } // model statistics (in qpos0)
  mjtNum meanmass              () { return m->meanmass              ; } // mean diagonal inertia
  mjtNum meansize              () { return m->meansize              ; } // mean body mass
  mjtNum extent                () { return m->extent                ; } // mean body size
private:
  mjStatistic *m;
};
class Model {
public:
  Model() { m = NULL; }

  mjModel *ptr  () { return m; }
  mjModel getVal() { return *m; }

  int nq                    () { return m->nq                    ; } // sizes needed at mjModel construction
  int nv                    () { return m->nv                    ; } // number of generalized coordinates = dim(qpos)
  int nu                    () { return m->nu                    ; } // number of degrees of freedom = dim(qvel)
  int na                    () { return m->na                    ; } // number of actuators/controls = dim(ctrl)
  int nbody                 () { return m->nbody                 ; } // number of activation states = dim(act)
  int njnt                  () { return m->njnt                  ; } // number of bodies
  int ngeom                 () { return m->ngeom                 ; } // number of joints
  int nsite                 () { return m->nsite                 ; } // number of geoms
  int ncam                  () { return m->ncam                  ; } // number of sites
  int nlight                () { return m->nlight                ; } // number of cameras
  int nmesh                 () { return m->nmesh                 ; } // number of lights
  int nmeshvert             () { return m->nmeshvert             ; } // number of meshes
  int nmeshtexvert          () { return m->nmeshtexvert          ; } // number of vertices in all meshes
  int nmeshface             () { return m->nmeshface             ; } // number of vertices with texcoords in all meshes
  int nmeshgraph            () { return m->nmeshgraph            ; } // number of triangular faces in all meshes
  int nskin                 () { return m->nskin                 ; } // number of ints in mesh auxiliary data
  int nskinvert             () { return m->nskinvert             ; } // number of skins
  int nskintexvert          () { return m->nskintexvert          ; } // number of vertices in all skins
  int nskinface             () { return m->nskinface             ; } // number of vertiex with texcoords in all skins
  int nskinbone             () { return m->nskinbone             ; } // number of triangular faces in all skins
  int nskinbonevert         () { return m->nskinbonevert         ; } // number of bones in all skins
  int nhfield               () { return m->nhfield               ; } // number of vertices in all skin bones
  int nhfielddata           () { return m->nhfielddata           ; } // number of heightfields
  int ntex                  () { return m->ntex                  ; } // number of data points in all heightfields
  int ntexdata              () { return m->ntexdata              ; } // number of textures
  int nmat                  () { return m->nmat                  ; } // number of bytes in texture rgb data
  int npair                 () { return m->npair                 ; } // number of materials
  int nexclude              () { return m->nexclude              ; } // number of predefined geom pairs
  int neq                   () { return m->neq                   ; } // number of excluded geom pairs
  int ntendon               () { return m->ntendon               ; } // number of equality constraints
  int nwrap                 () { return m->nwrap                 ; } // number of tendons
  int nsensor               () { return m->nsensor               ; } // number of wrap objects in all tendon paths
  int nnumeric              () { return m->nnumeric              ; } // number of sensors
  int nnumericdata          () { return m->nnumericdata          ; } // number of numeric custom fields
  int ntext                 () { return m->ntext                 ; } // number of mjtNums in all numeric fields
  int ntextdata             () { return m->ntextdata             ; } // number of text custom fields
  int ntuple                () { return m->ntuple                ; } // number of mjtBytes in all text fields
  int ntupledata            () { return m->ntupledata            ; } // number of tuple custom fields
  int nkey                  () { return m->nkey                  ; } // number of objects in all tuple fields
  int nmocap                () { return m->nmocap                ; } // number of keyframes
  int nplugin               () { return m->nplugin               ; } // number of mocap bodies
  int npluginattr           () { return m->npluginattr           ; } // number of plugin instances
  int nuser_body            () { return m->nuser_body            ; } // number of chars in all plugin config attributes
  int nuser_jnt             () { return m->nuser_jnt             ; } // number of mjtNums in body_user
  int nuser_geom            () { return m->nuser_geom            ; } // number of mjtNums in jnt_user
  int nuser_site            () { return m->nuser_site            ; } // number of mjtNums in geom_user
  int nuser_cam             () { return m->nuser_cam             ; } // number of mjtNums in site_user
  int nuser_tendon          () { return m->nuser_tendon          ; } // number of mjtNums in cam_user
  int nuser_actuator        () { return m->nuser_actuator        ; } // number of mjtNums in tendon_user
  int nuser_sensor          () { return m->nuser_sensor          ; } // number of mjtNums in actuator_user
  int nnames                () { return m->nnames                ; } // number of mjtNums in sensor_user
  int nM                    () { return m->nM                    ; } // sizes set after mjModel construction (only affect mjData)
  int nD                    () { return m->nD                    ; } // number of non-zeros in sparse inertia matrix
  int nemax                 () { return m->nemax                 ; } // number of non-zeros in sparse derivative matrix
  int njmax                 () { return m->njmax                 ; } // number of potential equality-constraint rows
  int nconmax               () { return m->nconmax               ; } // number of available rows in constraint Jacobian
  int nstack                () { return m->nstack                ; } // number of potential contacts in contact list
  int nuserdata             () { return m->nuserdata             ; } // number of fields in mjData stack
  int nsensordata           () { return m->nsensordata           ; } // number of extra fields in mjData
  int npluginstate          () { return m->npluginstate          ; } // number of fields in sensor data vector
  int nbuffer               () { return m->nbuffer               ; } // number of bytes in buffer
  mjOption opt                   () { return m->opt                   ; } // physics options
  mjVisual vis                   () { return m->vis                   ; } // visualization options
  mjStatistic stat                  () { return m->stat                  ; } // model statistics
  val buffer                () { return val(typed_memory_view(m->nmesh, m->buffer                 )); } // main buffer
  val qpos0                 () { return val(typed_memory_view(m->nmesh, m->qpos0                  )); } // default generalized coordinates
  val qpos_spring           () { return val(typed_memory_view(m->nmesh, m->qpos_spring            )); } // qpos values at default pose              (nq x 1)
  val body_parentid         () { return val(typed_memory_view(m->nmesh, m->body_parentid          )); } // bodies
  val body_rootid           () { return val(typed_memory_view(m->nmesh, m->body_rootid            )); } // id of body's parent                      (nbody x 1)
  val body_weldid           () { return val(typed_memory_view(m->nmesh, m->body_weldid            )); } // id of root above body                    (nbody x 1)
  val body_mocapid          () { return val(typed_memory_view(m->nmesh, m->body_mocapid           )); } // id of body that this body is welded to   (nbody x 1)
  val body_jntnum           () { return val(typed_memory_view(m->nmesh, m->body_jntnum            )); } // id of mocap data; -1: none               (nbody x 1)
  val body_jntadr           () { return val(typed_memory_view(m->nmesh, m->body_jntadr            )); } // number of joints for this body           (nbody x 1)
  val body_dofnum           () { return val(typed_memory_view(m->nmesh, m->body_dofnum            )); } // start addr of joints; -1: no joints      (nbody x 1)
  val body_dofadr           () { return val(typed_memory_view(m->nmesh, m->body_dofadr            )); } // number of motion degrees of freedom      (nbody x 1)
  val body_geomnum          () { return val(typed_memory_view(m->nmesh, m->body_geomnum           )); } // start addr of dofs; -1: no dofs          (nbody x 1)
  val body_geomadr          () { return val(typed_memory_view(m->nmesh, m->body_geomadr           )); } // number of geoms                          (nbody x 1)
  val body_simple           () { return val(typed_memory_view(m->nmesh, m->body_simple            )); } // start addr of geoms; -1: no geoms        (nbody x 1)
  val body_sameframe        () { return val(typed_memory_view(m->nmesh, m->body_sameframe         )); } // body is simple (has diagonal M)          (nbody x 1)
  val body_pos              () { return val(typed_memory_view(m->nmesh, m->body_pos               )); } // inertial frame is same as body frame     (nbody x 1)
  val body_quat             () { return val(typed_memory_view(m->nmesh, m->body_quat              )); } // position offset rel. to parent body      (nbody x 3)
  val body_ipos             () { return val(typed_memory_view(m->nmesh, m->body_ipos              )); } // orientation offset rel. to parent body   (nbody x 4)
  val body_iquat            () { return val(typed_memory_view(m->nmesh, m->body_iquat             )); } // local position of center of mass         (nbody x 3)
  val body_mass             () { return val(typed_memory_view(m->nmesh, m->body_mass              )); } // local orientation of inertia ellipsoid   (nbody x 4)
  val body_subtreemass      () { return val(typed_memory_view(m->nmesh, m->body_subtreemass       )); } // mass                                     (nbody x 1)
  val body_inertia          () { return val(typed_memory_view(m->nmesh, m->body_inertia           )); } // mass of subtree starting at this body    (nbody x 1)
  val body_invweight0       () { return val(typed_memory_view(m->nmesh, m->body_invweight0        )); } // diagonal inertia in ipos/iquat frame     (nbody x 3)
  val body_gravcomp         () { return val(typed_memory_view(m->nmesh, m->body_gravcomp          )); } // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
  val body_user             () { return val(typed_memory_view(m->nmesh, m->body_user              )); } // antigravity force, units of body weight  (nbody x 1)
  val body_plugin           () { return val(typed_memory_view(m->nmesh, m->body_plugin            )); } // user data                                (nbody x nuser_body)
  val jnt_type              () { return val(typed_memory_view(m->nmesh, m->jnt_type               )); } // joints
  val jnt_qposadr           () { return val(typed_memory_view(m->nmesh, m->jnt_qposadr            )); } // type of joint (mjtJoint)                 (njnt x 1)
  val jnt_dofadr            () { return val(typed_memory_view(m->nmesh, m->jnt_dofadr             )); } // start addr in 'qpos' for joint's data    (njnt x 1)
  val jnt_bodyid            () { return val(typed_memory_view(m->nmesh, m->jnt_bodyid             )); } // start addr in 'qvel' for joint's data    (njnt x 1)
  val jnt_group             () { return val(typed_memory_view(m->nmesh, m->jnt_group              )); } // id of joint's body                       (njnt x 1)
  val jnt_limited           () { return val(typed_memory_view(m->nmesh, m->jnt_limited            )); } // group for visibility                     (njnt x 1)
  val jnt_solref            () { return val(typed_memory_view(m->nmesh, m->jnt_solref             )); } // does joint have limits                   (njnt x 1)
  val jnt_solimp            () { return val(typed_memory_view(m->nmesh, m->jnt_solimp             )); } // constraint solver reference: limit       (njnt x mjNREF)
  val jnt_pos               () { return val(typed_memory_view(m->nmesh, m->jnt_pos                )); } // constraint solver impedance: limit       (njnt x mjNIMP)
  val jnt_axis              () { return val(typed_memory_view(m->nmesh, m->jnt_axis               )); } // local anchor position                    (njnt x 3)
  val jnt_stiffness         () { return val(typed_memory_view(m->nmesh, m->jnt_stiffness          )); } // local joint axis                         (njnt x 3)
  val jnt_range             () { return val(typed_memory_view(m->nmesh, m->jnt_range              )); } // stiffness coefficient                    (njnt x 1)
  val jnt_margin            () { return val(typed_memory_view(m->nmesh, m->jnt_margin             )); } // joint limits                             (njnt x 2)
  val jnt_user              () { return val(typed_memory_view(m->nmesh, m->jnt_user               )); } // min distance for limit detection         (njnt x 1)
  val dof_bodyid            () { return val(typed_memory_view(m->nmesh, m->dof_bodyid             )); } // dofs
  val dof_jntid             () { return val(typed_memory_view(m->nmesh, m->dof_jntid              )); } // id of dof's body                         (nv x 1)
  val dof_parentid          () { return val(typed_memory_view(m->nmesh, m->dof_parentid           )); } // id of dof's joint                        (nv x 1)
  val dof_Madr              () { return val(typed_memory_view(m->nmesh, m->dof_Madr               )); } // id of dof's parent; -1: none             (nv x 1)
  val dof_simplenum         () { return val(typed_memory_view(m->nmesh, m->dof_simplenum          )); } // dof address in M-diagonal                (nv x 1)
  val dof_solref            () { return val(typed_memory_view(m->nmesh, m->dof_solref             )); } // number of consecutive simple dofs        (nv x 1)
  val dof_solimp            () { return val(typed_memory_view(m->nmesh, m->dof_solimp             )); } // constraint solver reference:frictionloss (nv x mjNREF)
  val dof_frictionloss      () { return val(typed_memory_view(m->nmesh, m->dof_frictionloss       )); } // constraint solver impedance:frictionloss (nv x mjNIMP)
  val dof_armature          () { return val(typed_memory_view(m->nmesh, m->dof_armature           )); } // dof friction loss                        (nv x 1)
  val dof_damping           () { return val(typed_memory_view(m->nmesh, m->dof_damping            )); } // dof armature inertia/mass                (nv x 1)
  val dof_invweight0        () { return val(typed_memory_view(m->nmesh, m->dof_invweight0         )); } // damping coefficient                      (nv x 1)
  val dof_M0                () { return val(typed_memory_view(m->nmesh, m->dof_M0                 )); } // diag. inverse inertia in qpos0           (nv x 1)
  val geom_type             () { return val(typed_memory_view(m->nmesh, m->geom_type              )); } // geoms
  val geom_contype          () { return val(typed_memory_view(m->nmesh, m->geom_contype           )); } // geometric type (mjtGeom)                 (ngeom x 1)
  val geom_conaffinity      () { return val(typed_memory_view(m->nmesh, m->geom_conaffinity       )); } // geom contact type                        (ngeom x 1)
  val geom_condim           () { return val(typed_memory_view(m->nmesh, m->geom_condim            )); } // geom contact affinity                    (ngeom x 1)
  val geom_bodyid           () { return val(typed_memory_view(m->nmesh, m->geom_bodyid            )); } // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
  val geom_dataid           () { return val(typed_memory_view(m->nmesh, m->geom_dataid            )); } // id of geom's body                        (ngeom x 1)
  val geom_matid            () { return val(typed_memory_view(m->nmesh, m->geom_matid             )); } // id of geom's mesh/hfield (-1: none)      (ngeom x 1)
  val geom_group            () { return val(typed_memory_view(m->nmesh, m->geom_group             )); } // material id for rendering                (ngeom x 1)
  val geom_priority         () { return val(typed_memory_view(m->nmesh, m->geom_priority          )); } // group for visibility                     (ngeom x 1)
  val geom_sameframe        () { return val(typed_memory_view(m->nmesh, m->geom_sameframe         )); } // geom contact priority                    (ngeom x 1)
  val geom_solmix           () { return val(typed_memory_view(m->nmesh, m->geom_solmix            )); } // same as body frame (1) or iframe (2)     (ngeom x 1)
  val geom_solref           () { return val(typed_memory_view(m->nmesh, m->geom_solref            )); } // mixing coef for solref/imp in geom pair  (ngeom x 1)
  val geom_solimp           () { return val(typed_memory_view(m->nmesh, m->geom_solimp            )); } // constraint solver reference: contact     (ngeom x mjNREF)
  val geom_size             () { return val(typed_memory_view(m->nmesh, m->geom_size              )); } // constraint solver impedance: contact     (ngeom x mjNIMP)
  val geom_rbound           () { return val(typed_memory_view(m->nmesh, m->geom_rbound            )); } // geom-specific size parameters            (ngeom x 3)
  val geom_pos              () { return val(typed_memory_view(m->nmesh, m->geom_pos               )); } // radius of bounding sphere                (ngeom x 1)
  val geom_quat             () { return val(typed_memory_view(m->nmesh, m->geom_quat              )); } // local position offset rel. to body       (ngeom x 3)
  val geom_friction         () { return val(typed_memory_view(m->nmesh, m->geom_friction          )); } // local orientation offset rel. to body    (ngeom x 4)
  val geom_margin           () { return val(typed_memory_view(m->nmesh, m->geom_margin            )); } // friction for (slide, spin, roll)         (ngeom x 3)
  val geom_gap              () { return val(typed_memory_view(m->nmesh, m->geom_gap               )); } // detect contact if dist<margin            (ngeom x 1)
  val geom_fluid            () { return val(typed_memory_view(m->nmesh, m->geom_fluid             )); } // include in solver if dist<margin-gap     (ngeom x 1)
  val geom_user             () { return val(typed_memory_view(m->nmesh, m->geom_user              )); } // fluid interaction parameters             (ngeom x mjNFLUID)
  val geom_rgba             () { return val(typed_memory_view(m->nmesh, m->geom_rgba              )); } // user data                                (ngeom x nuser_geom)
  val site_type             () { return val(typed_memory_view(m->nmesh, m->site_type              )); } // sites
  val site_bodyid           () { return val(typed_memory_view(m->nmesh, m->site_bodyid            )); } // geom type for rendering (mjtGeom)        (nsite x 1)
  val site_matid            () { return val(typed_memory_view(m->nmesh, m->site_matid             )); } // id of site's body                        (nsite x 1)
  val site_group            () { return val(typed_memory_view(m->nmesh, m->site_group             )); } // material id for rendering                (nsite x 1)
  val site_sameframe        () { return val(typed_memory_view(m->nmesh, m->site_sameframe         )); } // group for visibility                     (nsite x 1)
  val site_size             () { return val(typed_memory_view(m->nmesh, m->site_size              )); } // same as body frame (1) or iframe (2)     (nsite x 1)
  val site_pos              () { return val(typed_memory_view(m->nmesh, m->site_pos               )); } // geom size for rendering                  (nsite x 3)
  val site_quat             () { return val(typed_memory_view(m->nmesh, m->site_quat              )); } // local position offset rel. to body       (nsite x 3)
  val site_user             () { return val(typed_memory_view(m->nmesh, m->site_user              )); } // local orientation offset rel. to body    (nsite x 4)
  val site_rgba             () { return val(typed_memory_view(m->nmesh, m->site_rgba              )); } // user data                                (nsite x nuser_site)
  val cam_mode              () { return val(typed_memory_view(m->nmesh, m->cam_mode               )); } // cameras
  val cam_bodyid            () { return val(typed_memory_view(m->nmesh, m->cam_bodyid             )); } // camera tracking mode (mjtCamLight)       (ncam x 1)
  val cam_targetbodyid      () { return val(typed_memory_view(m->nmesh, m->cam_targetbodyid       )); } // id of camera's body                      (ncam x 1)
  val cam_pos               () { return val(typed_memory_view(m->nmesh, m->cam_pos                )); } // id of targeted body; -1: none            (ncam x 1)
  val cam_quat              () { return val(typed_memory_view(m->nmesh, m->cam_quat               )); } // position rel. to body frame              (ncam x 3)
  val cam_poscom0           () { return val(typed_memory_view(m->nmesh, m->cam_poscom0            )); } // orientation rel. to body frame           (ncam x 4)
  val cam_pos0              () { return val(typed_memory_view(m->nmesh, m->cam_pos0               )); } // global position rel. to sub-com in qpos0 (ncam x 3)
  val cam_mat0              () { return val(typed_memory_view(m->nmesh, m->cam_mat0               )); } // global position rel. to body in qpos0    (ncam x 3)
  val cam_fovy              () { return val(typed_memory_view(m->nmesh, m->cam_fovy               )); } // global orientation in qpos0              (ncam x 9)
  val cam_ipd               () { return val(typed_memory_view(m->nmesh, m->cam_ipd                )); } // y-field of view (deg)                    (ncam x 1)
  val cam_user              () { return val(typed_memory_view(m->nmesh, m->cam_user               )); } // inter-pupilary distance                  (ncam x 1)
  val light_mode            () { return val(typed_memory_view(m->nmesh, m->light_mode             )); } // lights
  val light_bodyid          () { return val(typed_memory_view(m->nmesh, m->light_bodyid           )); } // light tracking mode (mjtCamLight)        (nlight x 1)
  val light_targetbodyid    () { return val(typed_memory_view(m->nmesh, m->light_targetbodyid     )); } // id of light's body                       (nlight x 1)
  val light_directional     () { return val(typed_memory_view(m->nmesh, m->light_directional      )); } // id of targeted body; -1: none            (nlight x 1)
  val light_castshadow      () { return val(typed_memory_view(m->nmesh, m->light_castshadow       )); } // directional light                        (nlight x 1)
  val light_active          () { return val(typed_memory_view(m->nmesh, m->light_active           )); } // does light cast shadows                  (nlight x 1)
  val light_pos             () { return val(typed_memory_view(m->nmesh, m->light_pos              )); } // is light on                              (nlight x 1)
  val light_dir             () { return val(typed_memory_view(m->nmesh, m->light_dir              )); } // position rel. to body frame              (nlight x 3)
  val light_poscom0         () { return val(typed_memory_view(m->nmesh, m->light_poscom0          )); } // direction rel. to body frame             (nlight x 3)
  val light_pos0            () { return val(typed_memory_view(m->nmesh, m->light_pos0             )); } // global position rel. to sub-com in qpos0 (nlight x 3)
  val light_dir0            () { return val(typed_memory_view(m->nmesh, m->light_dir0             )); } // global position rel. to body in qpos0    (nlight x 3)
  val light_attenuation     () { return val(typed_memory_view(m->nmesh, m->light_attenuation      )); } // global direction in qpos0                (nlight x 3)
  val light_cutoff          () { return val(typed_memory_view(m->nmesh, m->light_cutoff           )); } // OpenGL attenuation (quadratic model)     (nlight x 3)
  val light_exponent        () { return val(typed_memory_view(m->nmesh, m->light_exponent         )); } // OpenGL cutoff                            (nlight x 1)
  val light_ambient         () { return val(typed_memory_view(m->nmesh, m->light_ambient          )); } // OpenGL exponent                          (nlight x 1)
  val light_diffuse         () { return val(typed_memory_view(m->nmesh, m->light_diffuse          )); } // ambient rgb (alpha=1)                    (nlight x 3)
  val light_specular        () { return val(typed_memory_view(m->nmesh, m->light_specular         )); } // diffuse rgb (alpha=1)                    (nlight x 3)
  val mesh_vertadr          () { return val(typed_memory_view(m->nmesh, m->mesh_vertadr           )); } // meshes
  val mesh_vertnum          () { return val(typed_memory_view(m->nmesh, m->mesh_vertnum           )); } // first vertex address                     (nmesh x 1)
  val mesh_texcoordadr      () { return val(typed_memory_view(m->nmesh, m->mesh_texcoordadr       )); } // number of vertices                       (nmesh x 1)
  val mesh_faceadr          () { return val(typed_memory_view(m->nmesh, m->mesh_faceadr           )); } // texcoord data address; -1: no texcoord   (nmesh x 1)
  val mesh_facenum          () { return val(typed_memory_view(m->nmesh, m->mesh_facenum           )); } // first face address                       (nmesh x 1)
  val mesh_graphadr         () { return val(typed_memory_view(m->nmesh, m->mesh_graphadr          )); } // number of faces                          (nmesh x 1)
  val mesh_vert             () { return val(typed_memory_view(m->nmesh, m->mesh_vert              )); } // graph data address; -1: no graph         (nmesh x 1)
  val mesh_normal           () { return val(typed_memory_view(m->nmesh, m->mesh_normal            )); } // vertex positions for all meshes          (nmeshvert x 3)
  val mesh_texcoord         () { return val(typed_memory_view(m->nmesh, m->mesh_texcoord          )); } // vertex normals for all meshes            (nmeshvert x 3)
  val mesh_face             () { return val(typed_memory_view(m->nmesh, m->mesh_face              )); } // vertex texcoords for all meshes          (nmeshtexvert x 2)
  val mesh_graph            () { return val(typed_memory_view(m->nmesh, m->mesh_graph             )); } // triangle face data                       (nmeshface x 3)
  val skin_matid            () { return val(typed_memory_view(m->nmesh, m->skin_matid             )); } // skins
  val skin_group            () { return val(typed_memory_view(m->nmesh, m->skin_group             )); } // skin material id; -1: none               (nskin x 1)
  val skin_rgba             () { return val(typed_memory_view(m->nmesh, m->skin_rgba              )); } // group for visibility                     (nskin x 1)
  val skin_inflate          () { return val(typed_memory_view(m->nmesh, m->skin_inflate           )); } // skin rgba                                (nskin x 4)
  val skin_vertadr          () { return val(typed_memory_view(m->nmesh, m->skin_vertadr           )); } // inflate skin in normal direction         (nskin x 1)
  val skin_vertnum          () { return val(typed_memory_view(m->nmesh, m->skin_vertnum           )); } // first vertex address                     (nskin x 1)
  val skin_texcoordadr      () { return val(typed_memory_view(m->nmesh, m->skin_texcoordadr       )); } // number of vertices                       (nskin x 1)
  val skin_faceadr          () { return val(typed_memory_view(m->nmesh, m->skin_faceadr           )); } // texcoord data address; -1: no texcoord   (nskin x 1)
  val skin_facenum          () { return val(typed_memory_view(m->nmesh, m->skin_facenum           )); } // first face address                       (nskin x 1)
  val skin_boneadr          () { return val(typed_memory_view(m->nmesh, m->skin_boneadr           )); } // number of faces                          (nskin x 1)
  val skin_bonenum          () { return val(typed_memory_view(m->nmesh, m->skin_bonenum           )); } // first bone in skin                       (nskin x 1)
  val skin_vert             () { return val(typed_memory_view(m->nmesh, m->skin_vert              )); } // number of bones in skin                  (nskin x 1)
  val skin_texcoord         () { return val(typed_memory_view(m->nmesh, m->skin_texcoord          )); } // vertex positions for all skin meshes     (nskinvert x 3)
  val skin_face             () { return val(typed_memory_view(m->nmesh, m->skin_face              )); } // vertex texcoords for all skin meshes     (nskintexvert x 2)
  val skin_bonevertadr      () { return val(typed_memory_view(m->nmesh, m->skin_bonevertadr       )); } // triangle faces for all skin meshes       (nskinface x 3)
  val skin_bonevertnum      () { return val(typed_memory_view(m->nmesh, m->skin_bonevertnum       )); } // first vertex in each bone                (nskinbone x 1)
  val skin_bonebindpos      () { return val(typed_memory_view(m->nmesh, m->skin_bonebindpos       )); } // number of vertices in each bone          (nskinbone x 1)
  val skin_bonebindquat     () { return val(typed_memory_view(m->nmesh, m->skin_bonebindquat      )); } // bind pos of each bone                    (nskinbone x 3)
  val skin_bonebodyid       () { return val(typed_memory_view(m->nmesh, m->skin_bonebodyid        )); } // bind quat of each bone                   (nskinbone x 4)
  val skin_bonevertid       () { return val(typed_memory_view(m->nmesh, m->skin_bonevertid        )); } // body id of each bone                     (nskinbone x 1)
  val skin_bonevertweight   () { return val(typed_memory_view(m->nmesh, m->skin_bonevertweight    )); } // mesh ids of vertices in each bone        (nskinbonevert x 1)
  val hfield_size           () { return val(typed_memory_view(m->nmesh, m->hfield_size            )); } // height fields
  val hfield_nrow           () { return val(typed_memory_view(m->nmesh, m->hfield_nrow            )); } // (x, y, z_top, z_bottom)                  (nhfield x 4)
  val hfield_ncol           () { return val(typed_memory_view(m->nmesh, m->hfield_ncol            )); } // number of rows in grid                   (nhfield x 1)
  val hfield_adr            () { return val(typed_memory_view(m->nmesh, m->hfield_adr             )); } // number of columns in grid                (nhfield x 1)
  val hfield_data           () { return val(typed_memory_view(m->nmesh, m->hfield_data            )); } // address in hfield_data                   (nhfield x 1)
  val tex_type              () { return val(typed_memory_view(m->nmesh, m->tex_type               )); } // textures
  val tex_height            () { return val(typed_memory_view(m->nmesh, m->tex_height             )); } // texture type (mjtTexture)                (ntex x 1)
  val tex_width             () { return val(typed_memory_view(m->nmesh, m->tex_width              )); } // number of rows in texture image          (ntex x 1)
  val tex_adr               () { return val(typed_memory_view(m->nmesh, m->tex_adr                )); } // number of columns in texture image       (ntex x 1)
  val tex_rgb               () { return val(typed_memory_view(m->nmesh, m->tex_rgb                )); } // address in rgb                           (ntex x 1)
  val mat_texid             () { return val(typed_memory_view(m->nmesh, m->mat_texid              )); } // materials
  val mat_texuniform        () { return val(typed_memory_view(m->nmesh, m->mat_texuniform         )); } // texture id; -1: none                     (nmat x 1)
  val mat_texrepeat         () { return val(typed_memory_view(m->nmesh, m->mat_texrepeat          )); } // make texture cube uniform                (nmat x 1)
  val mat_emission          () { return val(typed_memory_view(m->nmesh, m->mat_emission           )); } // texture repetition for 2d mapping        (nmat x 2)
  val mat_specular          () { return val(typed_memory_view(m->nmesh, m->mat_specular           )); } // emission (x rgb)                         (nmat x 1)
  val mat_shininess         () { return val(typed_memory_view(m->nmesh, m->mat_shininess          )); } // specular (x white)                       (nmat x 1)
  val mat_reflectance       () { return val(typed_memory_view(m->nmesh, m->mat_reflectance        )); } // shininess coef                           (nmat x 1)
  val mat_rgba              () { return val(typed_memory_view(m->nmesh, m->mat_rgba               )); } // reflectance (0: disable)                 (nmat x 1)
  val pair_dim              () { return val(typed_memory_view(m->nmesh, m->pair_dim               )); } // predefined geom pairs for collision detection; has precedence over exclude
  val pair_geom1            () { return val(typed_memory_view(m->nmesh, m->pair_geom1             )); } // contact dimensionality                   (npair x 1)
  val pair_geom2            () { return val(typed_memory_view(m->nmesh, m->pair_geom2             )); } // id of geom1                              (npair x 1)
  val pair_signature        () { return val(typed_memory_view(m->nmesh, m->pair_signature         )); } // id of geom2                              (npair x 1)
  val pair_solref           () { return val(typed_memory_view(m->nmesh, m->pair_solref            )); } // (body1+1)<<16 + body2+1                  (npair x 1)
  val pair_solimp           () { return val(typed_memory_view(m->nmesh, m->pair_solimp            )); } // constraint solver reference: contact     (npair x mjNREF)
  val pair_margin           () { return val(typed_memory_view(m->nmesh, m->pair_margin            )); } // constraint solver impedance: contact     (npair x mjNIMP)
  val pair_gap              () { return val(typed_memory_view(m->nmesh, m->pair_gap               )); } // detect contact if dist<margin            (npair x 1)
  val pair_friction         () { return val(typed_memory_view(m->nmesh, m->pair_friction          )); } // include in solver if dist<margin-gap     (npair x 1)
  val exclude_signature     () { return val(typed_memory_view(m->nmesh, m->exclude_signature      )); } // excluded body pairs for collision detection
  val eq_type               () { return val(typed_memory_view(m->nmesh, m->eq_type                )); } // equality constraints
  val eq_obj1id             () { return val(typed_memory_view(m->nmesh, m->eq_obj1id              )); } // constraint type (mjtEq)                  (neq x 1)
  val eq_obj2id             () { return val(typed_memory_view(m->nmesh, m->eq_obj2id              )); } // id of object 1                           (neq x 1)
  val eq_active             () { return val(typed_memory_view(m->nmesh, m->eq_active              )); } // id of object 2                           (neq x 1)
  val eq_solref             () { return val(typed_memory_view(m->nmesh, m->eq_solref              )); } // enable/disable constraint                (neq x 1)
  val eq_solimp             () { return val(typed_memory_view(m->nmesh, m->eq_solimp              )); } // constraint solver reference              (neq x mjNREF)
  val eq_data               () { return val(typed_memory_view(m->nmesh, m->eq_data                )); } // constraint solver impedance              (neq x mjNIMP)
  val tendon_adr            () { return val(typed_memory_view(m->nmesh, m->tendon_adr             )); } // tendons
  val tendon_num            () { return val(typed_memory_view(m->nmesh, m->tendon_num             )); } // address of first object in tendon's path (ntendon x 1)
  val tendon_matid          () { return val(typed_memory_view(m->nmesh, m->tendon_matid           )); } // number of objects in tendon's path       (ntendon x 1)
  val tendon_group          () { return val(typed_memory_view(m->nmesh, m->tendon_group           )); } // material id for rendering                (ntendon x 1)
  val tendon_limited        () { return val(typed_memory_view(m->nmesh, m->tendon_limited         )); } // group for visibility                     (ntendon x 1)
  val tendon_width          () { return val(typed_memory_view(m->nmesh, m->tendon_width           )); } // does tendon have length limits           (ntendon x 1)
  val tendon_solref_lim     () { return val(typed_memory_view(m->nmesh, m->tendon_solref_lim      )); } // width for rendering                      (ntendon x 1)
  val tendon_solimp_lim     () { return val(typed_memory_view(m->nmesh, m->tendon_solimp_lim      )); } // constraint solver reference: limit       (ntendon x mjNREF)
  val tendon_solref_fri     () { return val(typed_memory_view(m->nmesh, m->tendon_solref_fri      )); } // constraint solver impedance: limit       (ntendon x mjNIMP)
  val tendon_solimp_fri     () { return val(typed_memory_view(m->nmesh, m->tendon_solimp_fri      )); } // constraint solver reference: friction    (ntendon x mjNREF)
  val tendon_range          () { return val(typed_memory_view(m->nmesh, m->tendon_range           )); } // constraint solver impedance: friction    (ntendon x mjNIMP)
  val tendon_margin         () { return val(typed_memory_view(m->nmesh, m->tendon_margin          )); } // tendon length limits                     (ntendon x 2)
  val tendon_stiffness      () { return val(typed_memory_view(m->nmesh, m->tendon_stiffness       )); } // min distance for limit detection         (ntendon x 1)
  val tendon_damping        () { return val(typed_memory_view(m->nmesh, m->tendon_damping         )); } // stiffness coefficient                    (ntendon x 1)
  val tendon_frictionloss   () { return val(typed_memory_view(m->nmesh, m->tendon_frictionloss    )); } // damping coefficient                      (ntendon x 1)
  val tendon_lengthspring   () { return val(typed_memory_view(m->nmesh, m->tendon_lengthspring    )); } // loss due to friction                     (ntendon x 1)
  val tendon_length0        () { return val(typed_memory_view(m->nmesh, m->tendon_length0         )); } // spring resting length range              (ntendon x 2)
  val tendon_invweight0     () { return val(typed_memory_view(m->nmesh, m->tendon_invweight0      )); } // tendon length in qpos0                   (ntendon x 1)
  val tendon_user           () { return val(typed_memory_view(m->nmesh, m->tendon_user            )); } // inv. weight in qpos0                     (ntendon x 1)
  val tendon_rgba           () { return val(typed_memory_view(m->nmesh, m->tendon_rgba            )); } // user data                                (ntendon x nuser_tendon)
  val wrap_type             () { return val(typed_memory_view(m->nmesh, m->wrap_type              )); } // list of all wrap objects in tendon paths
  val wrap_objid            () { return val(typed_memory_view(m->nmesh, m->wrap_objid             )); } // wrap object type (mjtWrap)               (nwrap x 1)
  val wrap_prm              () { return val(typed_memory_view(m->nmesh, m->wrap_prm               )); } // object id: geom, site, joint             (nwrap x 1)
  val actuator_trntype      () { return val(typed_memory_view(m->nmesh, m->actuator_trntype       )); } // actuators
  val actuator_dyntype      () { return val(typed_memory_view(m->nmesh, m->actuator_dyntype       )); } // transmission type (mjtTrn)               (nu x 1)
  val actuator_gaintype     () { return val(typed_memory_view(m->nmesh, m->actuator_gaintype      )); } // dynamics type (mjtDyn)                   (nu x 1)
  val actuator_biastype     () { return val(typed_memory_view(m->nmesh, m->actuator_biastype      )); } // gain type (mjtGain)                      (nu x 1)
  val actuator_trnid        () { return val(typed_memory_view(m->nmesh, m->actuator_trnid         )); } // bias type (mjtBias)                      (nu x 1)
  val actuator_actadr       () { return val(typed_memory_view(m->nmesh, m->actuator_actadr        )); } // transmission id: joint, tendon, site     (nu x 2)
  val actuator_actnum       () { return val(typed_memory_view(m->nmesh, m->actuator_actnum        )); } // first activation address; -1: stateless  (nu x 1)
  val actuator_group        () { return val(typed_memory_view(m->nmesh, m->actuator_group         )); } // number of activation variables           (nu x 1)
  val actuator_ctrllimited  () { return val(typed_memory_view(m->nmesh, m->actuator_ctrllimited   )); } // group for visibility                     (nu x 1)
  val actuator_forcelimited () { return val(typed_memory_view(m->nmesh, m->actuator_forcelimited  )); } // is control limited                       (nu x 1)
  val actuator_actlimited   () { return val(typed_memory_view(m->nmesh, m->actuator_actlimited    )); } // is force limited                         (nu x 1)
  val actuator_dynprm       () { return val(typed_memory_view(m->nmesh, m->actuator_dynprm        )); } // is activation limited                    (nu x 1)
  val actuator_gainprm      () { return val(typed_memory_view(m->nmesh, m->actuator_gainprm       )); } // dynamics parameters                      (nu x mjNDYN)
  val actuator_biasprm      () { return val(typed_memory_view(m->nmesh, m->actuator_biasprm       )); } // gain parameters                          (nu x mjNGAIN)
  val actuator_ctrlrange    () { return val(typed_memory_view(m->nmesh, m->actuator_ctrlrange     )); } // bias parameters                          (nu x mjNBIAS)
  val actuator_forcerange   () { return val(typed_memory_view(m->nmesh, m->actuator_forcerange    )); } // range of controls                        (nu x 2)
  val actuator_actrange     () { return val(typed_memory_view(m->nmesh, m->actuator_actrange      )); } // range of forces                          (nu x 2)
  val actuator_gear         () { return val(typed_memory_view(m->nmesh, m->actuator_gear          )); } // range of activations                     (nu x 2)
  val actuator_cranklength  () { return val(typed_memory_view(m->nmesh, m->actuator_cranklength   )); } // scale length and transmitted force       (nu x 6)
  val actuator_acc0         () { return val(typed_memory_view(m->nmesh, m->actuator_acc0          )); } // crank length for slider-crank            (nu x 1)
  val actuator_length0      () { return val(typed_memory_view(m->nmesh, m->actuator_length0       )); } // acceleration from unit force in qpos0    (nu x 1)
  val actuator_lengthrange  () { return val(typed_memory_view(m->nmesh, m->actuator_lengthrange   )); } // actuator length in qpos0                 (nu x 1)
  val actuator_user         () { return val(typed_memory_view(m->nmesh, m->actuator_user          )); } // feasible actuator length range           (nu x 2)
  val actuator_plugin       () { return val(typed_memory_view(m->nmesh, m->actuator_plugin        )); } // user data                                (nu x nuser_actuator)
  val sensor_type           () { return val(typed_memory_view(m->nmesh, m->sensor_type            )); } // sensors
  val sensor_datatype       () { return val(typed_memory_view(m->nmesh, m->sensor_datatype        )); } // sensor type (mjtSensor)                  (nsensor x 1)
  val sensor_needstage      () { return val(typed_memory_view(m->nmesh, m->sensor_needstage       )); } // numeric data type (mjtDataType)          (nsensor x 1)
  val sensor_objtype        () { return val(typed_memory_view(m->nmesh, m->sensor_objtype         )); } // required compute stage (mjtStage)        (nsensor x 1)
  val sensor_objid          () { return val(typed_memory_view(m->nmesh, m->sensor_objid           )); } // type of sensorized object (mjtObj)       (nsensor x 1)
  val sensor_reftype        () { return val(typed_memory_view(m->nmesh, m->sensor_reftype         )); } // id of sensorized object                  (nsensor x 1)
  val sensor_refid          () { return val(typed_memory_view(m->nmesh, m->sensor_refid           )); } // type of reference frame (mjtObj)         (nsensor x 1)
  val sensor_dim            () { return val(typed_memory_view(m->nmesh, m->sensor_dim             )); } // id of reference frame; -1: global frame  (nsensor x 1)
  val sensor_adr            () { return val(typed_memory_view(m->nmesh, m->sensor_adr             )); } // number of scalar outputs                 (nsensor x 1)
  val sensor_cutoff         () { return val(typed_memory_view(m->nmesh, m->sensor_cutoff          )); } // address in sensor array                  (nsensor x 1)
  val sensor_noise          () { return val(typed_memory_view(m->nmesh, m->sensor_noise           )); } // cutoff for real and positive; 0: ignore  (nsensor x 1)
  val sensor_user           () { return val(typed_memory_view(m->nmesh, m->sensor_user            )); } // noise standard deviation                 (nsensor x 1)
  val sensor_plugin         () { return val(typed_memory_view(m->nmesh, m->sensor_plugin          )); } // user data                                (nsensor x nuser_sensor)
  val plugin                () { return val(typed_memory_view(m->nmesh, m->plugin                 )); } // plugin instances
  val plugin_stateadr       () { return val(typed_memory_view(m->nmesh, m->plugin_stateadr        )); } // globally registered plugin slot number   (nplugin x 1)
  val plugin_statenum       () { return val(typed_memory_view(m->nmesh, m->plugin_statenum        )); } // address in the plugin state array        (nplugin x 1)
  val plugin_attr           () { return val(typed_memory_view(m->nmesh, m->plugin_attr            )); } // number of states in the plugin instance  (nplugin x 1)
  val plugin_attradr        () { return val(typed_memory_view(m->nmesh, m->plugin_attradr         )); } // config attributes of plugin instances    (npluginattr x 1)
  val numeric_adr           () { return val(typed_memory_view(m->nmesh, m->numeric_adr            )); } // custom numeric fields
  val numeric_size          () { return val(typed_memory_view(m->nmesh, m->numeric_size           )); } // address of field in numeric_data         (nnumeric x 1)
  val numeric_data          () { return val(typed_memory_view(m->nmesh, m->numeric_data           )); } // size of numeric field                    (nnumeric x 1)
  val text_adr              () { return val(typed_memory_view(m->nmesh, m->text_adr               )); } // custom text fields
  val text_size             () { return val(typed_memory_view(m->nmesh, m->text_size              )); } // address of text in text_data             (ntext x 1)
  val text_data             () { return val(typed_memory_view(m->nmesh, m->text_data              )); } // size of text field (strlen+1)            (ntext x 1)
  val tuple_adr             () { return val(typed_memory_view(m->nmesh, m->tuple_adr              )); } // custom tuple fields
  val tuple_size            () { return val(typed_memory_view(m->nmesh, m->tuple_size             )); } // address of text in text_data             (ntuple x 1)
  val tuple_objtype         () { return val(typed_memory_view(m->nmesh, m->tuple_objtype          )); } // number of objects in tuple               (ntuple x 1)
  val tuple_objid           () { return val(typed_memory_view(m->nmesh, m->tuple_objid            )); } // array of object types in all tuples      (ntupledata x 1)
  val tuple_objprm          () { return val(typed_memory_view(m->nmesh, m->tuple_objprm           )); } // array of object ids in all tuples        (ntupledata x 1)
  val key_time              () { return val(typed_memory_view(m->nmesh, m->key_time               )); } // keyframes
  val key_qpos              () { return val(typed_memory_view(m->nmesh, m->key_qpos               )); } // key time                                 (nkey x 1)
  val key_qvel              () { return val(typed_memory_view(m->nmesh, m->key_qvel               )); } // key position                             (nkey x nq)
  val key_act               () { return val(typed_memory_view(m->nmesh, m->key_act                )); } // key velocity                             (nkey x nv)
  val key_mpos              () { return val(typed_memory_view(m->nmesh, m->key_mpos               )); } // key activation                           (nkey x na)
  val key_mquat             () { return val(typed_memory_view(m->nmesh, m->key_mquat              )); } // key mocap position                       (nkey x 3*nmocap)
  val key_ctrl              () { return val(typed_memory_view(m->nmesh, m->key_ctrl               )); } // key mocap quaternion                     (nkey x 4*nmocap)
  val name_bodyadr          () { return val(typed_memory_view(m->nmesh, m->name_bodyadr           )); } // names
  val name_jntadr           () { return val(typed_memory_view(m->nmesh, m->name_jntadr            )); } // body name pointers                       (nbody x 1)
  val name_geomadr          () { return val(typed_memory_view(m->nmesh, m->name_geomadr           )); } // joint name pointers                      (njnt x 1)
  val name_siteadr          () { return val(typed_memory_view(m->nmesh, m->name_siteadr           )); } // geom name pointers                       (ngeom x 1)
  val name_camadr           () { return val(typed_memory_view(m->nmesh, m->name_camadr            )); } // site name pointers                       (nsite x 1)
  val name_lightadr         () { return val(typed_memory_view(m->nmesh, m->name_lightadr          )); } // camera name pointers                     (ncam x 1)
  val name_meshadr          () { return val(typed_memory_view(m->nmesh, m->name_meshadr           )); } // light name pointers                      (nlight x 1)
  val name_skinadr          () { return val(typed_memory_view(m->nmesh, m->name_skinadr           )); } // mesh name pointers                       (nmesh x 1)
  val name_hfieldadr        () { return val(typed_memory_view(m->nmesh, m->name_hfieldadr         )); } // skin name pointers                       (nskin x 1)
  val name_texadr           () { return val(typed_memory_view(m->nmesh, m->name_texadr            )); } // hfield name pointers                     (nhfield x 1)
  val name_matadr           () { return val(typed_memory_view(m->nmesh, m->name_matadr            )); } // texture name pointers                    (ntex x 1)
  val name_pairadr          () { return val(typed_memory_view(m->nmesh, m->name_pairadr           )); } // material name pointers                   (nmat x 1)
  val name_excludeadr       () { return val(typed_memory_view(m->nmesh, m->name_excludeadr        )); } // geom pair name pointers                  (npair x 1)
  val name_eqadr            () { return val(typed_memory_view(m->nmesh, m->name_eqadr             )); } // exclude name pointers                    (nexclude x 1)
  val name_tendonadr        () { return val(typed_memory_view(m->nmesh, m->name_tendonadr         )); } // equality constraint name pointers        (neq x 1)
  val name_actuatoradr      () { return val(typed_memory_view(m->nmesh, m->name_actuatoradr       )); } // tendon name pointers                     (ntendon x 1)
  val name_sensoradr        () { return val(typed_memory_view(m->nmesh, m->name_sensoradr         )); } // actuator name pointers                   (nu x 1)
  val name_numericadr       () { return val(typed_memory_view(m->nmesh, m->name_numericadr        )); } // sensor name pointers                     (nsensor x 1)
  val name_textadr          () { return val(typed_memory_view(m->nmesh, m->name_textadr           )); } // numeric name pointers                    (nnumeric x 1)
  val name_tupleadr         () { return val(typed_memory_view(m->nmesh, m->name_tupleadr          )); } // text name pointers                       (ntext x 1)
  val name_keyadr           () { return val(typed_memory_view(m->nmesh, m->name_keyadr            )); } // tuple name pointers                      (ntuple x 1)
  val name_pluginadr        () { return val(typed_memory_view(m->nmesh, m->name_pluginadr         )); } // keyframe name pointers                   (nkey x 1)
  val names                 () { return val(typed_memory_view(m->nmesh, m->names                  )); } // plugin instance name pointers            (nplugin x 1)
private:
  mjModel *m;
};


EMSCRIPTEN_BINDINGS(mujoco_wasm) {

  class_<LROpt>("LROpt                 ")
      .constructor<>()
      .function("mode"                , &LROpt::mode                  )
      .function("useexisting"         , &LROpt::useexisting           )
      .function("uselimit"            , &LROpt::uselimit              )
      .function("accel"               , &LROpt::accel                 )
      .function("maxforce"            , &LROpt::maxforce              )
      .function("timeconst"           , &LROpt::timeconst             )
      .function("timestep"            , &LROpt::timestep              )
      .function("inttotal"            , &LROpt::inttotal              )
      .function("inteval"             , &LROpt::inteval               )
      .function("tolrange"            , &LROpt::tolrange              )
;

  class_<VFS>("VFS                   ")
      .constructor<>()
      .function("nfile"               , &VFS::nfile                 )
      .function("filename"            , &VFS::filename              )
      .function("filesize"            , &VFS::filesize              )
      .function("filedata"            , &VFS::filedata              )
;

  class_<Option>("Option                ")
      .constructor<>()
      .function("timestep"            , &Option::timestep              )
      .function("apirate"             , &Option::apirate               )
      .function("impratio"            , &Option::impratio              )
      .function("tolerance"           , &Option::tolerance             )
      .function("noslip_tolerance"    , &Option::noslip_tolerance      )
      .function("mpr_tolerance"       , &Option::mpr_tolerance         )
      .function("gravity"             , &Option::gravity               )
      .function("wind"                , &Option::wind                  )
      .function("magnetic"            , &Option::magnetic              )
      .function("density"             , &Option::density               )
      .function("viscosity"           , &Option::viscosity             )
      .function("o_margin"            , &Option::o_margin              )
      .function("o_solref"            , &Option::o_solref              )
      .function("o_solimp"            , &Option::o_solimp              )
      .function("integrator"          , &Option::integrator            )
      .function("collision"           , &Option::collision             )
      .function("cone"                , &Option::cone                  )
      .function("jacobian"            , &Option::jacobian              )
      .function("solver"              , &Option::solver                )
      .function("iterations"          , &Option::iterations            )
      .function("noslip_iterations"   , &Option::noslip_iterations     )
      .function("mpr_iterations"      , &Option::mpr_iterations        )
      .function("disableflags"        , &Option::disableflags          )
      .function("enableflags"         , &Option::enableflags           )
;

  class_<Visual>("Visual                ")
      .constructor<>()
;

  class_<Statistic>("Statistic             ")
      .constructor<>()
      .function("meaninertia"         , &Statistic::meaninertia           )
      .function("meanmass"            , &Statistic::meanmass              )
      .function("meansize"            , &Statistic::meansize              )
      .function("extent"              , &Statistic::extent                )
      .function("center"              , &Statistic::center                )
;

  class_<Model>("Model                 ")
      .constructor<>()
      .function("nq"                  , &Model::nq                    )
      .function("nv"                  , &Model::nv                    )
      .function("nu"                  , &Model::nu                    )
      .function("na"                  , &Model::na                    )
      .function("nbody"               , &Model::nbody                 )
      .function("njnt"                , &Model::njnt                  )
      .function("ngeom"               , &Model::ngeom                 )
      .function("nsite"               , &Model::nsite                 )
      .function("ncam"                , &Model::ncam                  )
      .function("nlight"              , &Model::nlight                )
      .function("nmesh"               , &Model::nmesh                 )
      .function("nmeshvert"           , &Model::nmeshvert             )
      .function("nmeshtexvert"        , &Model::nmeshtexvert          )
      .function("nmeshface"           , &Model::nmeshface             )
      .function("nmeshgraph"          , &Model::nmeshgraph            )
      .function("nskin"               , &Model::nskin                 )
      .function("nskinvert"           , &Model::nskinvert             )
      .function("nskintexvert"        , &Model::nskintexvert          )
      .function("nskinface"           , &Model::nskinface             )
      .function("nskinbone"           , &Model::nskinbone             )
      .function("nskinbonevert"       , &Model::nskinbonevert         )
      .function("nhfield"             , &Model::nhfield               )
      .function("nhfielddata"         , &Model::nhfielddata           )
      .function("ntex"                , &Model::ntex                  )
      .function("ntexdata"            , &Model::ntexdata              )
      .function("nmat"                , &Model::nmat                  )
      .function("npair"               , &Model::npair                 )
      .function("nexclude"            , &Model::nexclude              )
      .function("neq"                 , &Model::neq                   )
      .function("ntendon"             , &Model::ntendon               )
      .function("nwrap"               , &Model::nwrap                 )
      .function("nsensor"             , &Model::nsensor               )
      .function("nnumeric"            , &Model::nnumeric              )
      .function("nnumericdata"        , &Model::nnumericdata          )
      .function("ntext"               , &Model::ntext                 )
      .function("ntextdata"           , &Model::ntextdata             )
      .function("ntuple"              , &Model::ntuple                )
      .function("ntupledata"          , &Model::ntupledata            )
      .function("nkey"                , &Model::nkey                  )
      .function("nmocap"              , &Model::nmocap                )
      .function("nplugin"             , &Model::nplugin               )
      .function("npluginattr"         , &Model::npluginattr           )
      .function("nuser_body"          , &Model::nuser_body            )
      .function("nuser_jnt"           , &Model::nuser_jnt             )
      .function("nuser_geom"          , &Model::nuser_geom            )
      .function("nuser_site"          , &Model::nuser_site            )
      .function("nuser_cam"           , &Model::nuser_cam             )
      .function("nuser_tendon"        , &Model::nuser_tendon          )
      .function("nuser_actuator"      , &Model::nuser_actuator        )
      .function("nuser_sensor"        , &Model::nuser_sensor          )
      .function("nnames"              , &Model::nnames                )
      .function("nM"                  , &Model::nM                    )
      .function("nD"                  , &Model::nD                    )
      .function("nemax"               , &Model::nemax                 )
      .function("njmax"               , &Model::njmax                 )
      .function("nconmax"             , &Model::nconmax               )
      .function("nstack"              , &Model::nstack                )
      .function("nuserdata"           , &Model::nuserdata             )
      .function("nsensordata"         , &Model::nsensordata           )
      .function("npluginstate"        , &Model::npluginstate          )
      .function("nbuffer"             , &Model::nbuffer               )
      .function("opt"                 , &Model::opt                   )
      .function("vis"                 , &Model::vis                   )
      .function("stat"                , &Model::stat                  )
      .function("buffer"              , &Model::buffer                )
      .function("qpos0"               , &Model::qpos0                 )
      .function("qpos_spring"         , &Model::qpos_spring           )
      .function("body_parentid"       , &Model::body_parentid         )
      .function("body_rootid"         , &Model::body_rootid           )
      .function("body_weldid"         , &Model::body_weldid           )
      .function("body_mocapid"        , &Model::body_mocapid          )
      .function("body_jntnum"         , &Model::body_jntnum           )
      .function("body_jntadr"         , &Model::body_jntadr           )
      .function("body_dofnum"         , &Model::body_dofnum           )
      .function("body_dofadr"         , &Model::body_dofadr           )
      .function("body_geomnum"        , &Model::body_geomnum          )
      .function("body_geomadr"        , &Model::body_geomadr          )
      .function("body_simple"         , &Model::body_simple           )
      .function("body_sameframe"      , &Model::body_sameframe        )
      .function("body_pos"            , &Model::body_pos              )
      .function("body_quat"           , &Model::body_quat             )
      .function("body_ipos"           , &Model::body_ipos             )
      .function("body_iquat"          , &Model::body_iquat            )
      .function("body_mass"           , &Model::body_mass             )
      .function("body_subtreemass"    , &Model::body_subtreemass      )
      .function("body_inertia"        , &Model::body_inertia          )
      .function("body_invweight0"     , &Model::body_invweight0       )
      .function("body_gravcomp"       , &Model::body_gravcomp         )
      .function("body_user"           , &Model::body_user             )
      .function("body_plugin"         , &Model::body_plugin           )
      .function("jnt_type"            , &Model::jnt_type              )
      .function("jnt_qposadr"         , &Model::jnt_qposadr           )
      .function("jnt_dofadr"          , &Model::jnt_dofadr            )
      .function("jnt_bodyid"          , &Model::jnt_bodyid            )
      .function("jnt_group"           , &Model::jnt_group             )
      .function("jnt_limited"         , &Model::jnt_limited           )
      .function("jnt_solref"          , &Model::jnt_solref            )
      .function("jnt_solimp"          , &Model::jnt_solimp            )
      .function("jnt_pos"             , &Model::jnt_pos               )
      .function("jnt_axis"            , &Model::jnt_axis              )
      .function("jnt_stiffness"       , &Model::jnt_stiffness         )
      .function("jnt_range"           , &Model::jnt_range             )
      .function("jnt_margin"          , &Model::jnt_margin            )
      .function("jnt_user"            , &Model::jnt_user              )
      .function("dof_bodyid"          , &Model::dof_bodyid            )
      .function("dof_jntid"           , &Model::dof_jntid             )
      .function("dof_parentid"        , &Model::dof_parentid          )
      .function("dof_Madr"            , &Model::dof_Madr              )
      .function("dof_simplenum"       , &Model::dof_simplenum         )
      .function("dof_solref"          , &Model::dof_solref            )
      .function("dof_solimp"          , &Model::dof_solimp            )
      .function("dof_frictionloss"    , &Model::dof_frictionloss      )
      .function("dof_armature"        , &Model::dof_armature          )
      .function("dof_damping"         , &Model::dof_damping           )
      .function("dof_invweight0"      , &Model::dof_invweight0        )
      .function("dof_M0"              , &Model::dof_M0                )
      .function("geom_type"           , &Model::geom_type             )
      .function("geom_contype"        , &Model::geom_contype          )
      .function("geom_conaffinity"    , &Model::geom_conaffinity      )
      .function("geom_condim"         , &Model::geom_condim           )
      .function("geom_bodyid"         , &Model::geom_bodyid           )
      .function("geom_dataid"         , &Model::geom_dataid           )
      .function("geom_matid"          , &Model::geom_matid            )
      .function("geom_group"          , &Model::geom_group            )
      .function("geom_priority"       , &Model::geom_priority         )
      .function("geom_sameframe"      , &Model::geom_sameframe        )
      .function("geom_solmix"         , &Model::geom_solmix           )
      .function("geom_solref"         , &Model::geom_solref           )
      .function("geom_solimp"         , &Model::geom_solimp           )
      .function("geom_size"           , &Model::geom_size             )
      .function("geom_rbound"         , &Model::geom_rbound           )
      .function("geom_pos"            , &Model::geom_pos              )
      .function("geom_quat"           , &Model::geom_quat             )
      .function("geom_friction"       , &Model::geom_friction         )
      .function("geom_margin"         , &Model::geom_margin           )
      .function("geom_gap"            , &Model::geom_gap              )
      .function("geom_fluid"          , &Model::geom_fluid            )
      .function("geom_user"           , &Model::geom_user             )
      .function("geom_rgba"           , &Model::geom_rgba             )
      .function("site_type"           , &Model::site_type             )
      .function("site_bodyid"         , &Model::site_bodyid           )
      .function("site_matid"          , &Model::site_matid            )
      .function("site_group"          , &Model::site_group            )
      .function("site_sameframe"      , &Model::site_sameframe        )
      .function("site_size"           , &Model::site_size             )
      .function("site_pos"            , &Model::site_pos              )
      .function("site_quat"           , &Model::site_quat             )
      .function("site_user"           , &Model::site_user             )
      .function("site_rgba"           , &Model::site_rgba             )
      .function("cam_mode"            , &Model::cam_mode              )
      .function("cam_bodyid"          , &Model::cam_bodyid            )
      .function("cam_targetbodyid"    , &Model::cam_targetbodyid      )
      .function("cam_pos"             , &Model::cam_pos               )
      .function("cam_quat"            , &Model::cam_quat              )
      .function("cam_poscom0"         , &Model::cam_poscom0           )
      .function("cam_pos0"            , &Model::cam_pos0              )
      .function("cam_mat0"            , &Model::cam_mat0              )
      .function("cam_fovy"            , &Model::cam_fovy              )
      .function("cam_ipd"             , &Model::cam_ipd               )
      .function("cam_user"            , &Model::cam_user              )
      .function("light_mode"          , &Model::light_mode            )
      .function("light_bodyid"        , &Model::light_bodyid          )
      .function("light_targetbodyid"  , &Model::light_targetbodyid    )
      .function("light_directional"   , &Model::light_directional     )
      .function("light_castshadow"    , &Model::light_castshadow      )
      .function("light_active"        , &Model::light_active          )
      .function("light_pos"           , &Model::light_pos             )
      .function("light_dir"           , &Model::light_dir             )
      .function("light_poscom0"       , &Model::light_poscom0         )
      .function("light_pos0"          , &Model::light_pos0            )
      .function("light_dir0"          , &Model::light_dir0            )
      .function("light_attenuation"   , &Model::light_attenuation     )
      .function("light_cutoff"        , &Model::light_cutoff          )
      .function("light_exponent"      , &Model::light_exponent        )
      .function("light_ambient"       , &Model::light_ambient         )
      .function("light_diffuse"       , &Model::light_diffuse         )
      .function("light_specular"      , &Model::light_specular        )
      .function("mesh_vertadr"        , &Model::mesh_vertadr          )
      .function("mesh_vertnum"        , &Model::mesh_vertnum          )
      .function("mesh_texcoordadr"    , &Model::mesh_texcoordadr      )
      .function("mesh_faceadr"        , &Model::mesh_faceadr          )
      .function("mesh_facenum"        , &Model::mesh_facenum          )
      .function("mesh_graphadr"       , &Model::mesh_graphadr         )
      .function("mesh_vert"           , &Model::mesh_vert             )
      .function("mesh_normal"         , &Model::mesh_normal           )
      .function("mesh_texcoord"       , &Model::mesh_texcoord         )
      .function("mesh_face"           , &Model::mesh_face             )
      .function("mesh_graph"          , &Model::mesh_graph            )
      .function("skin_matid"          , &Model::skin_matid            )
      .function("skin_group"          , &Model::skin_group            )
      .function("skin_rgba"           , &Model::skin_rgba             )
      .function("skin_inflate"        , &Model::skin_inflate          )
      .function("skin_vertadr"        , &Model::skin_vertadr          )
      .function("skin_vertnum"        , &Model::skin_vertnum          )
      .function("skin_texcoordadr"    , &Model::skin_texcoordadr      )
      .function("skin_faceadr"        , &Model::skin_faceadr          )
      .function("skin_facenum"        , &Model::skin_facenum          )
      .function("skin_boneadr"        , &Model::skin_boneadr          )
      .function("skin_bonenum"        , &Model::skin_bonenum          )
      .function("skin_vert"           , &Model::skin_vert             )
      .function("skin_texcoord"       , &Model::skin_texcoord         )
      .function("skin_face"           , &Model::skin_face             )
      .function("skin_bonevertadr"    , &Model::skin_bonevertadr      )
      .function("skin_bonevertnum"    , &Model::skin_bonevertnum      )
      .function("skin_bonebindpos"    , &Model::skin_bonebindpos      )
      .function("skin_bonebindquat"   , &Model::skin_bonebindquat     )
      .function("skin_bonebodyid"     , &Model::skin_bonebodyid       )
      .function("skin_bonevertid"     , &Model::skin_bonevertid       )
      .function("skin_bonevertweight" , &Model::skin_bonevertweight   )
      .function("hfield_size"         , &Model::hfield_size           )
      .function("hfield_nrow"         , &Model::hfield_nrow           )
      .function("hfield_ncol"         , &Model::hfield_ncol           )
      .function("hfield_adr"          , &Model::hfield_adr            )
      .function("hfield_data"         , &Model::hfield_data           )
      .function("tex_type"            , &Model::tex_type              )
      .function("tex_height"          , &Model::tex_height            )
      .function("tex_width"           , &Model::tex_width             )
      .function("tex_adr"             , &Model::tex_adr               )
      .function("tex_rgb"             , &Model::tex_rgb               )
      .function("mat_texid"           , &Model::mat_texid             )
      .function("mat_texuniform"      , &Model::mat_texuniform        )
      .function("mat_texrepeat"       , &Model::mat_texrepeat         )
      .function("mat_emission"        , &Model::mat_emission          )
      .function("mat_specular"        , &Model::mat_specular          )
      .function("mat_shininess"       , &Model::mat_shininess         )
      .function("mat_reflectance"     , &Model::mat_reflectance       )
      .function("mat_rgba"            , &Model::mat_rgba              )
      .function("pair_dim"            , &Model::pair_dim              )
      .function("pair_geom1"          , &Model::pair_geom1            )
      .function("pair_geom2"          , &Model::pair_geom2            )
      .function("pair_signature"      , &Model::pair_signature        )
      .function("pair_solref"         , &Model::pair_solref           )
      .function("pair_solimp"         , &Model::pair_solimp           )
      .function("pair_margin"         , &Model::pair_margin           )
      .function("pair_gap"            , &Model::pair_gap              )
      .function("pair_friction"       , &Model::pair_friction         )
      .function("exclude_signature"   , &Model::exclude_signature     )
      .function("eq_type"             , &Model::eq_type               )
      .function("eq_obj1id"           , &Model::eq_obj1id             )
      .function("eq_obj2id"           , &Model::eq_obj2id             )
      .function("eq_active"           , &Model::eq_active             )
      .function("eq_solref"           , &Model::eq_solref             )
      .function("eq_solimp"           , &Model::eq_solimp             )
      .function("eq_data"             , &Model::eq_data               )
      .function("tendon_adr"          , &Model::tendon_adr            )
      .function("tendon_num"          , &Model::tendon_num            )
      .function("tendon_matid"        , &Model::tendon_matid          )
      .function("tendon_group"        , &Model::tendon_group          )
      .function("tendon_limited"      , &Model::tendon_limited        )
      .function("tendon_width"        , &Model::tendon_width          )
      .function("tendon_solref_lim"   , &Model::tendon_solref_lim     )
      .function("tendon_solimp_lim"   , &Model::tendon_solimp_lim     )
      .function("tendon_solref_fri"   , &Model::tendon_solref_fri     )
      .function("tendon_solimp_fri"   , &Model::tendon_solimp_fri     )
      .function("tendon_range"        , &Model::tendon_range          )
      .function("tendon_margin"       , &Model::tendon_margin         )
      .function("tendon_stiffness"    , &Model::tendon_stiffness      )
      .function("tendon_damping"      , &Model::tendon_damping        )
      .function("tendon_frictionloss" , &Model::tendon_frictionloss   )
      .function("tendon_lengthspring" , &Model::tendon_lengthspring   )
      .function("tendon_length0"      , &Model::tendon_length0        )
      .function("tendon_invweight0"   , &Model::tendon_invweight0     )
      .function("tendon_user"         , &Model::tendon_user           )
      .function("tendon_rgba"         , &Model::tendon_rgba           )
      .function("wrap_type"           , &Model::wrap_type             )
      .function("wrap_objid"          , &Model::wrap_objid            )
      .function("wrap_prm"            , &Model::wrap_prm              )
      .function("actuator_trntype"    , &Model::actuator_trntype      )
      .function("actuator_dyntype"    , &Model::actuator_dyntype      )
      .function("actuator_gaintype"   , &Model::actuator_gaintype     )
      .function("actuator_biastype"   , &Model::actuator_biastype     )
      .function("actuator_trnid"      , &Model::actuator_trnid        )
      .function("actuator_actadr"     , &Model::actuator_actadr       )
      .function("actuator_actnum"     , &Model::actuator_actnum       )
      .function("actuator_group"      , &Model::actuator_group        )
      .function("actuator_ctrllimited", &Model::actuator_ctrllimited  )
      .function("actuator_forcelimited", &Model::actuator_forcelimited )
      .function("actuator_actlimited" , &Model::actuator_actlimited   )
      .function("actuator_dynprm"     , &Model::actuator_dynprm       )
      .function("actuator_gainprm"    , &Model::actuator_gainprm      )
      .function("actuator_biasprm"    , &Model::actuator_biasprm      )
      .function("actuator_ctrlrange"  , &Model::actuator_ctrlrange    )
      .function("actuator_forcerange" , &Model::actuator_forcerange   )
      .function("actuator_actrange"   , &Model::actuator_actrange     )
      .function("actuator_gear"       , &Model::actuator_gear         )
      .function("actuator_cranklength", &Model::actuator_cranklength  )
      .function("actuator_acc0"       , &Model::actuator_acc0         )
      .function("actuator_length0"    , &Model::actuator_length0      )
      .function("actuator_lengthrange", &Model::actuator_lengthrange  )
      .function("actuator_user"       , &Model::actuator_user         )
      .function("actuator_plugin"     , &Model::actuator_plugin       )
      .function("sensor_type"         , &Model::sensor_type           )
      .function("sensor_datatype"     , &Model::sensor_datatype       )
      .function("sensor_needstage"    , &Model::sensor_needstage      )
      .function("sensor_objtype"      , &Model::sensor_objtype        )
      .function("sensor_objid"        , &Model::sensor_objid          )
      .function("sensor_reftype"      , &Model::sensor_reftype        )
      .function("sensor_refid"        , &Model::sensor_refid          )
      .function("sensor_dim"          , &Model::sensor_dim            )
      .function("sensor_adr"          , &Model::sensor_adr            )
      .function("sensor_cutoff"       , &Model::sensor_cutoff         )
      .function("sensor_noise"        , &Model::sensor_noise          )
      .function("sensor_user"         , &Model::sensor_user           )
      .function("sensor_plugin"       , &Model::sensor_plugin         )
      .function("plugin"              , &Model::plugin                )
      .function("plugin_stateadr"     , &Model::plugin_stateadr       )
      .function("plugin_statenum"     , &Model::plugin_statenum       )
      .function("plugin_attr"         , &Model::plugin_attr           )
      .function("plugin_attradr"      , &Model::plugin_attradr        )
      .function("numeric_adr"         , &Model::numeric_adr           )
      .function("numeric_size"        , &Model::numeric_size          )
      .function("numeric_data"        , &Model::numeric_data          )
      .function("text_adr"            , &Model::text_adr              )
      .function("text_size"           , &Model::text_size             )
      .function("text_data"           , &Model::text_data             )
      .function("tuple_adr"           , &Model::tuple_adr             )
      .function("tuple_size"          , &Model::tuple_size            )
      .function("tuple_objtype"       , &Model::tuple_objtype         )
      .function("tuple_objid"         , &Model::tuple_objid           )
      .function("tuple_objprm"        , &Model::tuple_objprm          )
      .function("key_time"            , &Model::key_time              )
      .function("key_qpos"            , &Model::key_qpos              )
      .function("key_qvel"            , &Model::key_qvel              )
      .function("key_act"             , &Model::key_act               )
      .function("key_mpos"            , &Model::key_mpos              )
      .function("key_mquat"           , &Model::key_mquat             )
      .function("key_ctrl"            , &Model::key_ctrl              )
      .function("name_bodyadr"        , &Model::name_bodyadr          )
      .function("name_jntadr"         , &Model::name_jntadr           )
      .function("name_geomadr"        , &Model::name_geomadr          )
      .function("name_siteadr"        , &Model::name_siteadr          )
      .function("name_camadr"         , &Model::name_camadr           )
      .function("name_lightadr"       , &Model::name_lightadr         )
      .function("name_meshadr"        , &Model::name_meshadr          )
      .function("name_skinadr"        , &Model::name_skinadr          )
      .function("name_hfieldadr"      , &Model::name_hfieldadr        )
      .function("name_texadr"         , &Model::name_texadr           )
      .function("name_matadr"         , &Model::name_matadr           )
      .function("name_pairadr"        , &Model::name_pairadr          )
      .function("name_excludeadr"     , &Model::name_excludeadr       )
      .function("name_eqadr"          , &Model::name_eqadr            )
      .function("name_tendonadr"      , &Model::name_tendonadr        )
      .function("name_actuatoradr"    , &Model::name_actuatoradr      )
      .function("name_sensoradr"      , &Model::name_sensoradr        )
      .function("name_numericadr"     , &Model::name_numericadr       )
      .function("name_textadr"        , &Model::name_textadr          )
      .function("name_tupleadr"       , &Model::name_tupleadr         )
      .function("name_keyadr"         , &Model::name_keyadr           )
      .function("name_pluginadr"      , &Model::name_pluginadr        )
      .function("names"               , &Model::names                 )
;

}
