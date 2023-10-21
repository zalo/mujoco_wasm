// Type definitions for Emscripten 1.39.16
// Project: https://emscripten.org
// Definitions by: Kensuke Matsuzaki <https://github.com/zakki>
//                 Periklis Tsirakidis <https://github.com/periklis>
//                 Bumsik Kim <https://github.com/kbumsik>
//                 Louis DeScioli <https://github.com/lourd>
// Definitions: https://github.com/DefinitelyTyped/DefinitelyTyped/blob/master/types/emscripten/index.d.ts
// TypeScript Version: 2.2

/** Other WebAssembly declarations, for compatibility with older versions of Typescript */
declare namespace WebAssembly {
    interface Module {}
}

declare namespace Emscripten {
  interface FileSystemType {}
  type EnvironmentType = 'WEB' | 'NODE' | 'SHELL' | 'WORKER';

  type JSType = 'number' | 'string' | 'array' | 'boolean';
  type TypeCompatibleWithC = number | string | any[] | boolean;

  type CIntType = 'i8' | 'i16' | 'i32' | 'i64';
  type CFloatType = 'float' | 'double';
  type CPointerType = 'i8*' | 'i16*' | 'i32*' | 'i64*' | 'float*' | 'double*' | '*';
  type CType = CIntType | CFloatType | CPointerType;

  type WebAssemblyImports = Array<{
      name: string;
      kind: string;
  }>;

  type WebAssemblyExports = Array<{
      module: string;
      name: string;
      kind: string;
  }>;

  interface CCallOpts {
      async?: boolean | undefined;
  }
}

interface EmscriptenModule {
  print(str: string): void;
  printErr(str: string): void;
  arguments: string[];
  environment: Emscripten.EnvironmentType;
  preInit: Array<{ (): void }>;
  preRun: Array<{ (): void }>;
  postRun: Array<{ (): void }>;
  onAbort: { (what: any): void };
  onRuntimeInitialized: { (): void };
  preinitializedWebGLContext: WebGLRenderingContext;
  noInitialRun: boolean;
  noExitRuntime: boolean;
  logReadFiles: boolean;
  filePackagePrefixURL: string;
  wasmBinary: ArrayBuffer;

  destroy(object: object): void;
  getPreloadedPackage(remotePackageName: string, remotePackageSize: number): ArrayBuffer;
  instantiateWasm(
      imports: Emscripten.WebAssemblyImports,
      successCallback: (module: WebAssembly.Module) => void,
  ): Emscripten.WebAssemblyExports;
  locateFile(url: string, scriptDirectory: string): string;
  onCustomMessage(event: MessageEvent): void;

  // USE_TYPED_ARRAYS == 1
  HEAP: Int32Array;
  IHEAP: Int32Array;
  FHEAP: Float64Array;

  // USE_TYPED_ARRAYS == 2
  HEAP8: Int8Array;
  HEAP16: Int16Array;
  HEAP32: Int32Array;
  HEAPU8: Uint8Array;
  HEAPU16: Uint16Array;
  HEAPU32: Uint32Array;
  HEAPF32: Float32Array;
  HEAPF64: Float64Array;

  TOTAL_STACK: number;
  TOTAL_MEMORY: number;
  FAST_MEMORY: number;

  addOnPreRun(cb: () => any): void;
  addOnInit(cb: () => any): void;
  addOnPreMain(cb: () => any): void;
  addOnExit(cb: () => any): void;
  addOnPostRun(cb: () => any): void;

  preloadedImages: any;
  preloadedAudios: any;

  _malloc(size: number): number;
  _free(ptr: number): void;
}

/**
* A factory function is generated when setting the `MODULARIZE` build option
* to `1` in your Emscripten build. It return a Promise that resolves to an
* initialized, ready-to-call `EmscriptenModule` instance.
*
* By default, the factory function will be named `Module`. It's recommended to
* use the `EXPORT_ES6` option, in which the factory function will be the
* default export. If used without `EXPORT_ES6`, the factory function will be a
* global variable. You can rename the variable using the `EXPORT_NAME` build
* option. It's left to you to declare any global variables as needed in your
* application's types.
* @param moduleOverrides Default properties for the initialized module.
*/
type EmscriptenModuleFactory<T extends EmscriptenModule = EmscriptenModule> = (
  moduleOverrides?: Partial<T>,
) => Promise<T>;

declare namespace FS {
  interface Lookup {
      path: string;
      node: FSNode;
  }

  interface FSStream {}
  interface FSNode {}
  interface ErrnoError {}

  let ignorePermissions: boolean;
  let trackingDelegate: any;
  let tracking: any;
  let genericErrors: any;

  //
  // paths
  //
  function lookupPath(path: string, opts: any): Lookup;
  function getPath(node: FSNode): string;

  //
  // nodes
  //
  function isFile(mode: number): boolean;
  function isDir(mode: number): boolean;
  function isLink(mode: number): boolean;
  function isChrdev(mode: number): boolean;
  function isBlkdev(mode: number): boolean;
  function isFIFO(mode: number): boolean;
  function isSocket(mode: number): boolean;

  //
  // devices
  //
  function major(dev: number): number;
  function minor(dev: number): number;
  function makedev(ma: number, mi: number): number;
  function registerDevice(dev: number, ops: any): void;

  //
  // core
  //
  function syncfs(populate: boolean, callback: (e: any) => any): void;
  function syncfs(callback: (e: any) => any, populate?: boolean): void;
  function mount(type: Emscripten.FileSystemType, opts: any, mountpoint: string): any;
  function unmount(mountpoint: string): void;

  function mkdir(path: string, mode?: number): any;
  function mkdev(path: string, mode?: number, dev?: number): any;
  function symlink(oldpath: string, newpath: string): any;
  function rename(old_path: string, new_path: string): void;
  function rmdir(path: string): void;
  function readdir(path: string): any;
  function unlink(path: string): void;
  function readlink(path: string): string;
  function stat(path: string, dontFollow?: boolean): any;
  function lstat(path: string): any;
  function chmod(path: string, mode: number, dontFollow?: boolean): void;
  function lchmod(path: string, mode: number): void;
  function fchmod(fd: number, mode: number): void;
  function chown(path: string, uid: number, gid: number, dontFollow?: boolean): void;
  function lchown(path: string, uid: number, gid: number): void;
  function fchown(fd: number, uid: number, gid: number): void;
  function truncate(path: string, len: number): void;
  function ftruncate(fd: number, len: number): void;
  function utime(path: string, atime: number, mtime: number): void;
  function open(path: string, flags: string, mode?: number, fd_start?: number, fd_end?: number): FSStream;
  function close(stream: FSStream): void;
  function llseek(stream: FSStream, offset: number, whence: number): any;
  function read(stream: FSStream, buffer: ArrayBufferView, offset: number, length: number, position?: number): number;
  function write(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position?: number,
      canOwn?: boolean,
  ): number;
  function allocate(stream: FSStream, offset: number, length: number): void;
  function mmap(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position: number,
      prot: number,
      flags: number,
  ): any;
  function ioctl(stream: FSStream, cmd: any, arg: any): any;
  function readFile(path: string, opts: { encoding: 'binary'; flags?: string | undefined }): Uint8Array;
  function readFile(path: string, opts: { encoding: 'utf8'; flags?: string | undefined }): string;
  function readFile(path: string, opts?: { flags?: string | undefined }): Uint8Array;
  function writeFile(path: string, data: string | ArrayBufferView, opts?: { flags?: string | undefined }): void;

  //
  // module-level FS code
  //
  function cwd(): string;
  function chdir(path: string): void;
  function init(
      input: null | (() => number | null),
      output: null | ((c: number) => any),
      error: null | ((c: number) => any),
  ): void;

  function createLazyFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
  ): FSNode;
  function createPreloadedFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
      onload?: () => void,
      onerror?: () => void,
      dontCreateFile?: boolean,
      canOwn?: boolean,
  ): void;
  function createDataFile(
      parent: string | FSNode,
      name: string,
      data: ArrayBufferView,
      canRead: boolean,
      canWrite: boolean,
      canOwn: boolean,
  ): FSNode;
}

declare var MEMFS: Emscripten.FileSystemType;
declare var NODEFS: Emscripten.FileSystemType;
declare var IDBFS: Emscripten.FileSystemType;

// https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html
type StringToType<R extends any> = R extends Emscripten.JSType
? {
    number: number;
    string: string;
    array: number[] | string[] | boolean[] | Uint8Array | Int8Array;
    boolean: boolean;
    null: null;
  }[R]
: never;

type ArgsToType<T extends Array<Emscripten.JSType | null>> = Extract<
{
  [P in keyof T]: StringToType<T[P]>;
},
any[]
>;

type ReturnToType<R extends Emscripten.JSType | null> = R extends null
? null
: StringToType<Exclude<R, null>>;

// ENUMS
/**  disable default feature bitflags        */
export enum mjtDisableBit {
    /** entire constraint solver                 */
    mjDSBL_CONSTRAINT        ,
    /** equality constraints                     */
    mjDSBL_EQUALITY          ,
    /** joint and tendon frictionloss constraints */
    mjDSBL_FRICTIONLOSS      ,
    /** joint and tendon limit constraints       */
    mjDSBL_LIMIT             ,
    /** contact constraints                      */
    mjDSBL_CONTACT           ,
    /** passive forces                           */
    mjDSBL_PASSIVE           ,
    /** gravitational forces                     */
    mjDSBL_GRAVITY           ,
    /** clamp control to specified range         */
    mjDSBL_CLAMPCTRL         ,
    /** warmstart constraint solver              */
    mjDSBL_WARMSTART         ,
    /** remove collisions with parent body       */
    mjDSBL_FILTERPARENT      ,
    /** apply actuation forces                   */
    mjDSBL_ACTUATION         ,
    /** integrator safety: make ref[0]>=2*timestep */
    mjDSBL_REFSAFE           ,
    /** sensors                                  */
    mjDSBL_SENSOR            ,
    /** number of disable flags                  */
    mjNDISABLE               ,
}
/**  enable optional feature bitflags        */
export enum mjtEnableBit {
    /** override contact parameters              */
    mjENBL_OVERRIDE          ,
    /** energy computation                       */
    mjENBL_ENERGY            ,
    /** record solver statistics                 */
    mjENBL_FWDINV            ,
    /** add noise to sensor data                 */
    mjENBL_SENSORNOISE       ,
    /** multi-point convex collision detection   */
    mjENBL_MULTICCD          ,
    /** number of enable flags                   */
    mjNENABLE                ,
}
/**  type of degree of freedom               */
export enum mjtJoint {
    /** global position and orientation (quat)       (7) */
    mjJNT_FREE               ,
    /** orientation (quat) relative to parent        (4) */
    mjJNT_BALL               ,
    /** sliding distance along body-fixed axis       (1) */
    mjJNT_SLIDE              ,
    /** rotation angle (rad) around body-fixed axis  (1) */
    mjJNT_HINGE              ,
}
/**  type of geometric shape                 */
export enum mjtGeom {
    /** plane                                    */
    mjGEOM_PLANE             ,
    /** height field                             */
    mjGEOM_HFIELD            ,
    /** sphere                                   */
    mjGEOM_SPHERE            ,
    /** capsule                                  */
    mjGEOM_CAPSULE           ,
    /** ellipsoid                                */
    mjGEOM_ELLIPSOID         ,
    /** cylinder                                 */
    mjGEOM_CYLINDER          ,
    /** box                                      */
    mjGEOM_BOX               ,
    /** mesh                                     */
    mjGEOM_MESH              ,
    /** number of regular geom types             */
    mjNGEOMTYPES             ,
    /** arrow                                    */
    mjGEOM_ARROW             ,
    /** arrow without wedges                     */
    mjGEOM_ARROW1            ,
    /** arrow in both directions                 */
    mjGEOM_ARROW2            ,
    /** line                                     */
    mjGEOM_LINE              ,
    /** skin                                     */
    mjGEOM_SKIN              ,
    /** text label                               */
    mjGEOM_LABEL             ,
    /** missing geom type                        */
    mjGEOM_NONE              ,
}
/**  tracking mode for camera and light      */
export enum mjtCamLight {
    /** pos and rot fixed in body                */
    mjCAMLIGHT_FIXED         ,
    /** pos tracks body, rot fixed in global     */
    mjCAMLIGHT_TRACK         ,
    /** pos tracks subtree com, rot fixed in body */
    mjCAMLIGHT_TRACKCOM      ,
    /** pos fixed in body, rot tracks target body */
    mjCAMLIGHT_TARGETBODY    ,
    /** pos fixed in body, rot tracks target subtree com */
    mjCAMLIGHT_TARGETBODYCOM ,
}
/**  type of texture                         */
export enum mjtTexture {
    /** 2d texture, suitable for planes and hfields */
    mjTEXTURE_2D             ,
    /** cube texture, suitable for all other geom types */
    mjTEXTURE_CUBE           ,
    /** cube texture used as skybox              */
    mjTEXTURE_SKYBOX         ,
}
/**  integrator mode                         */
export enum mjtIntegrator {
    /** semi-implicit Euler                      */
    mjINT_EULER              ,
    /** 4th-order Runge Kutta                    */
    mjINT_RK4                ,
    /** implicit in velocity                     */
    mjINT_IMPLICIT           ,
}
/**  collision mode for selecting geom pairs */
export enum mjtCollision {
    /** test precomputed and dynamic pairs       */
    mjCOL_ALL                ,
    /** test predefined pairs only               */
    mjCOL_PAIR               ,
    /** test dynamic pairs only                  */
    mjCOL_DYNAMIC            ,
}
/**  type of friction cone                   */
export enum mjtCone {
    /** pyramidal                                */
    mjCONE_PYRAMIDAL         ,
    /** elliptic                                 */
    mjCONE_ELLIPTIC          ,
}
/**  type of constraint Jacobian             */
export enum mjtJacobian {
    /** dense                                    */
    mjJAC_DENSE              ,
    /** sparse                                   */
    mjJAC_SPARSE             ,
    /** dense if nv<60, sparse otherwise         */
    mjJAC_AUTO               ,
}
/**  constraint solver algorithm             */
export enum mjtSolver {
    /** PGS    (dual)                            */
    mjSOL_PGS                ,
    /** CG     (primal)                          */
    mjSOL_CG                 ,
    /** Newton (primal)                          */
    mjSOL_NEWTON             ,
}
/**  type of equality constraint             */
export enum mjtEq {
    /** connect two bodies at a point (ball joint) */
    mjEQ_CONNECT             ,
    /** fix relative position and orientation of two bodies */
    mjEQ_WELD                ,
    /** couple the values of two scalar joints with cubic */
    mjEQ_JOINT               ,
    /** couple the lengths of two tendons with cubic */
    mjEQ_TENDON              ,
    /** unsupported, will cause an error if used */
    mjEQ_DISTANCE            ,
}
/**  type of tendon wrap object              */
export enum mjtWrap {
    /** null object                              */
    mjWRAP_NONE              ,
    /** constant moment arm                      */
    mjWRAP_JOINT             ,
    /** pulley used to split tendon              */
    mjWRAP_PULLEY            ,
    /** pass through site                        */
    mjWRAP_SITE              ,
    /** wrap around sphere                       */
    mjWRAP_SPHERE            ,
    /** wrap around (infinite) cylinder          */
    mjWRAP_CYLINDER          ,
}
/**  type of actuator transmission           */
export enum mjtTrn {
    /** force on joint                           */
    mjTRN_JOINT              ,
    /** force on joint, expressed in parent frame */
    mjTRN_JOINTINPARENT      ,
    /** force via slider-crank linkage           */
    mjTRN_SLIDERCRANK        ,
    /** force on tendon                          */
    mjTRN_TENDON             ,
    /** force on site                            */
    mjTRN_SITE               ,
    /** adhesion force on a body's geoms         */
    mjTRN_BODY               ,
    /** undefined transmission type              */
    mjTRN_UNDEFINED          ,
}
/**  type of actuator dynamics               */
export enum mjtDyn {
    /** no internal dynamics; ctrl specifies force */
    mjDYN_NONE               ,
    /** integrator: da/dt = u                    */
    mjDYN_INTEGRATOR         ,
    /** linear filter: da/dt = (u-a) / tau       */
    mjDYN_FILTER             ,
    /** piece-wise linear filter with two time constants */
    mjDYN_MUSCLE             ,
    /** user-defined dynamics type               */
    mjDYN_USER               ,
}
/**  type of actuator gain                   */
export enum mjtGain {
    /** fixed gain                               */
    mjGAIN_FIXED             ,
    /** const + kp*length + kv*velocity          */
    mjGAIN_AFFINE            ,
    /** muscle FLV curve computed by mju_muscleGain() */
    mjGAIN_MUSCLE            ,
    /** user-defined gain type                   */
    mjGAIN_USER              ,
}
/**  type of actuator bias                   */
export enum mjtBias {
    /** no bias                                  */
    mjBIAS_NONE              ,
    /** const + kp*length + kv*velocity          */
    mjBIAS_AFFINE            ,
    /** muscle passive force computed by mju_muscleBias() */
    mjBIAS_MUSCLE            ,
    /** user-defined bias type                   */
    mjBIAS_USER              ,
}
/**  type of MujoCo object                   */
export enum mjtObj {
    /** unknown object type                      */
    mjOBJ_UNKNOWN            ,
    /** body                                     */
    mjOBJ_BODY               ,
    /** body, used to access regular frame instead of i-frame */
    mjOBJ_XBODY              ,
    /** joint                                    */
    mjOBJ_JOINT              ,
    /** dof                                      */
    mjOBJ_DOF                ,
    /** geom                                     */
    mjOBJ_GEOM               ,
    /** site                                     */
    mjOBJ_SITE               ,
    /** camera                                   */
    mjOBJ_CAMERA             ,
    /** light                                    */
    mjOBJ_LIGHT              ,
    /** mesh                                     */
    mjOBJ_MESH               ,
    /** skin                                     */
    mjOBJ_SKIN               ,
    /** heightfield                              */
    mjOBJ_HFIELD             ,
    /** texture                                  */
    mjOBJ_TEXTURE            ,
    /** material for rendering                   */
    mjOBJ_MATERIAL           ,
    /** geom pair to include                     */
    mjOBJ_PAIR               ,
    /** body pair to exclude                     */
    mjOBJ_EXCLUDE            ,
    /** equality constraint                      */
    mjOBJ_EQUALITY           ,
    /** tendon                                   */
    mjOBJ_TENDON             ,
    /** actuator                                 */
    mjOBJ_ACTUATOR           ,
    /** sensor                                   */
    mjOBJ_SENSOR             ,
    /** numeric                                  */
    mjOBJ_NUMERIC            ,
    /** text                                     */
    mjOBJ_TEXT               ,
    /** tuple                                    */
    mjOBJ_TUPLE              ,
    /** keyframe                                 */
    mjOBJ_KEY                ,
    /** plugin instance                          */
    mjOBJ_PLUGIN             ,
}
/**  type of constraint                      */
export enum mjtConstraint {
    /** equality constraint                      */
    mjCNSTR_EQUALITY         ,
    /** dof friction                             */
    mjCNSTR_FRICTION_DOF     ,
    /** tendon friction                          */
    mjCNSTR_FRICTION_TENDON  ,
    /** joint limit                              */
    mjCNSTR_LIMIT_JOINT      ,
    /** tendon limit                             */
    mjCNSTR_LIMIT_TENDON     ,
    /** frictionless contact                     */
    mjCNSTR_CONTACT_FRICTIONLESS,
    /** frictional contact, pyramidal friction cone */
    mjCNSTR_CONTACT_PYRAMIDAL,
    /** frictional contact, elliptic friction cone */
    mjCNSTR_CONTACT_ELLIPTIC ,
}
/**  constraint state                        */
export enum mjtConstraintState {
    /** constraint satisfied, zero cost (limit, contact) */
    mjCNSTRSTATE_SATISFIED   ,
    /** quadratic cost (equality, friction, limit, contact) */
    mjCNSTRSTATE_QUADRATIC   ,
    /** linear cost, negative side (friction)    */
    mjCNSTRSTATE_LINEARNEG   ,
    /** linear cost, positive side (friction)    */
    mjCNSTRSTATE_LINEARPOS   ,
    /** squared distance to cone cost (elliptic contact) */
    mjCNSTRSTATE_CONE        ,
}
/**  type of sensor                          */
export enum mjtSensor {
    /** scalar contact normal forces summed over sensor zone */
    mjSENS_TOUCH             ,
    /** 3D linear acceleration, in local frame   */
    mjSENS_ACCELEROMETER     ,
    /** 3D linear velocity, in local frame       */
    mjSENS_VELOCIMETER       ,
    /** 3D angular velocity, in local frame      */
    mjSENS_GYRO              ,
    /** 3D force between site's body and its parent body */
    mjSENS_FORCE             ,
    /** 3D torque between site's body and its parent body */
    mjSENS_TORQUE            ,
    /** 3D magnetometer                          */
    mjSENS_MAGNETOMETER      ,
    /** scalar distance to nearest geom or site along z-axis */
    mjSENS_RANGEFINDER       ,
    /** scalar joint position (hinge and slide only) */
    mjSENS_JOINTPOS          ,
    /** scalar joint velocity (hinge and slide only) */
    mjSENS_JOINTVEL          ,
    /** scalar tendon position                   */
    mjSENS_TENDONPOS         ,
    /** scalar tendon velocity                   */
    mjSENS_TENDONVEL         ,
    /** scalar actuator position                 */
    mjSENS_ACTUATORPOS       ,
    /** scalar actuator velocity                 */
    mjSENS_ACTUATORVEL       ,
    /** scalar actuator force                    */
    mjSENS_ACTUATORFRC       ,
    /** 4D ball joint quaternion                 */
    mjSENS_BALLQUAT          ,
    /** 3D ball joint angular velocity           */
    mjSENS_BALLANGVEL        ,
    /** joint limit distance-margin              */
    mjSENS_JOINTLIMITPOS     ,
    /** joint limit velocity                     */
    mjSENS_JOINTLIMITVEL     ,
    /** joint limit force                        */
    mjSENS_JOINTLIMITFRC     ,
    /** tendon limit distance-margin             */
    mjSENS_TENDONLIMITPOS    ,
    /** tendon limit velocity                    */
    mjSENS_TENDONLIMITVEL    ,
    /** tendon limit force                       */
    mjSENS_TENDONLIMITFRC    ,
    /** 3D position                              */
    mjSENS_FRAMEPOS          ,
    /** 4D unit quaternion orientation           */
    mjSENS_FRAMEQUAT         ,
    /** 3D unit vector: x-axis of object's frame */
    mjSENS_FRAMEXAXIS        ,
    /** 3D unit vector: y-axis of object's frame */
    mjSENS_FRAMEYAXIS        ,
    /** 3D unit vector: z-axis of object's frame */
    mjSENS_FRAMEZAXIS        ,
    /** 3D linear velocity                       */
    mjSENS_FRAMELINVEL       ,
    /** 3D angular velocity                      */
    mjSENS_FRAMEANGVEL       ,
    /** 3D linear acceleration                   */
    mjSENS_FRAMELINACC       ,
    /** 3D angular acceleration                  */
    mjSENS_FRAMEANGACC       ,
    /** 3D center of mass of subtree             */
    mjSENS_SUBTREECOM        ,
    /** 3D linear velocity of subtree            */
    mjSENS_SUBTREELINVEL     ,
    /** 3D angular momentum of subtree           */
    mjSENS_SUBTREEANGMOM     ,
    /** simulation time                          */
    mjSENS_CLOCK             ,
    /** plugin-controlled                        */
    mjSENS_PLUGIN            ,
    /** sensor data provided by mjcb_sensor callback */
    mjSENS_USER              ,
}
/**  computation stage                       */
export enum mjtStage {
    /** no computations                          */
    mjSTAGE_NONE             ,
    /** position-dependent computations          */
    mjSTAGE_POS              ,
    /** velocity-dependent computations          */
    mjSTAGE_VEL              ,
    /** acceleration/force-dependent computations */
    mjSTAGE_ACC              ,
}
/**  data type for sensors                   */
export enum mjtDataType {
    /** real values, no constraints              */
    mjDATATYPE_REAL          ,
    /** positive values; 0 or negative: inactive */
    mjDATATYPE_POSITIVE      ,
    /** 3D unit vector                           */
    mjDATATYPE_AXIS          ,
    /** unit quaternion                          */
    mjDATATYPE_QUATERNION    ,
}
/**  mode for actuator length range computation */
export enum mjtLRMode {
    /** do not process any actuators             */
    mjLRMODE_NONE            ,
    /** process muscle actuators                 */
    mjLRMODE_MUSCLE          ,
    /** process muscle and user actuators        */
    mjLRMODE_MUSCLEUSER      ,
    /** process all actuators                    */
    mjLRMODE_ALL             ,
}

export interface Model {
  new (filename : string) : Model;
  load_from_xml(str: string): Model;
  /** Free the memory associated with the model */
  free(): void;
  /** Retrive various parameters of the current simulation */
  getOptions(): any;
  // MODEL_INTERFACE
  /** number of generalized coordinates = dim(qpos)*/
  nq                    :       number;
  /** number of degrees of freedom = dim(qvel)*/
  nv                    :       number;
  /** number of actuators/controls = dim(ctrl)*/
  nu                    :       number;
  /** number of activation states = dim(act)*/
  na                    :       number;
  /** number of bodies*/
  nbody                 :       number;
  /** number of joints*/
  njnt                  :       number;
  /** number of geoms*/
  ngeom                 :       number;
  /** number of sites*/
  nsite                 :       number;
  /** number of cameras*/
  ncam                  :       number;
  /** number of lights*/
  nlight                :       number;
  /** number of meshes*/
  nmesh                 :       number;
  /** number of vertices in all meshes*/
  nmeshvert             :       number;
  /** number of vertices with texcoords in all meshes*/
  nmeshtexvert          :       number;
  /** number of triangular faces in all meshes*/
  nmeshface             :       number;
  /** number of ints in mesh auxiliary data*/
  nmeshgraph            :       number;
  /** number of skins*/
  nskin                 :       number;
  /** number of vertices in all skins*/
  nskinvert             :       number;
  /** number of vertiex with texcoords in all skins*/
  nskintexvert          :       number;
  /** number of triangular faces in all skins*/
  nskinface             :       number;
  /** number of bones in all skins*/
  nskinbone             :       number;
  /** number of vertices in all skin bones*/
  nskinbonevert         :       number;
  /** number of heightfields*/
  nhfield               :       number;
  /** number of data points in all heightfields*/
  nhfielddata           :       number;
  /** number of textures*/
  ntex                  :       number;
  /** number of bytes in texture rgb data*/
  ntexdata              :       number;
  /** number of materials*/
  nmat                  :       number;
  /** number of predefined geom pairs*/
  npair                 :       number;
  /** number of excluded geom pairs*/
  nexclude              :       number;
  /** number of equality constraints*/
  neq                   :       number;
  /** number of tendons*/
  ntendon               :       number;
  /** number of wrap objects in all tendon paths*/
  nwrap                 :       number;
  /** number of sensors*/
  nsensor               :       number;
  /** number of numeric custom fields*/
  nnumeric              :       number;
  /** number of mjtNums in all numeric fields*/
  nnumericdata          :       number;
  /** number of text custom fields*/
  ntext                 :       number;
  /** number of mjtBytes in all text fields*/
  ntextdata             :       number;
  /** number of tuple custom fields*/
  ntuple                :       number;
  /** number of objects in all tuple fields*/
  ntupledata            :       number;
  /** number of keyframes*/
  nkey                  :       number;
  /** number of mocap bodies*/
  nmocap                :       number;
  /** number of plugin instances*/
  nplugin               :       number;
  /** number of chars in all plugin config attributes*/
  npluginattr           :       number;
  /** number of mjtNums in body_user*/
  nuser_body            :       number;
  /** number of mjtNums in jnt_user*/
  nuser_jnt             :       number;
  /** number of mjtNums in geom_user*/
  nuser_geom            :       number;
  /** number of mjtNums in site_user*/
  nuser_site            :       number;
  /** number of mjtNums in cam_user*/
  nuser_cam             :       number;
  /** number of mjtNums in tendon_user*/
  nuser_tendon          :       number;
  /** number of mjtNums in actuator_user*/
  nuser_actuator        :       number;
  /** number of mjtNums in sensor_user*/
  nuser_sensor          :       number;
  /** number of chars in all names*/
  nnames                :       number;
  /** number of non-zeros in sparse inertia matrix*/
  nM                    :       number;
  /** number of non-zeros in sparse derivative matrix*/
  nD                    :       number;
  /** number of potential equality-constraint rows*/
  nemax                 :       number;
  /** number of available rows in constraint Jacobian*/
  njmax                 :       number;
  /** number of potential contacts in contact list*/
  nconmax               :       number;
  /** number of fields in mjData stack*/
  nstack                :       number;
  /** number of extra fields in mjData*/
  nuserdata             :       number;
  /** number of fields in sensor data vector*/
  nsensordata           :       number;
  /** number of fields in the plugin state vector*/
  npluginstate          :       number;
  /** number of bytes in buffer*/
  nbuffer               :       number;
  /** qpos values at default pose              (nq x 1)*/
  qpos0                 : Float64Array;
  /** reference pose for springs               (nq x 1)*/
  qpos_spring           : Float64Array;
  /** id of body's parent                      (nbody x 1)*/
  body_parentid         :   Int32Array;
  /** id of root above body                    (nbody x 1)*/
  body_rootid           :   Int32Array;
  /** id of body that this body is welded to   (nbody x 1)*/
  body_weldid           :   Int32Array;
  /** id of mocap data; -1: none               (nbody x 1)*/
  body_mocapid          :   Int32Array;
  /** number of joints for this body           (nbody x 1)*/
  body_jntnum           :   Int32Array;
  /** start addr of joints; -1: no joints      (nbody x 1)*/
  body_jntadr           :   Int32Array;
  /** number of motion degrees of freedom      (nbody x 1)*/
  body_dofnum           :   Int32Array;
  /** start addr of dofs; -1: no dofs          (nbody x 1)*/
  body_dofadr           :   Int32Array;
  /** number of geoms                          (nbody x 1)*/
  body_geomnum          :   Int32Array;
  /** start addr of geoms; -1: no geoms        (nbody x 1)*/
  body_geomadr          :   Int32Array;
  /** body is simple (has diagonal M)          (nbody x 1)*/
  body_simple           :   Uint8Array;
  /** inertial frame is same as body frame     (nbody x 1)*/
  body_sameframe        :   Uint8Array;
  /** position offset rel. to parent body      (nbody x 3)*/
  body_pos              : Float64Array;
  /** orientation offset rel. to parent body   (nbody x 4)*/
  body_quat             : Float64Array;
  /** local position of center of mass         (nbody x 3)*/
  body_ipos             : Float64Array;
  /** local orientation of inertia ellipsoid   (nbody x 4)*/
  body_iquat            : Float64Array;
  /** mass                                     (nbody x 1)*/
  body_mass             : Float64Array;
  /** mass of subtree starting at this body    (nbody x 1)*/
  body_subtreemass      : Float64Array;
  /** diagonal inertia in ipos/iquat frame     (nbody x 3)*/
  body_inertia          : Float64Array;
  /** mean inv inert in qpos0 (trn, rot)       (nbody x 2)*/
  body_invweight0       : Float64Array;
  /** antigravity force, units of body weight  (nbody x 1)*/
  body_gravcomp         : Float64Array;
  /** user data                                (nbody x nuser_body)*/
  body_user             : Float64Array;
  /** plugin instance id (-1 if not in use)    (nbody x 1)*/
  body_plugin           :   Int32Array;
  /** type of joint (mjtJoint)                 (njnt x 1)*/
  jnt_type              :   Int32Array;
  /** start addr in 'qpos' for joint's data    (njnt x 1)*/
  jnt_qposadr           :   Int32Array;
  /** start addr in 'qvel' for joint's data    (njnt x 1)*/
  jnt_dofadr            :   Int32Array;
  /** id of joint's body                       (njnt x 1)*/
  jnt_bodyid            :   Int32Array;
  /** group for visibility                     (njnt x 1)*/
  jnt_group             :   Int32Array;
  /** does joint have limits                   (njnt x 1)*/
  jnt_limited           :   Uint8Array;
  /** constraint solver reference: limit       (njnt x mjNREF)*/
  jnt_solref            : Float64Array;
  /** constraint solver impedance: limit       (njnt x mjNIMP)*/
  jnt_solimp            : Float64Array;
  /** local anchor position                    (njnt x 3)*/
  jnt_pos               : Float64Array;
  /** local joint axis                         (njnt x 3)*/
  jnt_axis              : Float64Array;
  /** stiffness coefficient                    (njnt x 1)*/
  jnt_stiffness         : Float64Array;
  /** joint limits                             (njnt x 2)*/
  jnt_range             : Float64Array;
  /** min distance for limit detection         (njnt x 1)*/
  jnt_margin            : Float64Array;
  /** user data                                (njnt x nuser_jnt)*/
  jnt_user              : Float64Array;
  /** id of dof's body                         (nv x 1)*/
  dof_bodyid            :   Int32Array;
  /** id of dof's joint                        (nv x 1)*/
  dof_jntid             :   Int32Array;
  /** id of dof's parent; -1: none             (nv x 1)*/
  dof_parentid          :   Int32Array;
  /** dof address in M-diagonal                (nv x 1)*/
  dof_Madr              :   Int32Array;
  /** number of consecutive simple dofs        (nv x 1)*/
  dof_simplenum         :   Int32Array;
  /** constraint solver reference:frictionloss (nv x mjNREF)*/
  dof_solref            : Float64Array;
  /** constraint solver impedance:frictionloss (nv x mjNIMP)*/
  dof_solimp            : Float64Array;
  /** dof friction loss                        (nv x 1)*/
  dof_frictionloss      : Float64Array;
  /** dof armature inertia/mass                (nv x 1)*/
  dof_armature          : Float64Array;
  /** damping coefficient                      (nv x 1)*/
  dof_damping           : Float64Array;
  /** diag. inverse inertia in qpos0           (nv x 1)*/
  dof_invweight0        : Float64Array;
  /** diag. inertia in qpos0                   (nv x 1)*/
  dof_M0                : Float64Array;
  /** geometric type (mjtGeom)                 (ngeom x 1)*/
  geom_type             :   Int32Array;
  /** geom contact type                        (ngeom x 1)*/
  geom_contype          :   Int32Array;
  /** geom contact affinity                    (ngeom x 1)*/
  geom_conaffinity      :   Int32Array;
  /** contact dimensionality (1, 3, 4, 6)      (ngeom x 1)*/
  geom_condim           :   Int32Array;
  /** id of geom's body                        (ngeom x 1)*/
  geom_bodyid           :   Int32Array;
  /** id of geom's mesh/hfield (-1: none)      (ngeom x 1)*/
  geom_dataid           :   Int32Array;
  /** material id for rendering                (ngeom x 1)*/
  geom_matid            :   Int32Array;
  /** group for visibility                     (ngeom x 1)*/
  geom_group            :   Int32Array;
  /** geom contact priority                    (ngeom x 1)*/
  geom_priority         :   Int32Array;
  /** same as body frame (1) or iframe (2)     (ngeom x 1)*/
  geom_sameframe        :   Uint8Array;
  /** mixing coef for solref/imp in geom pair  (ngeom x 1)*/
  geom_solmix           : Float64Array;
  /** constraint solver reference: contact     (ngeom x mjNREF)*/
  geom_solref           : Float64Array;
  /** constraint solver impedance: contact     (ngeom x mjNIMP)*/
  geom_solimp           : Float64Array;
  /** geom-specific size parameters            (ngeom x 3)*/
  geom_size             : Float64Array;
  /** radius of bounding sphere                (ngeom x 1)*/
  geom_rbound           : Float64Array;
  /** local position offset rel. to body       (ngeom x 3)*/
  geom_pos              : Float64Array;
  /** local orientation offset rel. to body    (ngeom x 4)*/
  geom_quat             : Float64Array;
  /** friction for (slide, spin, roll)         (ngeom x 3)*/
  geom_friction         : Float64Array;
  /** detect contact if dist<margin            (ngeom x 1)*/
  geom_margin           : Float64Array;
  /** include in solver if dist<margin-gap     (ngeom x 1)*/
  geom_gap              : Float64Array;
  /** fluid interaction parameters             (ngeom x mjNFLUID)*/
  geom_fluid            : Float64Array;
  /** user data                                (ngeom x nuser_geom)*/
  geom_user             : Float64Array;
  /** rgba when material is omitted            (ngeom x 4)*/
  geom_rgba             : Float32Array;
  /** geom type for rendering (mjtGeom)        (nsite x 1)*/
  site_type             :   Int32Array;
  /** id of site's body                        (nsite x 1)*/
  site_bodyid           :   Int32Array;
  /** material id for rendering                (nsite x 1)*/
  site_matid            :   Int32Array;
  /** group for visibility                     (nsite x 1)*/
  site_group            :   Int32Array;
  /** same as body frame (1) or iframe (2)     (nsite x 1)*/
  site_sameframe        :   Uint8Array;
  /** geom size for rendering                  (nsite x 3)*/
  site_size             : Float64Array;
  /** local position offset rel. to body       (nsite x 3)*/
  site_pos              : Float64Array;
  /** local orientation offset rel. to body    (nsite x 4)*/
  site_quat             : Float64Array;
  /** user data                                (nsite x nuser_site)*/
  site_user             : Float64Array;
  /** rgba when material is omitted            (nsite x 4)*/
  site_rgba             : Float32Array;
  /** camera tracking mode (mjtCamLight)       (ncam x 1)*/
  cam_mode              :   Int32Array;
  /** id of camera's body                      (ncam x 1)*/
  cam_bodyid            :   Int32Array;
  /** id of targeted body; -1: none            (ncam x 1)*/
  cam_targetbodyid      :   Int32Array;
  /** position rel. to body frame              (ncam x 3)*/
  cam_pos               : Float64Array;
  /** orientation rel. to body frame           (ncam x 4)*/
  cam_quat              : Float64Array;
  /** global position rel. to sub-com in qpos0 (ncam x 3)*/
  cam_poscom0           : Float64Array;
  /** global position rel. to body in qpos0    (ncam x 3)*/
  cam_pos0              : Float64Array;
  /** global orientation in qpos0              (ncam x 9)*/
  cam_mat0              : Float64Array;
  /** y-field of view (deg)                    (ncam x 1)*/
  cam_fovy              : Float64Array;
  /** inter-pupilary distance                  (ncam x 1)*/
  cam_ipd               : Float64Array;
  /** user data                                (ncam x nuser_cam)*/
  cam_user              : Float64Array;
  /** light tracking mode (mjtCamLight)        (nlight x 1)*/
  light_mode            :   Int32Array;
  /** id of light's body                       (nlight x 1)*/
  light_bodyid          :   Int32Array;
  /** id of targeted body; -1: none            (nlight x 1)*/
  light_targetbodyid    :   Int32Array;
  /** directional light                        (nlight x 1)*/
  light_directional     :   Uint8Array;
  /** does light cast shadows                  (nlight x 1)*/
  light_castshadow      :   Uint8Array;
  /** is light on                              (nlight x 1)*/
  light_active          :   Uint8Array;
  /** position rel. to body frame              (nlight x 3)*/
  light_pos             : Float64Array;
  /** direction rel. to body frame             (nlight x 3)*/
  light_dir             : Float64Array;
  /** global position rel. to sub-com in qpos0 (nlight x 3)*/
  light_poscom0         : Float64Array;
  /** global position rel. to body in qpos0    (nlight x 3)*/
  light_pos0            : Float64Array;
  /** global direction in qpos0                (nlight x 3)*/
  light_dir0            : Float64Array;
  /** OpenGL attenuation (quadratic model)     (nlight x 3)*/
  light_attenuation     : Float32Array;
  /** OpenGL cutoff                            (nlight x 1)*/
  light_cutoff          : Float32Array;
  /** OpenGL exponent                          (nlight x 1)*/
  light_exponent        : Float32Array;
  /** ambient rgb (alpha=1)                    (nlight x 3)*/
  light_ambient         : Float32Array;
  /** diffuse rgb (alpha=1)                    (nlight x 3)*/
  light_diffuse         : Float32Array;
  /** specular rgb (alpha=1)                   (nlight x 3)*/
  light_specular        : Float32Array;
  /** first vertex address                     (nmesh x 1)*/
  mesh_vertadr          :   Int32Array;
  /** number of vertices                       (nmesh x 1)*/
  mesh_vertnum          :   Int32Array;
  /** texcoord data address; -1: no texcoord   (nmesh x 1)*/
  mesh_texcoordadr      :   Int32Array;
  /** first face address                       (nmesh x 1)*/
  mesh_faceadr          :   Int32Array;
  /** number of faces                          (nmesh x 1)*/
  mesh_facenum          :   Int32Array;
  /** graph data address; -1: no graph         (nmesh x 1)*/
  mesh_graphadr         :   Int32Array;
  /** vertex positions for all meshes          (nmeshvert x 3)*/
  mesh_vert             : Float32Array;
  /** vertex normals for all meshes            (nmeshvert x 3)*/
  mesh_normal           : Float32Array;
  /** vertex texcoords for all meshes          (nmeshtexvert x 2)*/
  mesh_texcoord         : Float32Array;
  /** triangle face data                       (nmeshface x 3)*/
  mesh_face             :   Int32Array;
  /** convex graph data                        (nmeshgraph x 1)*/
  mesh_graph            :   Int32Array;
  /** skin material id; -1: none               (nskin x 1)*/
  skin_matid            :   Int32Array;
  /** group for visibility                     (nskin x 1)*/
  skin_group            :   Int32Array;
  /** skin rgba                                (nskin x 4)*/
  skin_rgba             : Float32Array;
  /** inflate skin in normal direction         (nskin x 1)*/
  skin_inflate          : Float32Array;
  /** first vertex address                     (nskin x 1)*/
  skin_vertadr          :   Int32Array;
  /** number of vertices                       (nskin x 1)*/
  skin_vertnum          :   Int32Array;
  /** texcoord data address; -1: no texcoord   (nskin x 1)*/
  skin_texcoordadr      :   Int32Array;
  /** first face address                       (nskin x 1)*/
  skin_faceadr          :   Int32Array;
  /** number of faces                          (nskin x 1)*/
  skin_facenum          :   Int32Array;
  /** first bone in skin                       (nskin x 1)*/
  skin_boneadr          :   Int32Array;
  /** number of bones in skin                  (nskin x 1)*/
  skin_bonenum          :   Int32Array;
  /** vertex positions for all skin meshes     (nskinvert x 3)*/
  skin_vert             : Float32Array;
  /** vertex texcoords for all skin meshes     (nskintexvert x 2)*/
  skin_texcoord         : Float32Array;
  /** triangle faces for all skin meshes       (nskinface x 3)*/
  skin_face             :   Int32Array;
  /** first vertex in each bone                (nskinbone x 1)*/
  skin_bonevertadr      :   Int32Array;
  /** number of vertices in each bone          (nskinbone x 1)*/
  skin_bonevertnum      :   Int32Array;
  /** bind pos of each bone                    (nskinbone x 3)*/
  skin_bonebindpos      : Float32Array;
  /** bind quat of each bone                   (nskinbone x 4)*/
  skin_bonebindquat     : Float32Array;
  /** body id of each bone                     (nskinbone x 1)*/
  skin_bonebodyid       :   Int32Array;
  /** mesh ids of vertices in each bone        (nskinbonevert x 1)*/
  skin_bonevertid       :   Int32Array;
  /** weights of vertices in each bone         (nskinbonevert x 1)*/
  skin_bonevertweight   : Float32Array;
  /** (x, y, z_top, z_bottom)                  (nhfield x 4)*/
  hfield_size           : Float64Array;
  /** number of rows in grid                   (nhfield x 1)*/
  hfield_nrow           :   Int32Array;
  /** number of columns in grid                (nhfield x 1)*/
  hfield_ncol           :   Int32Array;
  /** address in hfield_data                   (nhfield x 1)*/
  hfield_adr            :   Int32Array;
  /** elevation data                           (nhfielddata x 1)*/
  hfield_data           : Float32Array;
  /** texture type (mjtTexture)                (ntex x 1)*/
  tex_type              :   Int32Array;
  /** number of rows in texture image          (ntex x 1)*/
  tex_height            :   Int32Array;
  /** number of columns in texture image       (ntex x 1)*/
  tex_width             :   Int32Array;
  /** address in rgb                           (ntex x 1)*/
  tex_adr               :   Int32Array;
  /** rgb (alpha = 1)                          (ntexdata x 1)*/
  tex_rgb               :   Uint8Array;
  /** texture id; -1: none                     (nmat x 1)*/
  mat_texid             :   Int32Array;
  /** make texture cube uniform                (nmat x 1)*/
  mat_texuniform        :   Uint8Array;
  /** texture repetition for 2d mapping        (nmat x 2)*/
  mat_texrepeat         : Float32Array;
  /** emission (x rgb)                         (nmat x 1)*/
  mat_emission          : Float32Array;
  /** specular (x white)                       (nmat x 1)*/
  mat_specular          : Float32Array;
  /** shininess coef                           (nmat x 1)*/
  mat_shininess         : Float32Array;
  /** reflectance (0: disable)                 (nmat x 1)*/
  mat_reflectance       : Float32Array;
  /** rgba                                     (nmat x 4)*/
  mat_rgba              : Float32Array;
  /** contact dimensionality                   (npair x 1)*/
  pair_dim              :   Int32Array;
  /** id of geom1                              (npair x 1)*/
  pair_geom1            :   Int32Array;
  /** id of geom2                              (npair x 1)*/
  pair_geom2            :   Int32Array;
  /** (body1+1)<<16 + body2+1                  (npair x 1)*/
  pair_signature        :   Int32Array;
  /** constraint solver reference: contact     (npair x mjNREF)*/
  pair_solref           : Float64Array;
  /** constraint solver impedance: contact     (npair x mjNIMP)*/
  pair_solimp           : Float64Array;
  /** detect contact if dist<margin            (npair x 1)*/
  pair_margin           : Float64Array;
  /** include in solver if dist<margin-gap     (npair x 1)*/
  pair_gap              : Float64Array;
  /** tangent1, 2, spin, roll1, 2              (npair x 5)*/
  pair_friction         : Float64Array;
  /** (body1+1)<<16 + body2+1                  (nexclude x 1)*/
  exclude_signature     :   Int32Array;
  /** constraint type (mjtEq)                  (neq x 1)*/
  eq_type               :   Int32Array;
  /** id of object 1                           (neq x 1)*/
  eq_obj1id             :   Int32Array;
  /** id of object 2                           (neq x 1)*/
  eq_obj2id             :   Int32Array;
  /** enable/disable constraint                (neq x 1)*/
  eq_active             :   Uint8Array;
  /** constraint solver reference              (neq x mjNREF)*/
  eq_solref             : Float64Array;
  /** constraint solver impedance              (neq x mjNIMP)*/
  eq_solimp             : Float64Array;
  /** numeric data for constraint              (neq x mjNEQDATA)*/
  eq_data               : Float64Array;
  /** address of first object in tendon's path (ntendon x 1)*/
  tendon_adr            :   Int32Array;
  /** number of objects in tendon's path       (ntendon x 1)*/
  tendon_num            :   Int32Array;
  /** material id for rendering                (ntendon x 1)*/
  tendon_matid          :   Int32Array;
  /** group for visibility                     (ntendon x 1)*/
  tendon_group          :   Int32Array;
  /** does tendon have length limits           (ntendon x 1)*/
  tendon_limited        :   Uint8Array;
  /** width for rendering                      (ntendon x 1)*/
  tendon_width          : Float64Array;
  /** constraint solver reference: limit       (ntendon x mjNREF)*/
  tendon_solref_lim     : Float64Array;
  /** constraint solver impedance: limit       (ntendon x mjNIMP)*/
  tendon_solimp_lim     : Float64Array;
  /** constraint solver reference: friction    (ntendon x mjNREF)*/
  tendon_solref_fri     : Float64Array;
  /** constraint solver impedance: friction    (ntendon x mjNIMP)*/
  tendon_solimp_fri     : Float64Array;
  /** tendon length limits                     (ntendon x 2)*/
  tendon_range          : Float64Array;
  /** min distance for limit detection         (ntendon x 1)*/
  tendon_margin         : Float64Array;
  /** stiffness coefficient                    (ntendon x 1)*/
  tendon_stiffness      : Float64Array;
  /** damping coefficient                      (ntendon x 1)*/
  tendon_damping        : Float64Array;
  /** loss due to friction                     (ntendon x 1)*/
  tendon_frictionloss   : Float64Array;
  /** spring resting length range              (ntendon x 2)*/
  tendon_lengthspring   : Float64Array;
  /** tendon length in qpos0                   (ntendon x 1)*/
  tendon_length0        : Float64Array;
  /** inv. weight in qpos0                     (ntendon x 1)*/
  tendon_invweight0     : Float64Array;
  /** user data                                (ntendon x nuser_tendon)*/
  tendon_user           : Float64Array;
  /** rgba when material is omitted            (ntendon x 4)*/
  tendon_rgba           : Float32Array;
  /** wrap object type (mjtWrap)               (nwrap x 1)*/
  wrap_type             :   Int32Array;
  /** object id: geom, site, joint             (nwrap x 1)*/
  wrap_objid            :   Int32Array;
  /** divisor, joint coef, or site id          (nwrap x 1)*/
  wrap_prm              : Float64Array;
  /** transmission type (mjtTrn)               (nu x 1)*/
  actuator_trntype      :   Int32Array;
  /** dynamics type (mjtDyn)                   (nu x 1)*/
  actuator_dyntype      :   Int32Array;
  /** gain type (mjtGain)                      (nu x 1)*/
  actuator_gaintype     :   Int32Array;
  /** bias type (mjtBias)                      (nu x 1)*/
  actuator_biastype     :   Int32Array;
  /** transmission id: joint, tendon, site     (nu x 2)*/
  actuator_trnid        :   Int32Array;
  /** first activation address; -1: stateless  (nu x 1)*/
  actuator_actadr       :   Int32Array;
  /** number of activation variables           (nu x 1)*/
  actuator_actnum       :   Int32Array;
  /** group for visibility                     (nu x 1)*/
  actuator_group        :   Int32Array;
  /** is control limited                       (nu x 1)*/
  actuator_ctrllimited  :   Uint8Array;
  /** is force limited                         (nu x 1)*/
  actuator_forcelimited :   Uint8Array;
  /** is activation limited                    (nu x 1)*/
  actuator_actlimited   :   Uint8Array;
  /** dynamics parameters                      (nu x mjNDYN)*/
  actuator_dynprm       : Float64Array;
  /** gain parameters                          (nu x mjNGAIN)*/
  actuator_gainprm      : Float64Array;
  /** bias parameters                          (nu x mjNBIAS)*/
  actuator_biasprm      : Float64Array;
  /** range of controls                        (nu x 2)*/
  actuator_ctrlrange    : Float64Array;
  /** range of forces                          (nu x 2)*/
  actuator_forcerange   : Float64Array;
  /** range of activations                     (nu x 2)*/
  actuator_actrange     : Float64Array;
  /** scale length and transmitted force       (nu x 6)*/
  actuator_gear         : Float64Array;
  /** crank length for slider-crank            (nu x 1)*/
  actuator_cranklength  : Float64Array;
  /** acceleration from unit force in qpos0    (nu x 1)*/
  actuator_acc0         : Float64Array;
  /** actuator length in qpos0                 (nu x 1)*/
  actuator_length0      : Float64Array;
  /** feasible actuator length range           (nu x 2)*/
  actuator_lengthrange  : Float64Array;
  /** user data                                (nu x nuser_actuator)*/
  actuator_user         : Float64Array;
  /** plugin instance id; -1: not a plugin     (nu x 1)*/
  actuator_plugin       :   Int32Array;
  /** sensor type (mjtSensor)                  (nsensor x 1)*/
  sensor_type           :   Int32Array;
  /** numeric data type (mjtDataType)          (nsensor x 1)*/
  sensor_datatype       :   Int32Array;
  /** required compute stage (mjtStage)        (nsensor x 1)*/
  sensor_needstage      :   Int32Array;
  /** type of sensorized object (mjtObj)       (nsensor x 1)*/
  sensor_objtype        :   Int32Array;
  /** id of sensorized object                  (nsensor x 1)*/
  sensor_objid          :   Int32Array;
  /** type of reference frame (mjtObj)         (nsensor x 1)*/
  sensor_reftype        :   Int32Array;
  /** id of reference frame; -1: global frame  (nsensor x 1)*/
  sensor_refid          :   Int32Array;
  /** number of scalar outputs                 (nsensor x 1)*/
  sensor_dim            :   Int32Array;
  /** address in sensor array                  (nsensor x 1)*/
  sensor_adr            :   Int32Array;
  /** cutoff for real and positive; 0: ignore  (nsensor x 1)*/
  sensor_cutoff         : Float64Array;
  /** noise standard deviation                 (nsensor x 1)*/
  sensor_noise          : Float64Array;
  /** user data                                (nsensor x nuser_sensor)*/
  sensor_user           : Float64Array;
  /** plugin instance id; -1: not a plugin     (nsensor x 1)*/
  sensor_plugin         :   Int32Array;
  /** plugin instance id (-1 if not in use)    (nbody x 1)*/
  plugin                :   Int32Array;
  /** address in the plugin state array        (nplugin x 1)*/
  plugin_stateadr       :   Int32Array;
  /** number of states in the plugin instance  (nplugin x 1)*/
  plugin_statenum       :   Int32Array;
  /** config attributes of plugin instances    (npluginattr x 1)*/
  plugin_attr           :   Uint8Array;
  /** address to each instance's config attrib (nplugin x 1)*/
  plugin_attradr        :   Int32Array;
  /** address of field in numeric_data         (nnumeric x 1)*/
  numeric_adr           :   Int32Array;
  /** size of numeric field                    (nnumeric x 1)*/
  numeric_size          :   Int32Array;
  /** array of all numeric fields              (nnumericdata x 1)*/
  numeric_data          : Float64Array;
  /** address of text in text_data             (ntext x 1)*/
  text_adr              :   Int32Array;
  /** size of text field (strlen+1)            (ntext x 1)*/
  text_size             :   Int32Array;
  /** array of all text fields (0-terminated)  (ntextdata x 1)*/
  text_data             :   Uint8Array;
  /** address of text in text_data             (ntuple x 1)*/
  tuple_adr             :   Int32Array;
  /** number of objects in tuple               (ntuple x 1)*/
  tuple_size            :   Int32Array;
  /** array of object types in all tuples      (ntupledata x 1)*/
  tuple_objtype         :   Int32Array;
  /** array of object ids in all tuples        (ntupledata x 1)*/
  tuple_objid           :   Int32Array;
  /** array of object params in all tuples     (ntupledata x 1)*/
  tuple_objprm          : Float64Array;
  /** key time                                 (nkey x 1)*/
  key_time              : Float64Array;
  /** key position                             (nkey x nq)*/
  key_qpos              : Float64Array;
  /** key velocity                             (nkey x nv)*/
  key_qvel              : Float64Array;
  /** key activation                           (nkey x na)*/
  key_act               : Float64Array;
  /** key mocap position                       (nkey x 3*nmocap)*/
  key_mpos              : Float64Array;
  /** key mocap quaternion                     (nkey x 4*nmocap)*/
  key_mquat             : Float64Array;
  /** key control                              (nkey x nu)*/
  key_ctrl              : Float64Array;
  /** body name pointers                       (nbody x 1)*/
  name_bodyadr          :   Int32Array;
  /** joint name pointers                      (njnt x 1)*/
  name_jntadr           :   Int32Array;
  /** geom name pointers                       (ngeom x 1)*/
  name_geomadr          :   Int32Array;
  /** site name pointers                       (nsite x 1)*/
  name_siteadr          :   Int32Array;
  /** camera name pointers                     (ncam x 1)*/
  name_camadr           :   Int32Array;
  /** light name pointers                      (nlight x 1)*/
  name_lightadr         :   Int32Array;
  /** mesh name pointers                       (nmesh x 1)*/
  name_meshadr          :   Int32Array;
  /** skin name pointers                       (nskin x 1)*/
  name_skinadr          :   Int32Array;
  /** hfield name pointers                     (nhfield x 1)*/
  name_hfieldadr        :   Int32Array;
  /** texture name pointers                    (ntex x 1)*/
  name_texadr           :   Int32Array;
  /** material name pointers                   (nmat x 1)*/
  name_matadr           :   Int32Array;
  /** geom pair name pointers                  (npair x 1)*/
  name_pairadr          :   Int32Array;
  /** exclude name pointers                    (nexclude x 1)*/
  name_excludeadr       :   Int32Array;
  /** equality constraint name pointers        (neq x 1)*/
  name_eqadr            :   Int32Array;
  /** tendon name pointers                     (ntendon x 1)*/
  name_tendonadr        :   Int32Array;
  /** actuator name pointers                   (nu x 1)*/
  name_actuatoradr      :   Int32Array;
  /** sensor name pointers                     (nsensor x 1)*/
  name_sensoradr        :   Int32Array;
  /** numeric name pointers                    (nnumeric x 1)*/
  name_numericadr       :   Int32Array;
  /** text name pointers                       (ntext x 1)*/
  name_textadr          :   Int32Array;
  /** tuple name pointers                      (ntuple x 1)*/
  name_tupleadr         :   Int32Array;
  /** keyframe name pointers                   (nkey x 1)*/
  name_keyadr           :   Int32Array;
  /** plugin instance name pointers            (nplugin x 1)*/
  name_pluginadr        :   Int32Array;
  /** names of all objects, 0-terminated       (nnames x 1)*/
  names                 :   Uint8Array;
}

export interface State {
  new (model : Model) : State;
  /** Free the memory associated with the state */
  free(): void;
}

export interface Simulation {
  new (model : Model, state : State) : Simulation;
  state() : State;
  model() : Model;
  /** Free the memory associated with both the model and the state in the simulation */
  free()  : void;
  /** Apply cartesian force and torque (outside xfrc_applied mechanism) */
  applyForce(fx: number, fy: number, fz: number, tx: number, ty: number, tz: number, px: number, py: number, pz: number, body_id: number): void;
  
  /** sets perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
   * d->qpos written only if flg_paused and subtree root for selected body has free joint */
  applyPose(bodyID: number,
            refPosX : number, refPosY : number, refPosZ : number,
            refQuat1: number, refQuat2: number, refQuat3: number, refQuat4: number,
            flg_paused: number): void;
  // DATA_INTERFACE
  /** position                                 (nq x 1)*/
  qpos                  : Float64Array;
  /** velocity                                 (nv x 1)*/
  qvel                  : Float64Array;
  /** actuator activation                      (na x 1)*/
  act                   : Float64Array;
  /** acceleration used for warmstart          (nv x 1)*/
  qacc_warmstart        : Float64Array;
  /** plugin state                             (npluginstate x 1)*/
  plugin_state          : Float64Array;
  /** control                                  (nu x 1)*/
  ctrl                  : Float64Array;
  /** applied generalized force                (nv x 1)*/
  qfrc_applied          : Float64Array;
  /** applied Cartesian force/torque           (nbody x 6)*/
  xfrc_applied          : Float64Array;
  /** positions of mocap bodies                (nmocap x 3)*/
  mocap_pos             : Float64Array;
  /** orientations of mocap bodies             (nmocap x 4)*/
  mocap_quat            : Float64Array;
  /** acceleration                             (nv x 1)*/
  qacc                  : Float64Array;
  /** time-derivative of actuator activation   (na x 1)*/
  act_dot               : Float64Array;
  /** user data, not touched by engine         (nuserdata x 1)*/
  userdata              : Float64Array;
  /** sensor data array                        (nsensordata x 1)*/
  sensordata            : Float64Array;
  /** copy of m->plugin, required for deletion (nplugin x 1)*/
  plugin                :   Int32Array;
  /** pointer to plugin-managed data structure (nplugin x 1)*/
  plugin_data           : BigUint64Array;
  /** Cartesian position of body frame         (nbody x 3)*/
  xpos                  : Float64Array;
  /** Cartesian orientation of body frame      (nbody x 4)*/
  xquat                 : Float64Array;
  /** Cartesian orientation of body frame      (nbody x 9)*/
  xmat                  : Float64Array;
  /** Cartesian position of body com           (nbody x 3)*/
  xipos                 : Float64Array;
  /** Cartesian orientation of body inertia    (nbody x 9)*/
  ximat                 : Float64Array;
  /** Cartesian position of joint anchor       (njnt x 3)*/
  xanchor               : Float64Array;
  /** Cartesian joint axis                     (njnt x 3)*/
  xaxis                 : Float64Array;
  /** Cartesian geom position                  (ngeom x 3)*/
  geom_xpos             : Float64Array;
  /** Cartesian geom orientation               (ngeom x 9)*/
  geom_xmat             : Float64Array;
  /** Cartesian site position                  (nsite x 3)*/
  site_xpos             : Float64Array;
  /** Cartesian site orientation               (nsite x 9)*/
  site_xmat             : Float64Array;
  /** Cartesian camera position                (ncam x 3)*/
  cam_xpos              : Float64Array;
  /** Cartesian camera orientation             (ncam x 9)*/
  cam_xmat              : Float64Array;
  /** Cartesian light position                 (nlight x 3)*/
  light_xpos            : Float64Array;
  /** Cartesian light direction                (nlight x 3)*/
  light_xdir            : Float64Array;
  /** center of mass of each subtree           (nbody x 3)*/
  subtree_com           : Float64Array;
  /** com-based motion axis of each dof        (nv x 6)*/
  cdof                  : Float64Array;
  /** com-based body inertia and mass          (nbody x 10)*/
  cinert                : Float64Array;
  /** start address of tendon's path           (ntendon x 1)*/
  ten_wrapadr           :   Int32Array;
  /** number of wrap points in path            (ntendon x 1)*/
  ten_wrapnum           :   Int32Array;
  /** number of non-zeros in Jacobian row      (ntendon x 1)*/
  ten_J_rownnz          :   Int32Array;
  /** row start address in colind array        (ntendon x 1)*/
  ten_J_rowadr          :   Int32Array;
  /** column indices in sparse Jacobian        (ntendon x nv)*/
  ten_J_colind          :   Int32Array;
  /** tendon lengths                           (ntendon x 1)*/
  ten_length            : Float64Array;
  /** tendon Jacobian                          (ntendon x nv)*/
  ten_J                 : Float64Array;
  /** geom id; -1: site; -2: pulley            (nwrap*2 x 1)*/
  wrap_obj              :   Int32Array;
  /** Cartesian 3D points in all path          (nwrap*2 x 3)*/
  wrap_xpos             : Float64Array;
  /** actuator lengths                         (nu x 1)*/
  actuator_length       : Float64Array;
  /** actuator moments                         (nu x nv)*/
  actuator_moment       : Float64Array;
  /** com-based composite inertia and mass     (nbody x 10)*/
  crb                   : Float64Array;
  /** total inertia (sparse)                   (nM x 1)*/
  qM                    : Float64Array;
  /** L'*D*L factorization of M (sparse)       (nM x 1)*/
  qLD                   : Float64Array;
  /** 1/diag(D)                                (nv x 1)*/
  qLDiagInv             : Float64Array;
  /** 1/sqrt(diag(D))                          (nv x 1)*/
  qLDiagSqrtInv         : Float64Array;
  /** tendon velocities                        (ntendon x 1)*/
  ten_velocity          : Float64Array;
  /** actuator velocities                      (nu x 1)*/
  actuator_velocity     : Float64Array;
  /** com-based velocity [3D rot; 3D tran]     (nbody x 6)*/
  cvel                  : Float64Array;
  /** time-derivative of cdof                  (nv x 6)*/
  cdof_dot              : Float64Array;
  /** C(qpos,qvel)                             (nv x 1)*/
  qfrc_bias             : Float64Array;
  /** passive force                            (nv x 1)*/
  qfrc_passive          : Float64Array;
  /** linear velocity of subtree com           (nbody x 3)*/
  subtree_linvel        : Float64Array;
  /** angular momentum about subtree com       (nbody x 3)*/
  subtree_angmom        : Float64Array;
  /** L'*D*L factorization of modified M       (nM x 1)*/
  qH                    : Float64Array;
  /** 1/diag(D) of modified M                  (nv x 1)*/
  qHDiagInv             : Float64Array;
  /** non-zeros in each row                    (nv x 1)*/
  D_rownnz              :   Int32Array;
  /** address of each row in D_colind          (nv x 1)*/
  D_rowadr              :   Int32Array;
  /** column indices of non-zeros              (nD x 1)*/
  D_colind              :   Int32Array;
  /** d (passive + actuator - bias) / d qvel   (nD x 1)*/
  qDeriv                : Float64Array;
  /** sparse LU of (qM - dt*qDeriv)            (nD x 1)*/
  qLU                   : Float64Array;
  /** actuator force in actuation space        (nu x 1)*/
  actuator_force        : Float64Array;
  /** actuator force                           (nv x 1)*/
  qfrc_actuator         : Float64Array;
  /** net unconstrained force                  (nv x 1)*/
  qfrc_smooth           : Float64Array;
  /** unconstrained acceleration               (nv x 1)*/
  qacc_smooth           : Float64Array;
  /** constraint force                         (nv x 1)*/
  qfrc_constraint       : Float64Array;
  /** net external force; should equal:        (nv x 1)*/
  qfrc_inverse          : Float64Array;
  /** com-based acceleration                   (nbody x 6)*/
  cacc                  : Float64Array;
  /** com-based interaction force with parent  (nbody x 6)*/
  cfrc_int              : Float64Array;
  /** com-based external force on body         (nbody x 6)*/
  cfrc_ext              : Float64Array;
  /** Free last XML model if loaded. Called internally at each load.*/
  freeLastXML           (): void;
  /** Advance simulation, use control callback to obtain external force and control.*/
  step                  (): void;
  /** Advance simulation in two steps: before external force and control is set by user.*/
  step1                 (): void;
  /** Advance simulation in two steps: after external force and control is set by user.*/
  step2                 (): void;
  /** Forward dynamics: same as mj_step but do not integrate in time.*/
  forward               (): void;
  /** Inverse dynamics: qacc must be set before calling.*/
  inverse               (): void;
  /** Forward dynamics with skip; skipstage is mjtStage.*/
  forwardSkip           (skipstage : number, skipsensor : number): void;
  /** Inverse dynamics with skip; skipstage is mjtStage.*/
  inverseSkip           (skipstage : number, skipsensor : number): void;
  /** Set solver parameters to default values.    [Only works with MuJoCo Allocated Arrays!]*/
  defaultSolRefImp      (solref : Float64Array, solimp : Float64Array): void;
  /** Return size of buffer needed to hold model.*/
  sizeModel             (): number;
  /** Reset data to defaults.*/
  resetData             (): void;
  /** Reset data to defaults, fill everything else with debug_value.*/
  resetDataDebug        (debug_value : string): void;
  /** Reset data, set fields from specified keyframe.*/
  resetDataKeyframe     (key : number): void;
  /** Free memory allocation in mjData.*/
  deleteData            (): void;
  /** Reset all callbacks to NULL pointers (NULL is the default).*/
  resetCallbacks        (): void;
  /** Print mjModel to text file, specifying format. float_format must be a valid printf-style format string for a single float value.*/
  printFormattedModel   (filename : string, float_format : string): void;
  /** Print model to text file.*/
  printModel            (filename : string): void;
  /** Print mjData to text file, specifying format. float_format must be a valid printf-style format string for a single float value*/
  printFormattedData    (filename : string, float_format : string): void;
  /** Print data to text file.*/
  printData             (filename : string): void;
  /** Print matrix to screen.    [Only works with MuJoCo Allocated Arrays!]*/
  _printMat             (mat : Float64Array, nr : number, nc : number): void;
  /** Run position-dependent computations.*/
  fwdPosition           (): void;
  /** Run velocity-dependent computations.*/
  fwdVelocity           (): void;
  /** Compute actuator force qfrc_actuator.*/
  fwdActuation          (): void;
  /** Add up all non-constraint forces, compute qacc_smooth.*/
  fwdAcceleration       (): void;
  /** Run selected constraint solver.*/
  fwdConstraint         (): void;
  /** Euler integrator, semi-implicit in velocity.*/
  Euler                 (): void;
  /** Runge-Kutta explicit order-N integrator.*/
  RungeKutta            (N : number): void;
  /** Run position-dependent computations in inverse dynamics.*/
  invPosition           (): void;
  /** Run velocity-dependent computations in inverse dynamics.*/
  invVelocity           (): void;
  /** Apply the analytical formula for inverse constraint dynamics.*/
  invConstraint         (): void;
  /** Compare forward and inverse dynamics, save results in fwdinv.*/
  compareFwdInv         (): void;
  /** Evaluate position-dependent sensors.*/
  sensorPos             (): void;
  /** Evaluate velocity-dependent sensors.*/
  sensorVel             (): void;
  /** Evaluate acceleration and force-dependent sensors.*/
  sensorAcc             (): void;
  /** Evaluate position-dependent energy (potential).*/
  energyPos             (): void;
  /** Evaluate velocity-dependent energy (kinetic).*/
  energyVel             (): void;
  /** Check qpos, reset if any element is too big or nan.*/
  checkPos              (): void;
  /** Check qvel, reset if any element is too big or nan.*/
  checkVel              (): void;
  /** Check qacc, reset if any element is too big or nan.*/
  checkAcc              (): void;
  /** Run forward kinematics.*/
  kinematics            (): void;
  /** Map inertias and motion dofs to global frame centered at CoM.*/
  comPos                (): void;
  /** Compute camera and light positions and orientations.*/
  camlight              (): void;
  /** Compute tendon lengths, velocities and moment arms.*/
  tendon                (): void;
  /** Compute actuator transmission lengths and moments.*/
  transmission          (): void;
  /** Run composite rigid body inertia algorithm (CRB).*/
  crbCalculate          (): void;
  /** Compute sparse L'*D*L factorizaton of inertia matrix.*/
  factorM               (): void;
  /** Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y    [Only works with MuJoCo Allocated Arrays!]*/
  solveM                (x : Float64Array, y : Float64Array, n : number): void;
  /** Half of linear solve:  x = sqrt(inv(D))*inv(L')*y    [Only works with MuJoCo Allocated Arrays!]*/
  solveM2               (x : Float64Array, y : Float64Array, n : number): void;
  /** Compute cvel, cdof_dot.*/
  comVel                (): void;
  /** Compute qfrc_passive from spring-dampers, viscosity and density.*/
  passive               (): void;
  /** subtree linear velocity and angular momentum*/
  subtreeVel            (): void;
  /** RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.    [Only works with MuJoCo Allocated Arrays!]*/
  rne                   (flg_acc : number, result : Float64Array): void;
  /** RNE with complete data: compute cacc, cfrc_ext, cfrc_int.*/
  rnePostConstraint     (): void;
  /** Run collision detection.*/
  collision             (): void;
  /** Construct constraints.*/
  makeConstraint        (): void;
  /** Compute inverse constraint inertia efc_AR.*/
  projectConstraint     (): void;
  /** Compute efc_vel, efc_aref.*/
  referenceConstraint   (): void;
  /** Determine type of friction cone.*/
  isPyramidal           (): number;
  /** Determine type of constraint Jacobian.*/
  isSparse              (): number;
  /** Determine type of solver (PGS is dual, CG and Newton are primal).*/
  isDual                (): number;
  /** Multiply dense or sparse constraint Jacobian by vector.    [Only works with MuJoCo Allocated Arrays!]*/
  mulJacVec             (res : Float64Array, vec : Float64Array): void;
  /** Multiply dense or sparse constraint Jacobian transpose by vector.    [Only works with MuJoCo Allocated Arrays!]*/
  mulJacTVec            (res : Float64Array, vec : Float64Array): void;
  /** Compute subtree center-of-mass end-effector Jacobian.    [Only works with MuJoCo Allocated Arrays!]*/
  jacSubtreeCom         (jacp : Float64Array, body : number): void;
  /** Get id of object with the specified mjtObj type and name, returns -1 if id not found.*/
  name2id               (type : number, name : string): number;
  /** Get name of object with the specified mjtObj type and id, returns NULL if name not found.*/
  id2name               (type : number, id : number): string;
  /** Convert sparse inertia matrix M into full (i.e. dense) matrix.    [Only works with MuJoCo Allocated Arrays!]*/
  fullM                 (dst : Float64Array, M : Float64Array): void;
  /** Compute velocity by finite-differencing two positions.    [Only works with MuJoCo Allocated Arrays!]*/
  differentiatePos      (qvel : Float64Array, dt : number, qpos1 : Float64Array, qpos2 : Float64Array): void;
  /** Integrate position with given velocity.    [Only works with MuJoCo Allocated Arrays!]*/
  integratePos          (qpos : Float64Array, qvel : Float64Array, dt : number): void;
  /** Normalize all quaternions in qpos-type vector.    [Only works with MuJoCo Allocated Arrays!]*/
  normalizeQuat         (qpos : Float64Array): void;
  /** Sum all body masses.*/
  getTotalmass          (): number;
  /** Return a config attribute value of a plugin instance; NULL: invalid plugin instance ID or attribute name*/
  getPluginConfig       (plugin_id : number, attrib : string): string;
  /** Load a dynamic library. The dynamic library is assumed to register one or more plugins.*/
  loadPluginLibrary     (path : string): void;
  /** Return version number: 1.0.2 is encoded as 102.*/
  version               (): number;
  /** Return the current version of MuJoCo as a null-terminated string.*/
  versionString         (): string;
  /** Draw rectangle.*/
  _rectangle            (viewport : mjrRect, r : number, g : number, b : number, a : number): void;
  /** Call glFinish.*/
  _finish               (): void;
  /** Call glGetError and return result.*/
  _getError             (): number;
  /** Get builtin UI theme spacing (ind: 0-1).*/
  i_themeSpacing        (ind : number): mjuiThemeSpacing;
  /** Get builtin UI theme color (ind: 0-3).*/
  i_themeColor          (ind : number): mjuiThemeColor;
  /** Main error function; does not return to caller.*/
  _error                (msg : string): void;
  /** Deprecated: use mju_error.*/
  _error_i              (msg : string, i : number): void;
  /** Deprecated: use mju_error.*/
  _error_s              (msg : string, text : string): void;
  /** Main warning function; returns to caller.*/
  _warning              (msg : string): void;
  /** Deprecated: use mju_warning.*/
  _warning_i            (msg : string, i : number): void;
  /** Deprecated: use mju_warning.*/
  _warning_s            (msg : string, text : string): void;
  /** Clear user error and memory handlers.*/
  _clearHandlers        (): void;
  /** High-level warning function: count warnings in mjData, print only the first.*/
  warning               (warning : number, info : number): void;
  /** Write [datetime, type: message] to MUJOCO_LOG.TXT.*/
  _writeLog             (type : string, msg : string): void;
  /** Return 1 (for backward compatibility).*/
  activate              (filename : string): number;
  /** Do nothing (for backward compatibility).*/
  deactivate            (): void;
  /** Set res = 0.    [Only works with MuJoCo Allocated Arrays!]*/
  _zero                 (res : Float64Array, n : number): void;
  /** Set res = val.    [Only works with MuJoCo Allocated Arrays!]*/
  _fill                 (res : Float64Array, val : number, n : number): void;
  /** Set res = vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _copy                 (res : Float64Array, data : Float64Array, n : number): void;
  /** Return sum(vec).    [Only works with MuJoCo Allocated Arrays!]*/
  _sum                  (vec : Float64Array, n : number): number;
  /** Return L1 norm: sum(abs(vec)).    [Only works with MuJoCo Allocated Arrays!]*/
  _L1                   (vec : Float64Array, n : number): number;
  /** Set res = vec*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _scl                  (res : Float64Array, vec : Float64Array, scl : number, n : number): void;
  /** Set res = vec1 + vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _add                  (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, n : number): void;
  /** Set res = vec1 - vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _sub                  (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, n : number): void;
  /** Set res = res + vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _addTo                (res : Float64Array, vec : Float64Array, n : number): void;
  /** Set res = res - vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _subFrom              (res : Float64Array, vec : Float64Array, n : number): void;
  /** Set res = res + vec*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _addToScl             (res : Float64Array, vec : Float64Array, scl : number, n : number): void;
  /** Set res = vec1 + vec2*scl.    [Only works with MuJoCo Allocated Arrays!]*/
  _addScl               (res : Float64Array, vec1 : Float64Array, vec2 : Float64Array, scl : number, n : number): void;
  /** Normalize vector, return length before normalization.    [Only works with MuJoCo Allocated Arrays!]*/
  _normalize            (res : Float64Array, n : number): number;
  /** Return vector length (without normalizing vector).    [Only works with MuJoCo Allocated Arrays!]*/
  _norm                 (res : Float64Array, n : number): number;
  /** Return dot-product of vec1 and vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _dot                  (vec1 : Float64Array, vec2 : Float64Array, n : number): number;
  /** Multiply matrix and vector: res = mat * vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatVec            (res : Float64Array, mat : Float64Array, vec : Float64Array, nr : number, nc : number): void;
  /** Multiply transposed matrix and vector: res = mat' * vec.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatTVec           (res : Float64Array, mat : Float64Array, vec : Float64Array, nr : number, nc : number): void;
  /** Multiply square matrix with vectors on both sides: returns vec1' * mat * vec2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulVecMatVec         (vec1 : Float64Array, mat : Float64Array, vec2 : Float64Array, n : number): number;
  /** Transpose matrix: res = mat'.    [Only works with MuJoCo Allocated Arrays!]*/
  _transpose            (res : Float64Array, mat : Float64Array, nr : number, nc : number): void;
  /** Symmetrize square matrix res = (mat + mat')/2.    [Only works with MuJoCo Allocated Arrays!]*/
  _symmetrize           (res : Float64Array, mat : Float64Array, n : number): void;
  /** Set mat to the identity matrix.    [Only works with MuJoCo Allocated Arrays!]*/
  _eye                  (mat : Float64Array, n : number): void;
  /** Multiply matrices: res = mat1 * mat2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatMat            (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, c2 : number): void;
  /** Multiply matrices, second argument transposed: res = mat1 * mat2'.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatMatT           (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, r2 : number): void;
  /** Multiply matrices, first argument transposed: res = mat1' * mat2.    [Only works with MuJoCo Allocated Arrays!]*/
  _mulMatTMat           (res : Float64Array, mat1 : Float64Array, mat2 : Float64Array, r1 : number, c1 : number, c2 : number): void;
  /** Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.    [Only works with MuJoCo Allocated Arrays!]*/
  _sqrMatTD             (res : Float64Array, mat : Float64Array, diag : Float64Array, nr : number, nc : number): void;
  /** Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.    [Only works with MuJoCo Allocated Arrays!]*/
  _cholFactor           (mat : Float64Array, n : number, mindiag : number): number;
  /** Solve mat * res = vec, where mat is Cholesky-factorized    [Only works with MuJoCo Allocated Arrays!]*/
  _cholSolve            (res : Float64Array, mat : Float64Array, vec : Float64Array, n : number): void;
  /** Cholesky rank-one update: L*L' +/- x*x'; return rank.    [Only works with MuJoCo Allocated Arrays!]*/
  _cholUpdate           (mat : Float64Array, x : Float64Array, n : number, flg_plus : number): number;
  /** Convert contact force to pyramid representation.    [Only works with MuJoCo Allocated Arrays!]*/
  _encodePyramid        (pyramid : Float64Array, force : Float64Array, mu : Float64Array, dim : number): void;
  /** Convert pyramid representation to contact force.    [Only works with MuJoCo Allocated Arrays!]*/
  _decodePyramid        (force : Float64Array, pyramid : Float64Array, mu : Float64Array, dim : number): void;
  /** Integrate spring-damper analytically, return pos(dt).*/
  _springDamper         (pos0 : number, vel0 : number, Kp : number, Kv : number, dt : number): number;
  /** Return min(a,b) with single evaluation of a and b.*/
  _min                  (a : number, b : number): number;
  /** Return max(a,b) with single evaluation of a and b.*/
  _max                  (a : number, b : number): number;
  /** Clip x to the range [min, max].*/
  _clip                 (x : number, min : number, max : number): number;
  /** Return sign of x: +1, -1 or 0.*/
  _sign                 (x : number): number;
  /** Round x to nearest integer.*/
  _round                (x : number): number;
  /** Convert type id (mjtObj) to type name.*/
  _type2Str             (type : number): string;
  /** Convert type name to type id (mjtObj).*/
  _str2Type             (str : string): number;
  /** Return human readable number of bytes using standard letter suffix.*/
  _writeNumBytes        (nbytes : number): string;
  /** Construct a warning message given the warning type and info.*/
  _warningText          (warning : number, info : number): string;
  /** Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.*/
  _isBad                (x : number): number;
  /** Return 1 if all elements are 0.    [Only works with MuJoCo Allocated Arrays!]*/
  _isZero               (vec : Float64Array, n : number): number;
  /** Standard normal random number generator (optional second number).    [Only works with MuJoCo Allocated Arrays!]*/
  _standardNormal       (num2 : Float64Array): number;
  /** Insertion sort, resulting list is in increasing order.    [Only works with MuJoCo Allocated Arrays!]*/
  _insertionSort        (list : Float64Array, n : number): void;
  /** Generate Halton sequence.*/
  _Halton               (index : number, base : number): number;
  /** Sigmoid function over 0<=x<=1 constructed from half-quadratics.*/
  _sigmoid              (x : number): number;
  /** Finite differenced transition matrices (control theory notation)   d(x_next) = A*dx + B*du   d(sensor) = C*dx + D*du   required output matrix dimensions:      A: (2*nv+na x 2*nv+na)      B: (2*nv+na x nu)      D: (nsensordata x 2*nv+na)      C: (nsensordata x nu)    [Only works with MuJoCo Allocated Arrays!]*/
  _transitionFD         (eps : number, centered : mjtByte, A : Float64Array, B : Float64Array, C : Float64Array, D : Float64Array): void;
  /** Return the number of globally registered plugins.*/
  _pluginCount          (): number;
}

export interface mujoco extends EmscriptenModule {
  FS    : typeof FS;
  MEMFS : typeof MEMFS;
  Model : Model;
  State : State;
  Simulation : Simulation;
}
declare var load_mujoco: EmscriptenModuleFactory<mujoco>;
export default load_mujoco;
