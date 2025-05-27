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
    /** mid-phase collision filtering            */
    mjDSBL_MIDPHASE          ,
    /** implicit integration of joint damping in Euler integrator */
    mjDSBL_EULERDAMP         ,
    /** automatic reset when numerical issues are detected */
    mjDSBL_AUTORESET         ,
    /** native convex collision detection        */
    mjDSBL_NATIVECCD         ,
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
    /** discrete-time inverse dynamics           */
    mjENBL_INVDISCRETE       ,
    /** multi-point convex collision detection   */
    mjENBL_MULTICCD          ,
    /** constraint island discovery              */
    mjENBL_ISLAND            ,
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
    /** signed distance field                    */
    mjGEOM_SDF               ,
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
    /** box with line edges                      */
    mjGEOM_LINEBOX           ,
    /** flex                                     */
    mjGEOM_FLEX              ,
    /** skin                                     */
    mjGEOM_SKIN              ,
    /** text label                               */
    mjGEOM_LABEL             ,
    /** triangle                                 */
    mjGEOM_TRIANGLE          ,
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
/**  role of texture map in rendering        */
export enum mjtTextureRole {
    /** unspecified                              */
    mjTEXROLE_USER           ,
    /** base color (albedo)                      */
    mjTEXROLE_RGB            ,
    /** ambient occlusion                        */
    mjTEXROLE_OCCLUSION      ,
    /** roughness                                */
    mjTEXROLE_ROUGHNESS      ,
    /** metallic                                 */
    mjTEXROLE_METALLIC       ,
    /** normal (bump) map                        */
    mjTEXROLE_NORMAL         ,
    /** transperancy                             */
    mjTEXROLE_OPACITY        ,
    /** light emission                           */
    mjTEXROLE_EMISSIVE       ,
    /** base color, opacity                      */
    mjTEXROLE_RGBA           ,
    /** occlusion, roughness, metallic           */
    mjTEXROLE_ORM            ,
}
/**  integrator mode                         */
export enum mjtIntegrator {
    /** semi-implicit Euler                      */
    mjINT_EULER              ,
    /** 4th-order Runge Kutta                    */
    mjINT_RK4                ,
    /** implicit in velocity                     */
    mjINT_IMPLICIT           ,
    /** implicit in velocity, no rne derivative  */
    mjINT_IMPLICITFAST       ,
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
    /** fix all edge lengths of a flex           */
    mjEQ_FLEX                ,
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
    /** linear filter: da/dt = (u-a) / tau, with exact integration */
    mjDYN_FILTEREXACT        ,
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
    /** flex                                     */
    mjOBJ_FLEX               ,
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
    /** number of object types                   */
    mjNOBJECT                ,
    /** frame                                    */
    mjOBJ_FRAME              ,
    /** default                                  */
    mjOBJ_DEFAULT            ,
    /** entire model                             */
    mjOBJ_MODEL              ,
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
    /** pixel coordinates of a site in the camera image */
    mjSENS_CAMPROJECTION     ,
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
    /** scalar actuator force, measured at the joint */
    mjSENS_JOINTACTFRC       ,
    /** scalar actuator force, measured at the tendon */
    mjSENS_TENDONACTFRC      ,
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
    /** signed distance between two geoms        */
    mjSENS_GEOMDIST          ,
    /** normal direction between two geoms       */
    mjSENS_GEOMNORMAL        ,
    /** segment between two geoms                */
    mjSENS_GEOMFROMTO        ,
    /** potential energy                         */
    mjSENS_E_POTENTIAL       ,
    /** kinetic energy                           */
    mjSENS_E_KINETIC         ,
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
/**  frame alignment of bodies with their children */
export enum mjtSameFrame {
    /** no alignment                             */
    mjSAMEFRAME_NONE         ,
    /** frame is same as body frame              */
    mjSAMEFRAME_BODY         ,
    /** frame is same as inertial frame          */
    mjSAMEFRAME_INERTIA      ,
    /** frame orientation is same as body orientation */
    mjSAMEFRAME_BODYROT      ,
    /** frame orientation is same as inertia orientation */
    mjSAMEFRAME_INERTIAROT   ,
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
/**  mode for flex selfcollide               */
export enum mjtFlexSelf {
    /** no self-collisions                       */
    mjFLEXSELF_NONE          ,
    /** skip midphase, go directly to narrowphase */
    mjFLEXSELF_NARROW        ,
    /** use BVH in midphase (if midphase enabled) */
    mjFLEXSELF_BVH           ,
    /** use SAP in midphase                      */
    mjFLEXSELF_SAP           ,
    /** choose between BVH and SAP automatically */
    mjFLEXSELF_AUTO          ,
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
  /** number of total bounding volumes in all bodies*/
  nbvh                  :       number;
  /** number of static bounding volumes (aabb stored in mjModel)*/
  nbvhstatic            :       number;
  /** number of dynamic bounding volumes (aabb stored in mjData)*/
  nbvhdynamic           :       number;
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
  /** number of flexes*/
  nflex                 :       number;
  /** number of dofs in all flexes*/
  nflexnode             :       number;
  /** number of vertices in all flexes*/
  nflexvert             :       number;
  /** number of edges in all flexes*/
  nflexedge             :       number;
  /** number of elements in all flexes*/
  nflexelem             :       number;
  /** number of element vertex ids in all flexes*/
  nflexelemdata         :       number;
  /** number of element edge ids in all flexes*/
  nflexelemedge         :       number;
  /** number of shell fragment vertex ids in all flexes*/
  nflexshelldata        :       number;
  /** number of element-vertex pairs in all flexes*/
  nflexevpair           :       number;
  /** number of vertices with texture coordinates*/
  nflextexcoord         :       number;
  /** number of meshes*/
  nmesh                 :       number;
  /** number of vertices in all meshes*/
  nmeshvert             :       number;
  /** number of normals in all meshes*/
  nmeshnormal           :       number;
  /** number of texcoords in all meshes*/
  nmeshtexcoord         :       number;
  /** number of triangular faces in all meshes*/
  nmeshface             :       number;
  /** number of ints in mesh auxiliary data*/
  nmeshgraph            :       number;
  /** number of polygons in all meshes*/
  nmeshpoly             :       number;
  /** number of vertices in all polygons*/
  nmeshpolyvert         :       number;
  /** number of polygons in vertex map*/
  nmeshpolymap          :       number;
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
  /** number of chars in all paths*/
  npaths                :       number;
  /** number of slots in the names hash map*/
  nnames_map            :       number;
  /** number of non-zeros in sparse inertia matrix*/
  nM                    :       number;
  /** number of non-zeros in sparse body-dof matrix*/
  nB                    :       number;
  /** number of non-zeros in sparse reduced dof-dof matrix*/
  nC                    :       number;
  /** number of non-zeros in sparse dof-dof matrix*/
  nD                    :       number;
  /** number of non-zeros in sparse actuator_moment matrix*/
  nJmom                 :       number;
  /** number of kinematic trees under world body*/
  ntree                 :       number;
  /** number of bodies with nonzero gravcomp*/
  ngravcomp             :       number;
  /** number of potential equality-constraint rows*/
  nemax                 :       number;
  /** number of available rows in constraint Jacobian (legacy)*/
  njmax                 :       number;
  /** number of potential contacts in contact list (legacy)*/
  nconmax               :       number;
  /** number of mjtNums reserved for the user*/
  nuserdata             :       number;
  /** number of mjtNums in sensor data vector*/
  nsensordata           :       number;
  /** number of mjtNums in plugin state vector*/
  npluginstate          :       number;
  narena                :       number;
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
  /** id of body's kinematic tree; -1: static  (nbody x 1)*/
  body_treeid           :   Int32Array;
  /** number of geoms                          (nbody x 1)*/
  body_geomnum          :   Int32Array;
  /** start addr of geoms; -1: no geoms        (nbody x 1)*/
  body_geomadr          :   Int32Array;
  /** 1: diag M; 2: diag M, sliders only       (nbody x 1)*/
  body_simple           :   Uint8Array;
  /** same frame as inertia (mjtSameframe)     (nbody x 1)*/
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
  /** MAX over all geom margins                (nbody x 1)*/
  body_margin           : Float64Array;
  /** user data                                (nbody x nuser_body)*/
  body_user             : Float64Array;
  /** plugin instance id; -1: not in use       (nbody x 1)*/
  body_plugin           :   Int32Array;
  /** OR over all geom contypes                (nbody x 1)*/
  body_contype          :   Int32Array;
  /** OR over all geom conaffinities           (nbody x 1)*/
  body_conaffinity      :   Int32Array;
  /** address of bvh root                      (nbody x 1)*/
  body_bvhadr           :   Int32Array;
  /** number of bounding volumes               (nbody x 1)*/
  body_bvhnum           :   Int32Array;
  /** depth in the bounding volume hierarchy   (nbvh x 1)*/
  bvh_depth             :   Int32Array;
  /** left and right children in tree          (nbvh x 2)*/
  bvh_child             :   Int32Array;
  /** geom or elem id of node; -1: non-leaf    (nbvh x 1)*/
  bvh_nodeid            :   Int32Array;
  /** local bounding box (center, size)        (nbvhstatic x 6)*/
  bvh_aabb              : Float64Array;
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
  /** does joint have actuator force limits    (njnt x 1)*/
  jnt_actfrclimited     :   Uint8Array;
  /** is gravcomp force applied via actuators  (njnt x 1)*/
  jnt_actgravcomp       :   Uint8Array;
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
  /** range of total actuator force            (njnt x 2)*/
  jnt_actfrcrange       : Float64Array;
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
  /** id of dof's kinematic tree               (nv x 1)*/
  dof_treeid            :   Int32Array;
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
  /** id of geom's mesh/hfield; -1: none       (ngeom x 1)*/
  geom_dataid           :   Int32Array;
  /** material id for rendering; -1: none      (ngeom x 1)*/
  geom_matid            :   Int32Array;
  /** group for visibility                     (ngeom x 1)*/
  geom_group            :   Int32Array;
  /** geom contact priority                    (ngeom x 1)*/
  geom_priority         :   Int32Array;
  /** plugin instance id; -1: not in use       (ngeom x 1)*/
  geom_plugin           :   Int32Array;
  /** same frame as body (mjtSameframe)        (ngeom x 1)*/
  geom_sameframe        :   Uint8Array;
  /** mixing coef for solref/imp in geom pair  (ngeom x 1)*/
  geom_solmix           : Float64Array;
  /** constraint solver reference: contact     (ngeom x mjNREF)*/
  geom_solref           : Float64Array;
  /** constraint solver impedance: contact     (ngeom x mjNIMP)*/
  geom_solimp           : Float64Array;
  /** geom-specific size parameters            (ngeom x 3)*/
  geom_size             : Float64Array;
  /** bounding box, (center, size)             (ngeom x 6)*/
  geom_aabb             : Float64Array;
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
  /** material id for rendering; -1: none      (nsite x 1)*/
  site_matid            :   Int32Array;
  /** group for visibility                     (nsite x 1)*/
  site_group            :   Int32Array;
  /** same frame as body (mjtSameframe)        (nsite x 1)*/
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
  /** orthographic camera; 0: no, 1: yes       (ncam x 1)*/
  cam_orthographic      :   Int32Array;
  /** y field-of-view (ortho ? len : deg)      (ncam x 1)*/
  cam_fovy              : Float64Array;
  /** inter-pupilary distance                  (ncam x 1)*/
  cam_ipd               : Float64Array;
  /** resolution: pixels [width, height]       (ncam x 2)*/
  cam_resolution        :   Int32Array;
  /** sensor size: length [width, height]      (ncam x 2)*/
  cam_sensorsize        : Float32Array;
  /** [focal length; principal point]          (ncam x 4)*/
  cam_intrinsic         : Float32Array;
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
  /** light radius for soft shadows            (nlight x 1)*/
  light_bulbradius      : Float32Array;
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
  /** flex contact type                        (nflex x 1)*/
  flex_contype          :   Int32Array;
  /** flex contact affinity                    (nflex x 1)*/
  flex_conaffinity      :   Int32Array;
  /** contact dimensionality (1, 3, 4, 6)      (nflex x 1)*/
  flex_condim           :   Int32Array;
  /** flex contact priority                    (nflex x 1)*/
  flex_priority         :   Int32Array;
  /** mix coef for solref/imp in contact pair  (nflex x 1)*/
  flex_solmix           : Float64Array;
  /** constraint solver reference: contact     (nflex x mjNREF)*/
  flex_solref           : Float64Array;
  /** constraint solver impedance: contact     (nflex x mjNIMP)*/
  flex_solimp           : Float64Array;
  /** friction for (slide, spin, roll)         (nflex x 3)*/
  flex_friction         : Float64Array;
  /** detect contact if dist<margin            (nflex x 1)*/
  flex_margin           : Float64Array;
  /** include in solver if dist<margin-gap     (nflex x 1)*/
  flex_gap              : Float64Array;
  /** internal flex collision enabled          (nflex x 1)*/
  flex_internal         :   Uint8Array;
  /** self collision mode (mjtFlexSelf)        (nflex x 1)*/
  flex_selfcollide      :   Int32Array;
  /** number of active element layers, 3D only (nflex x 1)*/
  flex_activelayers     :   Int32Array;
  /** 1: lines, 2: triangles, 3: tetrahedra    (nflex x 1)*/
  flex_dim              :   Int32Array;
  /** material id for rendering                (nflex x 1)*/
  flex_matid            :   Int32Array;
  /** group for visibility                     (nflex x 1)*/
  flex_group            :   Int32Array;
  /** interpolation (0: vertex, 1: nodes)      (nflex x 1)*/
  flex_interp           :   Int32Array;
  /** first node address                       (nflex x 1)*/
  flex_nodeadr          :   Int32Array;
  /** number of nodes                          (nflex x 1)*/
  flex_nodenum          :   Int32Array;
  /** first vertex address                     (nflex x 1)*/
  flex_vertadr          :   Int32Array;
  /** number of vertices                       (nflex x 1)*/
  flex_vertnum          :   Int32Array;
  /** first edge address                       (nflex x 1)*/
  flex_edgeadr          :   Int32Array;
  /** number of edges                          (nflex x 1)*/
  flex_edgenum          :   Int32Array;
  /** first element address                    (nflex x 1)*/
  flex_elemadr          :   Int32Array;
  /** number of elements                       (nflex x 1)*/
  flex_elemnum          :   Int32Array;
  /** first element vertex id address          (nflex x 1)*/
  flex_elemdataadr      :   Int32Array;
  /** first element edge id address            (nflex x 1)*/
  flex_elemedgeadr      :   Int32Array;
  /** number of shells                         (nflex x 1)*/
  flex_shellnum         :   Int32Array;
  /** first shell data address                 (nflex x 1)*/
  flex_shelldataadr     :   Int32Array;
  /** first evpair address                     (nflex x 1)*/
  flex_evpairadr        :   Int32Array;
  /** number of evpairs                        (nflex x 1)*/
  flex_evpairnum        :   Int32Array;
  /** address in flex_texcoord; -1: none       (nflex x 1)*/
  flex_texcoordadr      :   Int32Array;
  /** node body ids                            (nflexnode x 1)*/
  flex_nodebodyid       :   Int32Array;
  /** vertex body ids                          (nflexvert x 1)*/
  flex_vertbodyid       :   Int32Array;
  /** edge vertex ids (2 per edge)             (nflexedge x 2)*/
  flex_edge             :   Int32Array;
  /** element vertex ids (dim+1 per elem)      (nflexelemdata x 1)*/
  flex_elem             :   Int32Array;
  /** element texture coordinates (dim+1)      (nflexelemdata x 1)*/
  flex_elemtexcoord     :   Int32Array;
  /** element edge ids                         (nflexelemedge x 1)*/
  flex_elemedge         :   Int32Array;
  /** element distance from surface, 3D only   (nflexelem x 1)*/
  flex_elemlayer        :   Int32Array;
  /** shell fragment vertex ids (dim per frag) (nflexshelldata x 1)*/
  flex_shell            :   Int32Array;
  /** (element, vertex) collision pairs        (nflexevpair x 2)*/
  flex_evpair           :   Int32Array;
  /** vertex positions in local body frames    (nflexvert x 3)*/
  flex_vert             : Float64Array;
  /** vertex positions in qpos0 on [0, 1]^d    (nflexvert x 3)*/
  flex_vert0            : Float64Array;
  /** node positions in local body frames      (nflexnode x 3)*/
  flex_node             : Float64Array;
  /** Cartesian node positions in qpos0        (nflexnode x 3)*/
  flex_node0            : Float64Array;
  /** edge lengths in qpos0                    (nflexedge x 1)*/
  flexedge_length0      : Float64Array;
  /** edge inv. weight in qpos0                (nflexedge x 1)*/
  flexedge_invweight0   : Float64Array;
  /** radius around primitive element          (nflex x 1)*/
  flex_radius           : Float64Array;
  /** finite element stiffness matrix          (nflexelem x 21)*/
  flex_stiffness        : Float64Array;
  /** Rayleigh's damping coefficient           (nflex x 1)*/
  flex_damping          : Float64Array;
  /** edge stiffness                           (nflex x 1)*/
  flex_edgestiffness    : Float64Array;
  /** edge damping                             (nflex x 1)*/
  flex_edgedamping      : Float64Array;
  /** is edge equality constraint defined      (nflex x 1)*/
  flex_edgeequality     :   Uint8Array;
  /** are all verices in the same body         (nflex x 1)*/
  flex_rigid            :   Uint8Array;
  /** are both edge vertices in same body      (nflexedge x 1)*/
  flexedge_rigid        :   Uint8Array;
  /** are all vertex coordinates (0,0,0)       (nflex x 1)*/
  flex_centered         :   Uint8Array;
  /** render flex skin with flat shading       (nflex x 1)*/
  flex_flatskin         :   Uint8Array;
  /** address of bvh root; -1: no bvh          (nflex x 1)*/
  flex_bvhadr           :   Int32Array;
  /** number of bounding volumes               (nflex x 1)*/
  flex_bvhnum           :   Int32Array;
  /** rgba when material is omitted            (nflex x 4)*/
  flex_rgba             : Float32Array;
  /** vertex texture coordinates               (nflextexcoord x 2)*/
  flex_texcoord         : Float32Array;
  /** first vertex address                     (nmesh x 1)*/
  mesh_vertadr          :   Int32Array;
  /** number of vertices                       (nmesh x 1)*/
  mesh_vertnum          :   Int32Array;
  /** first normal address                     (nmesh x 1)*/
  mesh_normaladr        :   Int32Array;
  /** number of normals                        (nmesh x 1)*/
  mesh_normalnum        :   Int32Array;
  /** texcoord data address; -1: no texcoord   (nmesh x 1)*/
  mesh_texcoordadr      :   Int32Array;
  /** number of texcoord                       (nmesh x 1)*/
  mesh_texcoordnum      :   Int32Array;
  /** first face address                       (nmesh x 1)*/
  mesh_faceadr          :   Int32Array;
  /** number of faces                          (nmesh x 1)*/
  mesh_facenum          :   Int32Array;
  /** address of bvh root                      (nmesh x 1)*/
  mesh_bvhadr           :   Int32Array;
  /** number of bvh                            (nmesh x 1)*/
  mesh_bvhnum           :   Int32Array;
  /** graph data address; -1: no graph         (nmesh x 1)*/
  mesh_graphadr         :   Int32Array;
  /** scaling applied to asset vertices        (nmesh x 3)*/
  mesh_scale            : Float64Array;
  /** translation applied to asset vertices    (nmesh x 3)*/
  mesh_pos              : Float64Array;
  /** rotation applied to asset vertices       (nmesh x 4)*/
  mesh_quat             : Float64Array;
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
  /** position                                         (nq x 1)*/
  qpos                  : Float64Array;
  /** velocity                                         (nv x 1)*/
  qvel                  : Float64Array;
  /** actuator activation                              (na x 1)*/
  act                   : Float64Array;
  /** acceleration used for warmstart                  (nv x 1)*/
  qacc_warmstart        : Float64Array;
  /** plugin state                                     (npluginstate x 1)*/
  plugin_state          : Float64Array;
  /** control                                          (nu x 1)*/
  ctrl                  : Float64Array;
  /** applied generalized force                        (nv x 1)*/
  qfrc_applied          : Float64Array;
  /** applied Cartesian force/torque                   (nbody x 6)*/
  xfrc_applied          : Float64Array;
  /** enable/disable constraints                       (neq x 1)*/
  eq_active             :   Uint8Array;
  /** positions of mocap bodies                        (nmocap x 3)*/
  mocap_pos             : Float64Array;
  /** orientations of mocap bodies                     (nmocap x 4)*/
  mocap_quat            : Float64Array;
  /** acceleration                                     (nv x 1)*/
  qacc                  : Float64Array;
  /** time-derivative of actuator activation           (na x 1)*/
  act_dot               : Float64Array;
  /** user data, not touched by engine                 (nuserdata x 1)*/
  userdata              : Float64Array;
  /** sensor data array                                (nsensordata x 1)*/
  sensordata            : Float64Array;
  /** copy of m->plugin, required for deletion         (nplugin x 1)*/
  plugin                :   Int32Array;
  /** pointer to plugin-managed data structure         (nplugin x 1)*/
  plugin_data           : BigUint64Array;
  /** Cartesian position of body frame                 (nbody x 3)*/
  xpos                  : Float64Array;
  /** Cartesian orientation of body frame              (nbody x 4)*/
  xquat                 : Float64Array;
  /** Cartesian orientation of body frame              (nbody x 9)*/
  xmat                  : Float64Array;
  /** Cartesian position of body com                   (nbody x 3)*/
  xipos                 : Float64Array;
  /** Cartesian orientation of body inertia            (nbody x 9)*/
  ximat                 : Float64Array;
  /** Cartesian position of joint anchor               (njnt x 3)*/
  xanchor               : Float64Array;
  /** Cartesian joint axis                             (njnt x 3)*/
  xaxis                 : Float64Array;
  /** Cartesian geom position                          (ngeom x 3)*/
  geom_xpos             : Float64Array;
  /** Cartesian geom orientation                       (ngeom x 9)*/
  geom_xmat             : Float64Array;
  /** Cartesian site position                          (nsite x 3)*/
  site_xpos             : Float64Array;
  /** Cartesian site orientation                       (nsite x 9)*/
  site_xmat             : Float64Array;
  /** Cartesian camera position                        (ncam x 3)*/
  cam_xpos              : Float64Array;
  /** Cartesian camera orientation                     (ncam x 9)*/
  cam_xmat              : Float64Array;
  /** Cartesian light position                         (nlight x 3)*/
  light_xpos            : Float64Array;
  /** Cartesian light direction                        (nlight x 3)*/
  light_xdir            : Float64Array;
  /** center of mass of each subtree                   (nbody x 3)*/
  subtree_com           : Float64Array;
  /** com-based motion axis of each dof (rot:lin)      (nv x 6)*/
  cdof                  : Float64Array;
  /** com-based body inertia and mass                  (nbody x 10)*/
  cinert                : Float64Array;
  /** Cartesian flex vertex positions                  (nflexvert x 3)*/
  flexvert_xpos         : Float64Array;
  /** flex element bounding boxes (center, size)       (nflexelem x 6)*/
  flexelem_aabb         : Float64Array;
  /** number of non-zeros in Jacobian row              (nflexedge x 1)*/
  flexedge_J_rownnz     :   Int32Array;
  /** row start address in colind array                (nflexedge x 1)*/
  flexedge_J_rowadr     :   Int32Array;
  /** column indices in sparse Jacobian                (nflexedge x nv)*/
  flexedge_J_colind     :   Int32Array;
  /** flex edge Jacobian                               (nflexedge x nv)*/
  flexedge_J            : Float64Array;
  /** flex edge lengths                                (nflexedge x 1)*/
  flexedge_length       : Float64Array;
  /** start address of tendon's path                   (ntendon x 1)*/
  ten_wrapadr           :   Int32Array;
  /** number of wrap points in path                    (ntendon x 1)*/
  ten_wrapnum           :   Int32Array;
  /** number of non-zeros in Jacobian row              (ntendon x 1)*/
  ten_J_rownnz          :   Int32Array;
  /** row start address in colind array                (ntendon x 1)*/
  ten_J_rowadr          :   Int32Array;
  /** column indices in sparse Jacobian                (ntendon x nv)*/
  ten_J_colind          :   Int32Array;
  /** tendon lengths                                   (ntendon x 1)*/
  ten_length            : Float64Array;
  /** tendon Jacobian                                  (ntendon x nv)*/
  ten_J                 : Float64Array;
  /** geom id; -1: site; -2: pulley                    (nwrap x 2)*/
  wrap_obj              :   Int32Array;
  /** Cartesian 3D points in all paths                 (nwrap x 6)*/
  wrap_xpos             : Float64Array;
  /** actuator lengths                                 (nu x 1)*/
  actuator_length       : Float64Array;
  /** number of non-zeros in actuator_moment row       (nu x 1)*/
  moment_rownnz         :   Int32Array;
  /** row start address in colind array                (nu x 1)*/
  moment_rowadr         :   Int32Array;
  /** column indices in sparse Jacobian                (nJmom x 1)*/
  moment_colind         :   Int32Array;
  /** actuator moments                                 (nJmom x 1)*/
  actuator_moment       : Float64Array;
  /** com-based composite inertia and mass             (nbody x 10)*/
  crb                   : Float64Array;
  /** total inertia (sparse)                           (nM x 1)*/
  qM                    : Float64Array;
  /** L'*D*L factorization of M (sparse)               (nM x 1)*/
  qLD                   : Float64Array;
  /** 1/diag(D)                                        (nv x 1)*/
  qLDiagInv             : Float64Array;
  /** global bounding box (center, size)               (nbvhdynamic x 6)*/
  bvh_aabb_dyn          : Float64Array;
  /** was bounding volume checked for collision        (nbvh x 1)*/
  bvh_active            :   Uint8Array;
  /** flex edge velocities                             (nflexedge x 1)*/
  flexedge_velocity     : Float64Array;
  /** tendon velocities                                (ntendon x 1)*/
  ten_velocity          : Float64Array;
  /** actuator velocities                              (nu x 1)*/
  actuator_velocity     : Float64Array;
  /** com-based velocity (rot:lin)                     (nbody x 6)*/
  cvel                  : Float64Array;
  /** time-derivative of cdof (rot:lin)                (nv x 6)*/
  cdof_dot              : Float64Array;
  /** C(qpos,qvel)                                     (nv x 1)*/
  qfrc_bias             : Float64Array;
  /** passive spring force                             (nv x 1)*/
  qfrc_spring           : Float64Array;
  /** passive damper force                             (nv x 1)*/
  qfrc_damper           : Float64Array;
  /** passive gravity compensation force               (nv x 1)*/
  qfrc_gravcomp         : Float64Array;
  /** passive fluid force                              (nv x 1)*/
  qfrc_fluid            : Float64Array;
  /** total passive force                              (nv x 1)*/
  qfrc_passive          : Float64Array;
  /** linear velocity of subtree com                   (nbody x 3)*/
  subtree_linvel        : Float64Array;
  /** angular momentum about subtree com               (nbody x 3)*/
  subtree_angmom        : Float64Array;
  /** L'*D*L factorization of modified M               (nM x 1)*/
  qH                    : Float64Array;
  /** 1/diag(D) of modified M                          (nv x 1)*/
  qHDiagInv             : Float64Array;
  /** body-dof: non-zeros in each row                  (nbody x 1)*/
  B_rownnz              :   Int32Array;
  /** body-dof: address of each row in B_colind        (nbody x 1)*/
  B_rowadr              :   Int32Array;
  /** body-dof: column indices of non-zeros            (nB x 1)*/
  B_colind              :   Int32Array;
  /** inertia: non-zeros in each row                   (nv x 1)*/
  M_rownnz              :   Int32Array;
  /** inertia: address of each row in M_colind         (nv x 1)*/
  M_rowadr              :   Int32Array;
  /** inertia: column indices of non-zeros             (nM x 1)*/
  M_colind              :   Int32Array;
  /** index mapping from M (legacy) to M (CSR)         (nM x 1)*/
  mapM2M                :   Int32Array;
  /** reduced dof-dof: non-zeros in each row           (nv x 1)*/
  C_rownnz              :   Int32Array;
  /** reduced dof-dof: address of each row in C_colind (nv x 1)*/
  C_rowadr              :   Int32Array;
  /** reduced dof-dof: column indices of non-zeros     (nC x 1)*/
  C_colind              :   Int32Array;
  /** index mapping from M to C                        (nC x 1)*/
  mapM2C                :   Int32Array;
  /** dof-dof: non-zeros in each row                   (nv x 1)*/
  D_rownnz              :   Int32Array;
  /** dof-dof: address of each row in D_colind         (nv x 1)*/
  D_rowadr              :   Int32Array;
  /** dof-dof: index of diagonal element               (nv x 1)*/
  D_diag                :   Int32Array;
  /** dof-dof: column indices of non-zeros             (nD x 1)*/
  D_colind              :   Int32Array;
  /** index mapping from M to D                        (nD x 1)*/
  mapM2D                :   Int32Array;
  /** index mapping from D to M                        (nM x 1)*/
  mapD2M                :   Int32Array;
  /** d (passive + actuator - bias) / d qvel           (nD x 1)*/
  qDeriv                : Float64Array;
  /** sparse LU of (qM - dt*qDeriv)                    (nD x 1)*/
  qLU                   : Float64Array;
  /** actuator force in actuation space                (nu x 1)*/
  actuator_force        : Float64Array;
  /** actuator force                                   (nv x 1)*/
  qfrc_actuator         : Float64Array;
  /** net unconstrained force                          (nv x 1)*/
  qfrc_smooth           : Float64Array;
  /** unconstrained acceleration                       (nv x 1)*/
  qacc_smooth           : Float64Array;
  /** constraint force                                 (nv x 1)*/
  qfrc_constraint       : Float64Array;
  /** net external force; should equal:*/
  qfrc_inverse          : Float64Array;
  /** com-based acceleration                           (nbody x 6)*/
  cacc                  : Float64Array;
  /** com-based interaction force with parent          (nbody x 6)*/
  cfrc_int              : Float64Array;
  /** com-based external force on body                 (nbody x 6)*/
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
