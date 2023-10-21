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

export interface Model {
  new (filename : string) : Model;
  load_from_xml(str: string): Model;
  /** Free the memory associated with the model */
  free(): void;
  /** Retrive various parameters of the current simulation */
  getOptions(): any;
  // MODEL_INTERFACE
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
