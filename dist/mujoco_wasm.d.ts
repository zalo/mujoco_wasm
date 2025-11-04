// TypeScript bindings for emscripten-generated code.  Automatically generated at compile time.
declare namespace RuntimeExports {
    /**
     * @param {string|null=} returnType
     * @param {Array=} argTypes
     * @param {Arguments|Array=} args
     * @param {Object=} opts
     */
    function ccall(ident: any, returnType?: (string | null) | undefined, argTypes?: any[] | undefined, args?: (Arguments | any[]) | undefined, opts?: any | undefined): any;
    /**
     * @param {string=} returnType
     * @param {Array=} argTypes
     * @param {Object=} opts
     */
    function cwrap(ident: any, returnType?: string | undefined, argTypes?: any[] | undefined, opts?: any | undefined): (...args: any[]) => any;
    namespace FS {
        export let root: any;
        export let mounts: any[];
        export let devices: {};
        export let streams: any[];
        export let nextInode: number;
        export let nameTable: any;
        export let currentPath: string;
        export let initialized: boolean;
        export let ignorePermissions: boolean;
        export let filesystems: any;
        export let syncFSRequests: number;
        export let readFiles: {};
        export { ErrnoError };
        export { FSStream };
        export { FSNode };
        export function lookupPath(path: any, opts?: {}): {
            path: string;
            node?: undefined;
        } | {
            path: string;
            node: any;
        };
        export function getPath(node: any): any;
        export function hashName(parentid: any, name: any): number;
        export function hashAddNode(node: any): void;
        export function hashRemoveNode(node: any): void;
        export function lookupNode(parent: any, name: any): any;
        export function createNode(parent: any, name: any, mode: any, rdev: any): any;
        export function destroyNode(node: any): void;
        export function isRoot(node: any): boolean;
        export function isMountpoint(node: any): boolean;
        export function isFile(mode: any): boolean;
        export function isDir(mode: any): boolean;
        export function isLink(mode: any): boolean;
        export function isChrdev(mode: any): boolean;
        export function isBlkdev(mode: any): boolean;
        export function isFIFO(mode: any): boolean;
        export function isSocket(mode: any): boolean;
        export function flagsToPermissionString(flag: any): string;
        export function nodePermissions(node: any, perms: any): 0 | 2;
        export function mayLookup(dir: any): any;
        export function mayCreate(dir: any, name: any): any;
        export function mayDelete(dir: any, name: any, isdir: any): any;
        export function mayOpen(node: any, flags: any): any;
        export function checkOpExists(op: any, err: any): any;
        export let MAX_OPEN_FDS: number;
        export function nextfd(): number;
        export function getStreamChecked(fd: any): any;
        export function getStream(fd: any): any;
        export function createStream(stream: any, fd?: number): any;
        export function closeStream(fd: any): void;
        export function dupStream(origStream: any, fd?: number): any;
        export function doSetAttr(stream: any, node: any, attr: any): void;
        export namespace chrdev_stream_ops {
            function open(stream: any): void;
            function llseek(): never;
        }
        export function major(dev: any): number;
        export function minor(dev: any): number;
        export function makedev(ma: any, mi: any): number;
        export function registerDevice(dev: any, ops: any): void;
        export function getDevice(dev: any): any;
        export function getMounts(mount: any): any[];
        export function syncfs(populate: any, callback: any): void;
        export function mount(type: any, opts: any, mountpoint: any): any;
        export function unmount(mountpoint: any): void;
        export function lookup(parent: any, name: any): any;
        export function mknod(path: any, mode: any, dev: any): any;
        export function statfs(path: any): any;
        export function statfsStream(stream: any): any;
        export function statfsNode(node: any): {
            bsize: number;
            frsize: number;
            blocks: number;
            bfree: number;
            bavail: number;
            files: any;
            ffree: number;
            fsid: number;
            flags: number;
            namelen: number;
        };
        export function create(path: any, mode?: number): any;
        export function mkdir(path: any, mode?: number): any;
        export function mkdirTree(path: any, mode: any): void;
        export function mkdev(path: any, mode: any, dev: any): any;
        export function symlink(oldpath: any, newpath: any): any;
        export function rename(old_path: any, new_path: any): void;
        export function rmdir(path: any): void;
        export function readdir(path: any): any;
        export function unlink(path: any): void;
        export function readlink(path: any): any;
        export function stat(path: any, dontFollow: any): any;
        export function fstat(fd: any): any;
        export function lstat(path: any): any;
        export function doChmod(stream: any, node: any, mode: any, dontFollow: any): void;
        export function chmod(path: any, mode: any, dontFollow: any): void;
        export function lchmod(path: any, mode: any): void;
        export function fchmod(fd: any, mode: any): void;
        export function doChown(stream: any, node: any, dontFollow: any): void;
        export function chown(path: any, uid: any, gid: any, dontFollow: any): void;
        export function lchown(path: any, uid: any, gid: any): void;
        export function fchown(fd: any, uid: any, gid: any): void;
        export function doTruncate(stream: any, node: any, len: any): void;
        export function truncate(path: any, len: any): void;
        export function ftruncate(fd: any, len: any): void;
        export function utime(path: any, atime: any, mtime: any): void;
        export function open(path: any, flags: any, mode?: number): any;
        export function close(stream: any): void;
        export function isClosed(stream: any): boolean;
        export function llseek(stream: any, offset: any, whence: any): any;
        export function read(stream: any, buffer: any, offset: any, length: any, position: any): any;
        export function write(stream: any, buffer: any, offset: any, length: any, position: any, canOwn: any): any;
        export function mmap(stream: any, length: any, position: any, prot: any, flags: any): any;
        export function msync(stream: any, buffer: any, offset: any, length: any, mmapFlags: any): any;
        export function ioctl(stream: any, cmd: any, arg: any): any;
        export function readFile(path: any, opts?: {}): Uint8Array<any>;
        export function writeFile(path: any, data: any, opts?: {}): void;
        export function cwd(): any;
        export function chdir(path: any): void;
        export function createDefaultDirectories(): void;
        export function createDefaultDevices(): void;
        export function createSpecialDirectories(): void;
        export function createStandardStreams(input: any, output: any, error: any): void;
        export function staticInit(): void;
        export function init(input: any, output: any, error: any): void;
        export function quit(): void;
        export function findObject(path: any, dontResolveLastLink: any): any;
        export function analyzePath(path: any, dontResolveLastLink: any): {
            isRoot: boolean;
            exists: boolean;
            error: number;
            name: any;
            path: any;
            object: any;
            parentExists: boolean;
            parentPath: any;
            parentObject: any;
        };
        export function createPath(parent: any, path: any, canRead: any, canWrite: any): any;
        export function createFile(parent: any, name: any, properties: any, canRead: any, canWrite: any): any;
        export function createDataFile(parent: any, name: any, data: any, canRead: any, canWrite: any, canOwn: any): void;
        export function createDevice(parent: any, name: any, input: any, output: any): any;
        export function forceLoadFile(obj: any): boolean;
        export function createLazyFile(parent: any, name: any, url: any, canRead: any, canWrite: any): any;
        export function absolutePath(): void;
        export function createFolder(): void;
        export function createLink(): void;
        export function joinPath(): void;
        export function mmapAlloc(): void;
        export function standardizePath(): void;
    }
    namespace MEMFS {
        let ops_table: any;
        function mount(mount: any): any;
        function createNode(parent: any, name: any, mode: any, dev: any): any;
        function getFileDataAsTypedArray(node: any): any;
        function expandFileStorage(node: any, newCapacity: any): void;
        function resizeFileStorage(node: any, newSize: any): void;
        namespace node_ops {
            function getattr(node: any): {
                dev: any;
                ino: any;
                mode: any;
                nlink: number;
                uid: number;
                gid: number;
                rdev: any;
                size: any;
                atime: Date;
                mtime: Date;
                ctime: Date;
                blksize: number;
                blocks: number;
            };
            function setattr(node: any, attr: any): void;
            function lookup(parent: any, name: any): never;
            function mknod(parent: any, name: any, mode: any, dev: any): any;
            function rename(old_node: any, new_dir: any, new_name: any): void;
            function unlink(parent: any, name: any): void;
            function rmdir(parent: any, name: any): void;
            function readdir(node: any): string[];
            function symlink(parent: any, newname: any, oldpath: any): any;
            function readlink(node: any): any;
        }
        namespace stream_ops {
            function read(stream: any, buffer: any, offset: any, length: any, position: any): number;
            function write(stream: any, buffer: any, offset: any, length: any, position: any, canOwn: any): any;
            function llseek(stream: any, offset: any, whence: any): any;
            function mmap(stream: any, length: any, position: any, prot: any, flags: any): {
                ptr: any;
                allocated: boolean;
            };
            function msync(stream: any, buffer: any, offset: any, length: any, mmapFlags: any): number;
        }
    }
    function FS_createPath(...args: any[]): any;
    function FS_createDataFile(...args: any[]): any;
    function FS_createPreloadedFile(parent: any, name: any, url: any, canRead: any, canWrite: any, onload: any, onerror: any, dontCreateFile: any, canOwn: any, preFinish: any): void;
    function FS_unlink(...args: any[]): any;
    function FS_createLazyFile(...args: any[]): any;
    function FS_createDevice(...args: any[]): any;
    let addRunDependency: any;
    let removeRunDependency: any;
}
declare class ErrnoError extends Error {
    constructor(errno: any);
    errno: any;
    code: string;
}
declare class FSStream {
    shared: {};
    set object(val: any);
    get object(): any;
    node: any;
    get isRead(): boolean;
    get isWrite(): boolean;
    get isAppend(): number;
    set flags(val: any);
    get flags(): any;
    set position(val: any);
    get position(): any;
}
declare class FSNode {
    constructor(parent: any, name: any, mode: any, rdev: any);
    node_ops: {};
    stream_ops: {};
    readMode: number;
    writeMode: number;
    mounted: any;
    parent: any;
    mount: any;
    id: number;
    name: any;
    mode: any;
    rdev: any;
    atime: number;
    mtime: number;
    ctime: number;
    set read(val: boolean);
    get read(): boolean;
    set write(val: boolean);
    get write(): boolean;
    get isFolder(): any;
    get isDevice(): any;
}
interface WasmModule {
}

type EmbindString = ArrayBuffer|Uint8Array|Uint8ClampedArray|Int8Array|string;
export interface ClassHandle {
  isAliasOf(other: ClassHandle): boolean;
  delete(): void;
  deleteLater(): this;
  isDeleted(): boolean;
  // @ts-ignore - If targeting lower than ESNext, this symbol might not exist.
  [Symbol.dispose](): void;
  clone(): this;
}
export interface mjtDisableBitValue<T extends number> {
  value: T;
}
export type mjtDisableBit = mjtDisableBitValue<1>|mjtDisableBitValue<2>|mjtDisableBitValue<4>|mjtDisableBitValue<8>|mjtDisableBitValue<16>|mjtDisableBitValue<32>|mjtDisableBitValue<64>|mjtDisableBitValue<128>|mjtDisableBitValue<256>|mjtDisableBitValue<512>|mjtDisableBitValue<1024>|mjtDisableBitValue<2048>|mjtDisableBitValue<4096>|mjtDisableBitValue<8192>|mjtDisableBitValue<16384>|mjtDisableBitValue<32768>|mjtDisableBitValue<65536>|mjtDisableBitValue<131072>|mjtDisableBitValue<262144>|mjtDisableBitValue<19>;

export interface mjtEnableBitValue<T extends number> {
  value: T;
}
export type mjtEnableBit = mjtEnableBitValue<1>|mjtEnableBitValue<2>|mjtEnableBitValue<4>|mjtEnableBitValue<8>|mjtEnableBitValue<16>|mjtEnableBitValue<5>;

export interface mjtJointValue<T extends number> {
  value: T;
}
export type mjtJoint = mjtJointValue<0>|mjtJointValue<1>|mjtJointValue<2>|mjtJointValue<3>;

export interface mjtGeomValue<T extends number> {
  value: T;
}
export type mjtGeom = mjtGeomValue<0>|mjtGeomValue<1>|mjtGeomValue<2>|mjtGeomValue<3>|mjtGeomValue<4>|mjtGeomValue<5>|mjtGeomValue<6>|mjtGeomValue<7>|mjtGeomValue<8>|mjtGeomValue<9>|mjtGeomValue<100>|mjtGeomValue<101>|mjtGeomValue<102>|mjtGeomValue<103>|mjtGeomValue<104>|mjtGeomValue<105>|mjtGeomValue<106>|mjtGeomValue<107>|mjtGeomValue<108>|mjtGeomValue<1001>;

export interface mjtCamLightValue<T extends number> {
  value: T;
}
export type mjtCamLight = mjtCamLightValue<0>|mjtCamLightValue<1>|mjtCamLightValue<2>|mjtCamLightValue<3>|mjtCamLightValue<4>;

export interface mjtLightTypeValue<T extends number> {
  value: T;
}
export type mjtLightType = mjtLightTypeValue<0>|mjtLightTypeValue<1>|mjtLightTypeValue<2>|mjtLightTypeValue<3>;

export interface mjtTextureValue<T extends number> {
  value: T;
}
export type mjtTexture = mjtTextureValue<0>|mjtTextureValue<1>|mjtTextureValue<2>;

export interface mjtTextureRoleValue<T extends number> {
  value: T;
}
export type mjtTextureRole = mjtTextureRoleValue<0>|mjtTextureRoleValue<1>|mjtTextureRoleValue<2>|mjtTextureRoleValue<3>|mjtTextureRoleValue<4>|mjtTextureRoleValue<5>|mjtTextureRoleValue<6>|mjtTextureRoleValue<7>|mjtTextureRoleValue<8>|mjtTextureRoleValue<9>|mjtTextureRoleValue<10>;

export interface mjtColorSpaceValue<T extends number> {
  value: T;
}
export type mjtColorSpace = mjtColorSpaceValue<0>|mjtColorSpaceValue<1>|mjtColorSpaceValue<2>;

export interface mjtIntegratorValue<T extends number> {
  value: T;
}
export type mjtIntegrator = mjtIntegratorValue<0>|mjtIntegratorValue<1>|mjtIntegratorValue<2>|mjtIntegratorValue<3>;

export interface mjtConeValue<T extends number> {
  value: T;
}
export type mjtCone = mjtConeValue<0>|mjtConeValue<1>;

export interface mjtJacobianValue<T extends number> {
  value: T;
}
export type mjtJacobian = mjtJacobianValue<0>|mjtJacobianValue<1>|mjtJacobianValue<2>;

export interface mjtSolverValue<T extends number> {
  value: T;
}
export type mjtSolver = mjtSolverValue<0>|mjtSolverValue<1>|mjtSolverValue<2>;

export interface mjtEqValue<T extends number> {
  value: T;
}
export type mjtEq = mjtEqValue<0>|mjtEqValue<1>|mjtEqValue<2>|mjtEqValue<3>|mjtEqValue<4>|mjtEqValue<5>;

export interface mjtWrapValue<T extends number> {
  value: T;
}
export type mjtWrap = mjtWrapValue<0>|mjtWrapValue<1>|mjtWrapValue<2>|mjtWrapValue<3>|mjtWrapValue<4>|mjtWrapValue<5>;

export interface mjtTrnValue<T extends number> {
  value: T;
}
export type mjtTrn = mjtTrnValue<0>|mjtTrnValue<1>|mjtTrnValue<2>|mjtTrnValue<3>|mjtTrnValue<4>|mjtTrnValue<5>|mjtTrnValue<1000>;

export interface mjtDynValue<T extends number> {
  value: T;
}
export type mjtDyn = mjtDynValue<0>|mjtDynValue<1>|mjtDynValue<2>|mjtDynValue<3>|mjtDynValue<4>|mjtDynValue<5>;

export interface mjtGainValue<T extends number> {
  value: T;
}
export type mjtGain = mjtGainValue<0>|mjtGainValue<1>|mjtGainValue<2>|mjtGainValue<3>;

export interface mjtBiasValue<T extends number> {
  value: T;
}
export type mjtBias = mjtBiasValue<0>|mjtBiasValue<1>|mjtBiasValue<2>|mjtBiasValue<3>;

export interface mjtObjValue<T extends number> {
  value: T;
}
export type mjtObj = mjtObjValue<0>|mjtObjValue<1>|mjtObjValue<2>|mjtObjValue<3>|mjtObjValue<4>|mjtObjValue<5>|mjtObjValue<6>|mjtObjValue<7>|mjtObjValue<8>|mjtObjValue<9>|mjtObjValue<10>|mjtObjValue<11>|mjtObjValue<12>|mjtObjValue<13>|mjtObjValue<14>|mjtObjValue<15>|mjtObjValue<16>|mjtObjValue<17>|mjtObjValue<18>|mjtObjValue<19>|mjtObjValue<20>|mjtObjValue<21>|mjtObjValue<22>|mjtObjValue<23>|mjtObjValue<24>|mjtObjValue<25>|mjtObjValue<26>|mjtObjValue<100>|mjtObjValue<101>|mjtObjValue<102>;

export interface mjtSensorValue<T extends number> {
  value: T;
}
export type mjtSensor = mjtSensorValue<0>|mjtSensorValue<1>|mjtSensorValue<2>|mjtSensorValue<3>|mjtSensorValue<4>|mjtSensorValue<5>|mjtSensorValue<6>|mjtSensorValue<7>|mjtSensorValue<8>|mjtSensorValue<9>|mjtSensorValue<10>|mjtSensorValue<11>|mjtSensorValue<12>|mjtSensorValue<13>|mjtSensorValue<14>|mjtSensorValue<15>|mjtSensorValue<16>|mjtSensorValue<17>|mjtSensorValue<18>|mjtSensorValue<19>|mjtSensorValue<20>|mjtSensorValue<21>|mjtSensorValue<22>|mjtSensorValue<23>|mjtSensorValue<24>|mjtSensorValue<25>|mjtSensorValue<26>|mjtSensorValue<27>|mjtSensorValue<28>|mjtSensorValue<29>|mjtSensorValue<30>|mjtSensorValue<31>|mjtSensorValue<32>|mjtSensorValue<33>|mjtSensorValue<34>|mjtSensorValue<35>|mjtSensorValue<36>|mjtSensorValue<37>|mjtSensorValue<38>|mjtSensorValue<39>|mjtSensorValue<40>|mjtSensorValue<41>|mjtSensorValue<42>|mjtSensorValue<43>|mjtSensorValue<44>|mjtSensorValue<45>|mjtSensorValue<46>|mjtSensorValue<47>|mjtSensorValue<48>;

export interface mjtStageValue<T extends number> {
  value: T;
}
export type mjtStage = mjtStageValue<0>|mjtStageValue<1>|mjtStageValue<2>|mjtStageValue<3>;

export interface mjtDataTypeValue<T extends number> {
  value: T;
}
export type mjtDataType = mjtDataTypeValue<0>|mjtDataTypeValue<1>|mjtDataTypeValue<2>|mjtDataTypeValue<3>;

export interface mjtConDataFieldValue<T extends number> {
  value: T;
}
export type mjtConDataField = mjtConDataFieldValue<0>|mjtConDataFieldValue<1>|mjtConDataFieldValue<2>|mjtConDataFieldValue<3>|mjtConDataFieldValue<4>|mjtConDataFieldValue<5>|mjtConDataFieldValue<6>|mjtConDataFieldValue<7>;

export interface mjtSameFrameValue<T extends number> {
  value: T;
}
export type mjtSameFrame = mjtSameFrameValue<0>|mjtSameFrameValue<1>|mjtSameFrameValue<2>|mjtSameFrameValue<3>|mjtSameFrameValue<4>;

export interface mjtLRModeValue<T extends number> {
  value: T;
}
export type mjtLRMode = mjtLRModeValue<0>|mjtLRModeValue<1>|mjtLRModeValue<2>|mjtLRModeValue<3>;

export interface mjtFlexSelfValue<T extends number> {
  value: T;
}
export type mjtFlexSelf = mjtFlexSelfValue<0>|mjtFlexSelfValue<1>|mjtFlexSelfValue<2>|mjtFlexSelfValue<3>|mjtFlexSelfValue<4>;

export interface mjtSDFTypeValue<T extends number> {
  value: T;
}
export type mjtSDFType = mjtSDFTypeValue<0>|mjtSDFTypeValue<1>|mjtSDFTypeValue<2>|mjtSDFTypeValue<3>;

export interface mjtTaskStatusValue<T extends number> {
  value: T;
}
export type mjtTaskStatus = mjtTaskStatusValue<0>|mjtTaskStatusValue<1>|mjtTaskStatusValue<2>;

export interface mjtStateValue<T extends number> {
  value: T;
}
export type mjtState = mjtStateValue<1>|mjtStateValue<2>|mjtStateValue<4>|mjtStateValue<8>|mjtStateValue<16>|mjtStateValue<32>|mjtStateValue<64>|mjtStateValue<128>|mjtStateValue<256>|mjtStateValue<512>|mjtStateValue<1024>|mjtStateValue<2048>|mjtStateValue<4096>|mjtStateValue<13>|mjtStateValue<14>|mjtStateValue<4111>|mjtStateValue<4064>|mjtStateValue<8191>;

export interface mjtConstraintValue<T extends number> {
  value: T;
}
export type mjtConstraint = mjtConstraintValue<0>|mjtConstraintValue<1>|mjtConstraintValue<2>|mjtConstraintValue<3>|mjtConstraintValue<4>|mjtConstraintValue<5>|mjtConstraintValue<6>|mjtConstraintValue<7>;

export interface mjtConstraintStateValue<T extends number> {
  value: T;
}
export type mjtConstraintState = mjtConstraintStateValue<0>|mjtConstraintStateValue<1>|mjtConstraintStateValue<2>|mjtConstraintStateValue<3>|mjtConstraintStateValue<4>;

export interface mjtWarningValue<T extends number> {
  value: T;
}
export type mjtWarning = mjtWarningValue<0>|mjtWarningValue<1>|mjtWarningValue<2>|mjtWarningValue<3>|mjtWarningValue<4>|mjtWarningValue<5>|mjtWarningValue<6>|mjtWarningValue<7>|mjtWarningValue<8>;

export interface mjtTimerValue<T extends number> {
  value: T;
}
export type mjtTimer = mjtTimerValue<0>|mjtTimerValue<1>|mjtTimerValue<2>|mjtTimerValue<3>|mjtTimerValue<4>|mjtTimerValue<5>|mjtTimerValue<6>|mjtTimerValue<7>|mjtTimerValue<8>|mjtTimerValue<9>|mjtTimerValue<10>|mjtTimerValue<11>|mjtTimerValue<12>|mjtTimerValue<13>|mjtTimerValue<14>|mjtTimerValue<15>;

export interface mjtGeomInertiaValue<T extends number> {
  value: T;
}
export type mjtGeomInertia = mjtGeomInertiaValue<0>|mjtGeomInertiaValue<1>;

export interface mjtMeshInertiaValue<T extends number> {
  value: T;
}
export type mjtMeshInertia = mjtMeshInertiaValue<0>|mjtMeshInertiaValue<1>|mjtMeshInertiaValue<2>|mjtMeshInertiaValue<3>;

export interface mjtMeshBuiltinValue<T extends number> {
  value: T;
}
export type mjtMeshBuiltin = mjtMeshBuiltinValue<0>|mjtMeshBuiltinValue<1>|mjtMeshBuiltinValue<2>|mjtMeshBuiltinValue<3>|mjtMeshBuiltinValue<4>|mjtMeshBuiltinValue<5>|mjtMeshBuiltinValue<6>|mjtMeshBuiltinValue<7>;

export interface mjtBuiltinValue<T extends number> {
  value: T;
}
export type mjtBuiltin = mjtBuiltinValue<0>|mjtBuiltinValue<1>|mjtBuiltinValue<2>|mjtBuiltinValue<3>;

export interface mjtMarkValue<T extends number> {
  value: T;
}
export type mjtMark = mjtMarkValue<0>|mjtMarkValue<1>|mjtMarkValue<2>|mjtMarkValue<3>;

export interface mjtLimitedValue<T extends number> {
  value: T;
}
export type mjtLimited = mjtLimitedValue<0>|mjtLimitedValue<1>|mjtLimitedValue<2>;

export interface mjtAlignFreeValue<T extends number> {
  value: T;
}
export type mjtAlignFree = mjtAlignFreeValue<0>|mjtAlignFreeValue<1>|mjtAlignFreeValue<2>;

export interface mjtInertiaFromGeomValue<T extends number> {
  value: T;
}
export type mjtInertiaFromGeom = mjtInertiaFromGeomValue<0>|mjtInertiaFromGeomValue<1>|mjtInertiaFromGeomValue<2>;

export interface mjtOrientationValue<T extends number> {
  value: T;
}
export type mjtOrientation = mjtOrientationValue<0>|mjtOrientationValue<1>|mjtOrientationValue<2>|mjtOrientationValue<3>|mjtOrientationValue<4>;

export interface mjtCatBitValue<T extends number> {
  value: T;
}
export type mjtCatBit = mjtCatBitValue<1>|mjtCatBitValue<2>|mjtCatBitValue<4>|mjtCatBitValue<7>;

export interface mjtMouseValue<T extends number> {
  value: T;
}
export type mjtMouse = mjtMouseValue<0>|mjtMouseValue<1>|mjtMouseValue<2>|mjtMouseValue<3>|mjtMouseValue<4>|mjtMouseValue<5>|mjtMouseValue<6>|mjtMouseValue<7>;

export interface mjtPertBitValue<T extends number> {
  value: T;
}
export type mjtPertBit = mjtPertBitValue<1>|mjtPertBitValue<2>;

export interface mjtCameraValue<T extends number> {
  value: T;
}
export type mjtCamera = mjtCameraValue<0>|mjtCameraValue<1>|mjtCameraValue<2>|mjtCameraValue<3>;

export interface mjtLabelValue<T extends number> {
  value: T;
}
export type mjtLabel = mjtLabelValue<0>|mjtLabelValue<1>|mjtLabelValue<2>|mjtLabelValue<3>|mjtLabelValue<4>|mjtLabelValue<5>|mjtLabelValue<6>|mjtLabelValue<7>|mjtLabelValue<8>|mjtLabelValue<9>|mjtLabelValue<10>|mjtLabelValue<11>|mjtLabelValue<12>|mjtLabelValue<13>|mjtLabelValue<14>|mjtLabelValue<15>|mjtLabelValue<16>|mjtLabelValue<17>;

export interface mjtFrameValue<T extends number> {
  value: T;
}
export type mjtFrame = mjtFrameValue<0>|mjtFrameValue<1>|mjtFrameValue<2>|mjtFrameValue<3>|mjtFrameValue<4>|mjtFrameValue<5>|mjtFrameValue<6>|mjtFrameValue<7>|mjtFrameValue<8>;

export interface mjtVisFlagValue<T extends number> {
  value: T;
}
export type mjtVisFlag = mjtVisFlagValue<0>|mjtVisFlagValue<1>|mjtVisFlagValue<2>|mjtVisFlagValue<3>|mjtVisFlagValue<4>|mjtVisFlagValue<5>|mjtVisFlagValue<6>|mjtVisFlagValue<7>|mjtVisFlagValue<8>|mjtVisFlagValue<9>|mjtVisFlagValue<10>|mjtVisFlagValue<11>|mjtVisFlagValue<12>|mjtVisFlagValue<13>|mjtVisFlagValue<14>|mjtVisFlagValue<15>|mjtVisFlagValue<16>|mjtVisFlagValue<17>|mjtVisFlagValue<18>|mjtVisFlagValue<19>|mjtVisFlagValue<20>|mjtVisFlagValue<21>|mjtVisFlagValue<22>|mjtVisFlagValue<23>|mjtVisFlagValue<24>|mjtVisFlagValue<25>|mjtVisFlagValue<26>|mjtVisFlagValue<27>|mjtVisFlagValue<28>|mjtVisFlagValue<29>|mjtVisFlagValue<30>|mjtVisFlagValue<31>;

export interface mjtRndFlagValue<T extends number> {
  value: T;
}
export type mjtRndFlag = mjtRndFlagValue<0>|mjtRndFlagValue<1>|mjtRndFlagValue<2>|mjtRndFlagValue<3>|mjtRndFlagValue<4>|mjtRndFlagValue<5>|mjtRndFlagValue<6>|mjtRndFlagValue<7>|mjtRndFlagValue<8>|mjtRndFlagValue<9>|mjtRndFlagValue<10>;

export interface mjtStereoValue<T extends number> {
  value: T;
}
export type mjtStereo = mjtStereoValue<0>|mjtStereoValue<1>|mjtStereoValue<2>;

export interface mjtPluginCapabilityBitValue<T extends number> {
  value: T;
}
export type mjtPluginCapabilityBit = mjtPluginCapabilityBitValue<1>|mjtPluginCapabilityBitValue<2>|mjtPluginCapabilityBitValue<4>|mjtPluginCapabilityBitValue<8>;

export interface mjtGridPosValue<T extends number> {
  value: T;
}
export type mjtGridPos = mjtGridPosValue<0>|mjtGridPosValue<1>|mjtGridPosValue<2>|mjtGridPosValue<3>|mjtGridPosValue<4>|mjtGridPosValue<5>|mjtGridPosValue<6>|mjtGridPosValue<7>;

export interface mjtFramebufferValue<T extends number> {
  value: T;
}
export type mjtFramebuffer = mjtFramebufferValue<0>|mjtFramebufferValue<1>;

export interface mjtDepthMapValue<T extends number> {
  value: T;
}
export type mjtDepthMap = mjtDepthMapValue<0>|mjtDepthMapValue<1>;

export interface mjtFontScaleValue<T extends number> {
  value: T;
}
export type mjtFontScale = mjtFontScaleValue<50>|mjtFontScaleValue<100>|mjtFontScaleValue<150>|mjtFontScaleValue<200>|mjtFontScaleValue<250>|mjtFontScaleValue<300>;

export interface mjtFontValue<T extends number> {
  value: T;
}
export type mjtFont = mjtFontValue<0>|mjtFontValue<1>|mjtFontValue<2>;

export interface mjtButtonValue<T extends number> {
  value: T;
}
export type mjtButton = mjtButtonValue<0>|mjtButtonValue<1>|mjtButtonValue<2>|mjtButtonValue<3>;

export interface mjtEventValue<T extends number> {
  value: T;
}
export type mjtEvent = mjtEventValue<0>|mjtEventValue<1>|mjtEventValue<2>|mjtEventValue<3>|mjtEventValue<4>|mjtEventValue<5>|mjtEventValue<6>|mjtEventValue<7>|mjtEventValue<8>;

export interface mjtItemValue<T extends number> {
  value: T;
}
export type mjtItem = mjtItemValue<-2>|mjtItemValue<-1>|mjtItemValue<0>|mjtItemValue<1>|mjtItemValue<2>|mjtItemValue<3>|mjtItemValue<4>|mjtItemValue<5>|mjtItemValue<6>|mjtItemValue<7>|mjtItemValue<8>|mjtItemValue<9>|mjtItemValue<10>|mjtItemValue<11>|mjtItemValue<12>|mjtItemValue<13>|mjtItemValue<14>;

export interface mjtSectionValue<T extends number> {
  value: T;
}
export type mjtSection = mjtSectionValue<0>|mjtSectionValue<1>|mjtSectionValue<2>;

export interface MjLROpt extends ClassHandle {
  mode: number;
  useexisting: number;
  uselimit: number;
  accel: number;
  maxforce: number;
  timeconst: number;
  timestep: number;
  inttotal: number;
  interval: number;
  tolrange: number;
  copy(): MjLROpt;
}

export interface MjModel extends ClassHandle {
  opt: MjOption;
  stat: MjStatistic;
  vis: MjVisual;
  nq: number;
  nv: number;
  nu: number;
  na: number;
  nbody: number;
  nbvh: number;
  nbvhstatic: number;
  nbvhdynamic: number;
  noct: number;
  njnt: number;
  ntree: number;
  nM: number;
  nB: number;
  nC: number;
  nD: number;
  ngeom: number;
  nsite: number;
  ncam: number;
  nlight: number;
  nflex: number;
  nflexnode: number;
  nflexvert: number;
  nflexedge: number;
  nflexelem: number;
  nflexelemdata: number;
  nflexelemedge: number;
  nflexshelldata: number;
  nflexevpair: number;
  nflextexcoord: number;
  nmesh: number;
  nmeshvert: number;
  nmeshnormal: number;
  nmeshtexcoord: number;
  nmeshface: number;
  nmeshgraph: number;
  nmeshpoly: number;
  nmeshpolyvert: number;
  nmeshpolymap: number;
  nskin: number;
  nskinvert: number;
  nskintexvert: number;
  nskinface: number;
  nskinbone: number;
  nskinbonevert: number;
  nhfield: number;
  nhfielddata: number;
  ntex: number;
  ntexdata: number;
  nmat: number;
  npair: number;
  nexclude: number;
  neq: number;
  ntendon: number;
  nwrap: number;
  nsensor: number;
  nnumeric: number;
  nnumericdata: number;
  ntext: number;
  ntextdata: number;
  ntuple: number;
  ntupledata: number;
  nkey: number;
  nmocap: number;
  nplugin: number;
  npluginattr: number;
  nuser_body: number;
  nuser_jnt: number;
  nuser_geom: number;
  nuser_site: number;
  nuser_cam: number;
  nuser_tendon: number;
  nuser_actuator: number;
  nuser_sensor: number;
  nnames: number;
  npaths: number;
  nnames_map: number;
  nJmom: number;
  ngravcomp: number;
  nemax: number;
  njmax: number;
  nconmax: number;
  nuserdata: number;
  nsensordata: number;
  npluginstate: number;
  narena: bigint;
  nbuffer: bigint;
  signature: bigint;
  readonly buffer: any;
  readonly qpos0: any;
  readonly qpos_spring: any;
  readonly body_parentid: any;
  readonly body_rootid: any;
  readonly body_weldid: any;
  readonly body_mocapid: any;
  readonly body_jntnum: any;
  readonly body_jntadr: any;
  readonly body_dofnum: any;
  readonly body_dofadr: any;
  readonly body_treeid: any;
  readonly body_geomnum: any;
  readonly body_geomadr: any;
  readonly body_simple: any;
  readonly body_sameframe: any;
  readonly body_pos: any;
  readonly body_quat: any;
  readonly body_ipos: any;
  readonly body_iquat: any;
  readonly body_mass: any;
  readonly body_subtreemass: any;
  readonly body_inertia: any;
  readonly body_invweight0: any;
  readonly body_gravcomp: any;
  readonly body_margin: any;
  readonly body_user: any;
  readonly body_plugin: any;
  readonly body_contype: any;
  readonly body_conaffinity: any;
  readonly body_bvhadr: any;
  readonly body_bvhnum: any;
  readonly bvh_depth: any;
  readonly bvh_child: any;
  readonly bvh_nodeid: any;
  readonly bvh_aabb: any;
  readonly oct_depth: any;
  readonly oct_child: any;
  readonly oct_aabb: any;
  readonly oct_coeff: any;
  readonly jnt_type: any;
  readonly jnt_qposadr: any;
  readonly jnt_dofadr: any;
  readonly jnt_bodyid: any;
  readonly jnt_group: any;
  readonly jnt_limited: any;
  readonly jnt_actfrclimited: any;
  readonly jnt_actgravcomp: any;
  readonly jnt_solref: any;
  readonly jnt_solimp: any;
  readonly jnt_pos: any;
  readonly jnt_axis: any;
  readonly jnt_stiffness: any;
  readonly jnt_range: any;
  readonly jnt_actfrcrange: any;
  readonly jnt_margin: any;
  readonly jnt_user: any;
  readonly dof_bodyid: any;
  readonly dof_jntid: any;
  readonly dof_parentid: any;
  readonly dof_treeid: any;
  readonly dof_Madr: any;
  readonly dof_simplenum: any;
  readonly dof_solref: any;
  readonly dof_solimp: any;
  readonly dof_frictionloss: any;
  readonly dof_armature: any;
  readonly dof_damping: any;
  readonly dof_invweight0: any;
  readonly dof_M0: any;
  readonly geom_type: any;
  readonly geom_contype: any;
  readonly geom_conaffinity: any;
  readonly geom_condim: any;
  readonly geom_bodyid: any;
  readonly geom_dataid: any;
  readonly geom_matid: any;
  readonly geom_group: any;
  readonly geom_priority: any;
  readonly geom_plugin: any;
  readonly geom_sameframe: any;
  readonly geom_solmix: any;
  readonly geom_solref: any;
  readonly geom_solimp: any;
  readonly geom_size: any;
  readonly geom_aabb: any;
  readonly geom_rbound: any;
  readonly geom_pos: any;
  readonly geom_quat: any;
  readonly geom_friction: any;
  readonly geom_margin: any;
  readonly geom_gap: any;
  readonly geom_fluid: any;
  readonly geom_user: any;
  readonly geom_rgba: any;
  readonly site_type: any;
  readonly site_bodyid: any;
  readonly site_matid: any;
  readonly site_group: any;
  readonly site_sameframe: any;
  readonly site_size: any;
  readonly site_pos: any;
  readonly site_quat: any;
  readonly site_user: any;
  readonly site_rgba: any;
  readonly cam_mode: any;
  readonly cam_bodyid: any;
  readonly cam_targetbodyid: any;
  readonly cam_pos: any;
  readonly cam_quat: any;
  readonly cam_poscom0: any;
  readonly cam_pos0: any;
  readonly cam_mat0: any;
  readonly cam_orthographic: any;
  readonly cam_fovy: any;
  readonly cam_ipd: any;
  readonly cam_resolution: any;
  readonly cam_sensorsize: any;
  readonly cam_intrinsic: any;
  readonly cam_user: any;
  readonly light_mode: any;
  readonly light_bodyid: any;
  readonly light_targetbodyid: any;
  readonly light_type: any;
  readonly light_texid: any;
  readonly light_castshadow: any;
  readonly light_bulbradius: any;
  readonly light_intensity: any;
  readonly light_range: any;
  readonly light_active: any;
  readonly light_pos: any;
  readonly light_dir: any;
  readonly light_poscom0: any;
  readonly light_pos0: any;
  readonly light_dir0: any;
  readonly light_attenuation: any;
  readonly light_cutoff: any;
  readonly light_exponent: any;
  readonly light_ambient: any;
  readonly light_diffuse: any;
  readonly light_specular: any;
  readonly flex_contype: any;
  readonly flex_conaffinity: any;
  readonly flex_condim: any;
  readonly flex_priority: any;
  readonly flex_solmix: any;
  readonly flex_solref: any;
  readonly flex_solimp: any;
  readonly flex_friction: any;
  readonly flex_margin: any;
  readonly flex_gap: any;
  readonly flex_internal: any;
  readonly flex_selfcollide: any;
  readonly flex_activelayers: any;
  readonly flex_passive: any;
  readonly flex_dim: any;
  readonly flex_matid: any;
  readonly flex_group: any;
  readonly flex_interp: any;
  readonly flex_nodeadr: any;
  readonly flex_nodenum: any;
  readonly flex_vertadr: any;
  readonly flex_vertnum: any;
  readonly flex_edgeadr: any;
  readonly flex_edgenum: any;
  readonly flex_elemadr: any;
  readonly flex_elemnum: any;
  readonly flex_elemdataadr: any;
  readonly flex_elemedgeadr: any;
  readonly flex_shellnum: any;
  readonly flex_shelldataadr: any;
  readonly flex_evpairadr: any;
  readonly flex_evpairnum: any;
  readonly flex_texcoordadr: any;
  readonly flex_nodebodyid: any;
  readonly flex_vertbodyid: any;
  readonly flex_edge: any;
  readonly flex_edgeflap: any;
  readonly flex_elem: any;
  readonly flex_elemtexcoord: any;
  readonly flex_elemedge: any;
  readonly flex_elemlayer: any;
  readonly flex_shell: any;
  readonly flex_evpair: any;
  readonly flex_vert: any;
  readonly flex_vert0: any;
  readonly flex_node: any;
  readonly flex_node0: any;
  readonly flexedge_length0: any;
  readonly flexedge_invweight0: any;
  readonly flex_radius: any;
  readonly flex_stiffness: any;
  readonly flex_bending: any;
  readonly flex_damping: any;
  readonly flex_edgestiffness: any;
  readonly flex_edgedamping: any;
  readonly flex_edgeequality: any;
  readonly flex_rigid: any;
  readonly flexedge_rigid: any;
  readonly flex_centered: any;
  readonly flex_flatskin: any;
  readonly flex_bvhadr: any;
  readonly flex_bvhnum: any;
  readonly flex_rgba: any;
  readonly flex_texcoord: any;
  readonly mesh_vertadr: any;
  readonly mesh_vertnum: any;
  readonly mesh_faceadr: any;
  readonly mesh_facenum: any;
  readonly mesh_bvhadr: any;
  readonly mesh_bvhnum: any;
  readonly mesh_octadr: any;
  readonly mesh_octnum: any;
  readonly mesh_normaladr: any;
  readonly mesh_normalnum: any;
  readonly mesh_texcoordadr: any;
  readonly mesh_texcoordnum: any;
  readonly mesh_graphadr: any;
  readonly mesh_vert: any;
  readonly mesh_normal: any;
  readonly mesh_texcoord: any;
  readonly mesh_face: any;
  readonly mesh_facenormal: any;
  readonly mesh_facetexcoord: any;
  readonly mesh_graph: any;
  readonly mesh_scale: any;
  readonly mesh_pos: any;
  readonly mesh_quat: any;
  readonly mesh_pathadr: any;
  readonly mesh_polynum: any;
  readonly mesh_polyadr: any;
  readonly mesh_polynormal: any;
  readonly mesh_polyvertadr: any;
  readonly mesh_polyvertnum: any;
  readonly mesh_polyvert: any;
  readonly mesh_polymapadr: any;
  readonly mesh_polymapnum: any;
  readonly mesh_polymap: any;
  readonly skin_matid: any;
  readonly skin_group: any;
  readonly skin_rgba: any;
  readonly skin_inflate: any;
  readonly skin_vertadr: any;
  readonly skin_vertnum: any;
  readonly skin_texcoordadr: any;
  readonly skin_faceadr: any;
  readonly skin_facenum: any;
  readonly skin_boneadr: any;
  readonly skin_bonenum: any;
  readonly skin_vert: any;
  readonly skin_texcoord: any;
  readonly skin_face: any;
  readonly skin_bonevertadr: any;
  readonly skin_bonevertnum: any;
  readonly skin_bonebindpos: any;
  readonly skin_bonebindquat: any;
  readonly skin_bonebodyid: any;
  readonly skin_bonevertid: any;
  readonly skin_bonevertweight: any;
  readonly skin_pathadr: any;
  readonly hfield_size: any;
  readonly hfield_nrow: any;
  readonly hfield_ncol: any;
  readonly hfield_adr: any;
  readonly hfield_data: any;
  readonly hfield_pathadr: any;
  readonly tex_type: any;
  readonly tex_colorspace: any;
  readonly tex_height: any;
  readonly tex_width: any;
  readonly tex_nchannel: any;
  readonly tex_adr: any;
  readonly tex_data: any;
  readonly tex_pathadr: any;
  readonly mat_texid: any;
  readonly mat_texuniform: any;
  readonly mat_texrepeat: any;
  readonly mat_emission: any;
  readonly mat_specular: any;
  readonly mat_shininess: any;
  readonly mat_reflectance: any;
  readonly mat_metallic: any;
  readonly mat_roughness: any;
  readonly mat_rgba: any;
  readonly pair_dim: any;
  readonly pair_geom1: any;
  readonly pair_geom2: any;
  readonly pair_signature: any;
  readonly pair_solref: any;
  readonly pair_solreffriction: any;
  readonly pair_solimp: any;
  readonly pair_margin: any;
  readonly pair_gap: any;
  readonly pair_friction: any;
  readonly exclude_signature: any;
  readonly eq_type: any;
  readonly eq_obj1id: any;
  readonly eq_obj2id: any;
  readonly eq_objtype: any;
  readonly eq_active0: any;
  readonly eq_solref: any;
  readonly eq_solimp: any;
  readonly eq_data: any;
  readonly tendon_adr: any;
  readonly tendon_num: any;
  readonly tendon_matid: any;
  readonly tendon_group: any;
  readonly tendon_limited: any;
  readonly tendon_actfrclimited: any;
  readonly tendon_width: any;
  readonly tendon_solref_lim: any;
  readonly tendon_solimp_lim: any;
  readonly tendon_solref_fri: any;
  readonly tendon_solimp_fri: any;
  readonly tendon_range: any;
  readonly tendon_actfrcrange: any;
  readonly tendon_margin: any;
  readonly tendon_stiffness: any;
  readonly tendon_damping: any;
  readonly tendon_armature: any;
  readonly tendon_frictionloss: any;
  readonly tendon_lengthspring: any;
  readonly tendon_length0: any;
  readonly tendon_invweight0: any;
  readonly tendon_user: any;
  readonly tendon_rgba: any;
  readonly wrap_type: any;
  readonly wrap_objid: any;
  readonly wrap_prm: any;
  readonly actuator_trntype: any;
  readonly actuator_dyntype: any;
  readonly actuator_gaintype: any;
  readonly actuator_biastype: any;
  readonly actuator_trnid: any;
  readonly actuator_actadr: any;
  readonly actuator_actnum: any;
  readonly actuator_group: any;
  readonly actuator_ctrllimited: any;
  readonly actuator_forcelimited: any;
  readonly actuator_actlimited: any;
  readonly actuator_dynprm: any;
  readonly actuator_gainprm: any;
  readonly actuator_biasprm: any;
  readonly actuator_actearly: any;
  readonly actuator_ctrlrange: any;
  readonly actuator_forcerange: any;
  readonly actuator_actrange: any;
  readonly actuator_gear: any;
  readonly actuator_cranklength: any;
  readonly actuator_acc0: any;
  readonly actuator_length0: any;
  readonly actuator_lengthrange: any;
  readonly actuator_user: any;
  readonly actuator_plugin: any;
  readonly sensor_type: any;
  readonly sensor_datatype: any;
  readonly sensor_needstage: any;
  readonly sensor_objtype: any;
  readonly sensor_objid: any;
  readonly sensor_reftype: any;
  readonly sensor_refid: any;
  readonly sensor_intprm: any;
  readonly sensor_dim: any;
  readonly sensor_adr: any;
  readonly sensor_cutoff: any;
  readonly sensor_noise: any;
  readonly sensor_user: any;
  readonly sensor_plugin: any;
  readonly plugin: any;
  readonly plugin_stateadr: any;
  readonly plugin_statenum: any;
  readonly plugin_attr: any;
  readonly plugin_attradr: any;
  readonly numeric_adr: any;
  readonly numeric_size: any;
  readonly numeric_data: any;
  readonly text_adr: any;
  readonly text_size: any;
  readonly text_data: any;
  readonly tuple_adr: any;
  readonly tuple_size: any;
  readonly tuple_objtype: any;
  readonly tuple_objid: any;
  readonly tuple_objprm: any;
  readonly key_time: any;
  readonly key_qpos: any;
  readonly key_qvel: any;
  readonly key_act: any;
  readonly key_mpos: any;
  readonly key_mquat: any;
  readonly key_ctrl: any;
  readonly name_bodyadr: any;
  readonly name_jntadr: any;
  readonly name_geomadr: any;
  readonly name_siteadr: any;
  readonly name_camadr: any;
  readonly name_lightadr: any;
  readonly name_flexadr: any;
  readonly name_meshadr: any;
  readonly name_skinadr: any;
  readonly name_hfieldadr: any;
  readonly name_texadr: any;
  readonly name_matadr: any;
  readonly name_pairadr: any;
  readonly name_excludeadr: any;
  readonly name_eqadr: any;
  readonly name_tendonadr: any;
  readonly name_actuatoradr: any;
  readonly name_sensoradr: any;
  readonly name_numericadr: any;
  readonly name_textadr: any;
  readonly name_tupleadr: any;
  readonly name_keyadr: any;
  readonly name_pluginadr: any;
  readonly names: any;
  readonly names_map: any;
  readonly paths: any;
  readonly B_rownnz: any;
  readonly B_rowadr: any;
  readonly B_colind: any;
  readonly M_rownnz: any;
  readonly M_rowadr: any;
  readonly M_colind: any;
  readonly mapM2M: any;
  readonly D_rownnz: any;
  readonly D_rowadr: any;
  readonly D_diag: any;
  readonly D_colind: any;
  readonly mapM2D: any;
  readonly mapD2M: any;
}

export interface MjData extends ClassHandle {
  solver: MjSolverStatVec;
  timer: MjTimerStatVec;
  warning: MjWarningStatVec;
  readonly contact: MjContactVec;
  nplugin: number;
  maxuse_con: number;
  maxuse_efc: number;
  ncon: number;
  ne: number;
  nf: number;
  nl: number;
  nefc: number;
  nJ: number;
  nA: number;
  nisland: number;
  nidof: number;
  pstack: number;
  pbase: number;
  parena: number;
  threadpool: number;
  narena: bigint;
  nbuffer: bigint;
  maxuse_stack: bigint;
  maxuse_arena: bigint;
  signature: bigint;
  time: number;
  readonly maxuse_threadstack: any;
  readonly solver_niter: any;
  readonly solver_nnz: any;
  readonly solver_fwdinv: any;
  readonly energy: any;
  readonly buffer: any;
  readonly arena: any;
  readonly qpos: any;
  readonly qvel: any;
  readonly act: any;
  readonly qacc_warmstart: any;
  readonly plugin_state: any;
  readonly ctrl: any;
  readonly qfrc_applied: any;
  readonly xfrc_applied: any;
  readonly eq_active: any;
  readonly mocap_pos: any;
  readonly mocap_quat: any;
  readonly qacc: any;
  readonly act_dot: any;
  readonly userdata: any;
  readonly sensordata: any;
  readonly plugin: any;
  readonly plugin_data: any;
  readonly xpos: any;
  readonly xquat: any;
  readonly xmat: any;
  readonly xipos: any;
  readonly ximat: any;
  readonly xanchor: any;
  readonly xaxis: any;
  readonly geom_xpos: any;
  readonly geom_xmat: any;
  readonly site_xpos: any;
  readonly site_xmat: any;
  readonly cam_xpos: any;
  readonly cam_xmat: any;
  readonly light_xpos: any;
  readonly light_xdir: any;
  readonly subtree_com: any;
  readonly cdof: any;
  readonly cinert: any;
  readonly flexvert_xpos: any;
  readonly flexelem_aabb: any;
  readonly flexedge_J_rownnz: any;
  readonly flexedge_J_rowadr: any;
  readonly flexedge_J_colind: any;
  readonly flexedge_J: any;
  readonly flexedge_length: any;
  readonly bvh_aabb_dyn: any;
  readonly ten_wrapadr: any;
  readonly ten_wrapnum: any;
  readonly ten_J_rownnz: any;
  readonly ten_J_rowadr: any;
  readonly ten_J_colind: any;
  readonly ten_J: any;
  readonly ten_length: any;
  readonly wrap_obj: any;
  readonly wrap_xpos: any;
  readonly actuator_length: any;
  readonly moment_rownnz: any;
  readonly moment_rowadr: any;
  readonly moment_colind: any;
  readonly actuator_moment: any;
  readonly crb: any;
  readonly qM: any;
  readonly M: any;
  readonly qLD: any;
  readonly qLDiagInv: any;
  readonly bvh_active: any;
  readonly flexedge_velocity: any;
  readonly ten_velocity: any;
  readonly actuator_velocity: any;
  readonly cvel: any;
  readonly cdof_dot: any;
  readonly qfrc_bias: any;
  readonly qfrc_spring: any;
  readonly qfrc_damper: any;
  readonly qfrc_gravcomp: any;
  readonly qfrc_fluid: any;
  readonly qfrc_passive: any;
  readonly subtree_linvel: any;
  readonly subtree_angmom: any;
  readonly qH: any;
  readonly qHDiagInv: any;
  readonly qDeriv: any;
  readonly qLU: any;
  readonly actuator_force: any;
  readonly qfrc_actuator: any;
  readonly qfrc_smooth: any;
  readonly qacc_smooth: any;
  readonly qfrc_constraint: any;
  readonly qfrc_inverse: any;
  readonly cacc: any;
  readonly cfrc_int: any;
  readonly cfrc_ext: any;
  readonly efc_type: any;
  readonly efc_id: any;
  readonly efc_J_rownnz: any;
  readonly efc_J_rowadr: any;
  readonly efc_J_rowsuper: any;
  readonly efc_J_colind: any;
  readonly efc_J: any;
  readonly efc_pos: any;
  readonly efc_margin: any;
  readonly efc_frictionloss: any;
  readonly efc_diagApprox: any;
  readonly efc_KBIP: any;
  readonly efc_D: any;
  readonly efc_R: any;
  readonly tendon_efcadr: any;
  readonly dof_island: any;
  readonly island_nv: any;
  readonly island_idofadr: any;
  readonly island_dofadr: any;
  readonly map_dof2idof: any;
  readonly map_idof2dof: any;
  readonly ifrc_smooth: any;
  readonly iacc_smooth: any;
  readonly iM_rownnz: any;
  readonly iM_rowadr: any;
  readonly iM_colind: any;
  readonly iM: any;
  readonly iLD: any;
  readonly iLDiagInv: any;
  readonly iacc: any;
  readonly efc_island: any;
  readonly island_ne: any;
  readonly island_nf: any;
  readonly island_nefc: any;
  readonly island_iefcadr: any;
  readonly map_efc2iefc: any;
  readonly map_iefc2efc: any;
  readonly iefc_type: any;
  readonly iefc_id: any;
  readonly iefc_J_rownnz: any;
  readonly iefc_J_rowadr: any;
  readonly iefc_J_rowsuper: any;
  readonly iefc_J_colind: any;
  readonly iefc_J: any;
  readonly iefc_frictionloss: any;
  readonly iefc_D: any;
  readonly iefc_R: any;
  readonly efc_AR_rownnz: any;
  readonly efc_AR_rowadr: any;
  readonly efc_AR_colind: any;
  readonly efc_AR: any;
  readonly efc_vel: any;
  readonly efc_aref: any;
  readonly efc_b: any;
  readonly iefc_aref: any;
  readonly iefc_state: any;
  readonly iefc_force: any;
  readonly efc_state: any;
  readonly efc_force: any;
  readonly ifrc_constraint: any;
}

export interface MjOption extends ClassHandle {
  integrator: number;
  cone: number;
  jacobian: number;
  solver: number;
  iterations: number;
  ls_iterations: number;
  noslip_iterations: number;
  ccd_iterations: number;
  disableflags: number;
  enableflags: number;
  disableactuator: number;
  sdf_initpoints: number;
  sdf_iterations: number;
  timestep: number;
  impratio: number;
  tolerance: number;
  ls_tolerance: number;
  noslip_tolerance: number;
  ccd_tolerance: number;
  density: number;
  viscosity: number;
  o_margin: number;
  readonly gravity: any;
  readonly wind: any;
  readonly magnetic: any;
  readonly o_solref: any;
  readonly o_solimp: any;
  readonly o_friction: any;
  copy(): MjOption;
}

export interface MjStatistic extends ClassHandle {
  meaninertia: number;
  meanmass: number;
  meansize: number;
  extent: number;
  readonly center: any;
  copy(): MjStatistic;
}

export interface MjVisualGlobal extends ClassHandle {
  cameraid: number;
  orthographic: number;
  offwidth: number;
  offheight: number;
  ellipsoidinertia: number;
  bvactive: number;
  fovy: number;
  ipd: number;
  azimuth: number;
  elevation: number;
  linewidth: number;
  glow: number;
  realtime: number;
  copy(): MjVisualGlobal;
}

export interface MjVisualQuality extends ClassHandle {
  shadowsize: number;
  offsamples: number;
  numslices: number;
  numstacks: number;
  numquads: number;
  copy(): MjVisualQuality;
}

export interface MjVisualHeadlight extends ClassHandle {
  active: number;
  readonly ambient: any;
  readonly diffuse: any;
  readonly specular: any;
  copy(): MjVisualHeadlight;
}

export interface MjVisualMap extends ClassHandle {
  stiffness: number;
  stiffnessrot: number;
  force: number;
  torque: number;
  alpha: number;
  fogstart: number;
  fogend: number;
  znear: number;
  zfar: number;
  haze: number;
  shadowclip: number;
  shadowscale: number;
  actuatortendon: number;
  copy(): MjVisualMap;
}

export interface MjVisualScale extends ClassHandle {
  forcewidth: number;
  contactwidth: number;
  contactheight: number;
  connect: number;
  com: number;
  camera: number;
  light: number;
  selectpoint: number;
  jointlength: number;
  jointwidth: number;
  actuatorlength: number;
  actuatorwidth: number;
  framelength: number;
  framewidth: number;
  constraint: number;
  slidercrank: number;
  frustum: number;
  copy(): MjVisualScale;
}

export interface MjVisualRgba extends ClassHandle {
  readonly fog: any;
  readonly haze: any;
  readonly force: any;
  readonly inertia: any;
  readonly joint: any;
  readonly actuator: any;
  readonly actuatornegative: any;
  readonly actuatorpositive: any;
  readonly com: any;
  readonly camera: any;
  readonly light: any;
  readonly selectpoint: any;
  readonly connect: any;
  readonly contactpoint: any;
  readonly contactforce: any;
  readonly contactfriction: any;
  readonly contacttorque: any;
  readonly contactgap: any;
  readonly rangefinder: any;
  readonly constraint: any;
  readonly slidercrank: any;
  readonly crankbroken: any;
  readonly frustum: any;
  readonly bv: any;
  readonly bvactive: any;
  copy(): MjVisualRgba;
}

export interface MjVisual extends ClassHandle {
  global: MjVisualGlobal;
  quality: MjVisualQuality;
  headlight: MjVisualHeadlight;
  map: MjVisualMap;
  scale: MjVisualScale;
  rgba: MjVisualRgba;
  copy(): MjVisual;
}

export interface MjSolverStat extends ClassHandle {
  nactive: number;
  nchange: number;
  neval: number;
  nupdate: number;
  improvement: number;
  gradient: number;
  lineslope: number;
  copy(): MjSolverStat;
}

export interface MjTimerStat extends ClassHandle {
  number: number;
  duration: number;
  copy(): MjTimerStat;
}

export interface MjWarningStat extends ClassHandle {
  lastinfo: number;
  number: number;
  copy(): MjWarningStat;
}

export interface MjContact extends ClassHandle {
  dim: number;
  geom1: number;
  geom2: number;
  exclude: number;
  efc_address: number;
  dist: number;
  includemargin: number;
  mu: number;
  readonly pos: any;
  readonly frame: any;
  readonly friction: any;
  readonly solref: any;
  readonly solreffriction: any;
  readonly solimp: any;
  readonly H: any;
  readonly geom: any;
  readonly flex: any;
  readonly elem: any;
  readonly vert: any;
  copy(): MjContact;
}

export interface MjvPerturb extends ClassHandle {
  select: number;
  flexselect: number;
  skinselect: number;
  active: number;
  active2: number;
  localmass: number;
  scale: number;
  readonly refpos: any;
  readonly refquat: any;
  readonly refselpos: any;
  readonly localpos: any;
  copy(): MjvPerturb;
}

export interface MjvCamera extends ClassHandle {
  type: number;
  fixedcamid: number;
  trackbodyid: number;
  orthographic: number;
  distance: number;
  azimuth: number;
  elevation: number;
  readonly lookat: any;
  copy(): MjvCamera;
}

export interface MjvGLCamera extends ClassHandle {
  orthographic: number;
  frustum_center: number;
  frustum_width: number;
  frustum_bottom: number;
  frustum_top: number;
  frustum_near: number;
  frustum_far: number;
  readonly pos: any;
  readonly forward: any;
  readonly up: any;
  copy(): MjvGLCamera;
  copy(): MjvGLCamera;
}

export interface MjvGeom extends ClassHandle {
  transparent: number;
  type: number;
  dataid: number;
  objtype: number;
  objid: number;
  category: number;
  matid: number;
  texcoord: number;
  segid: number;
  emission: number;
  specular: number;
  shininess: number;
  reflectance: number;
  camdist: number;
  modelrbound: number;
  readonly size: any;
  readonly pos: any;
  readonly mat: any;
  readonly rgba: any;
  readonly label: any;
}

export interface MjvLight extends ClassHandle {
  headlight: number;
  castshadow: number;
  id: number;
  type: number;
  texid: number;
  cutoff: number;
  exponent: number;
  bulbradius: number;
  intensity: number;
  range: number;
  readonly pos: any;
  readonly dir: any;
  readonly attenuation: any;
  readonly ambient: any;
  readonly diffuse: any;
  readonly specular: any;
  copy(): MjvLight;
}

export interface MjvOption extends ClassHandle {
  label: number;
  frame: number;
  bvh_depth: number;
  flex_layer: number;
  readonly geomgroup: any;
  readonly sitegroup: any;
  readonly jointgroup: any;
  readonly tendongroup: any;
  readonly actuatorgroup: any;
  readonly flexgroup: any;
  readonly skingroup: any;
  readonly flags: any;
  copy(): MjvOption;
}

export interface MjvScene extends ClassHandle {
  lights: MjvLightVec;
  camera: MjvGLCameraVec;
  readonly geoms: MjvGeomVec;
  flexvertopt: number;
  flexedgeopt: number;
  flexfaceopt: number;
  flexskinopt: number;
  enabletransform: number;
  maxgeom: number;
  ngeom: number;
  nflex: number;
  nskin: number;
  nlight: number;
  stereo: number;
  framewidth: number;
  status: number;
  scale: number;
  readonly geomorder: any;
  readonly flexedgeadr: any;
  readonly flexedgenum: any;
  readonly flexvertadr: any;
  readonly flexvertnum: any;
  readonly flexfaceadr: any;
  readonly flexfacenum: any;
  readonly flexfaceused: any;
  readonly flexedge: any;
  readonly flexvert: any;
  readonly flexface: any;
  readonly flexnormal: any;
  readonly flextexcoord: any;
  readonly skinfacenum: any;
  readonly skinvertadr: any;
  readonly skinvertnum: any;
  readonly skinvert: any;
  readonly skinnormal: any;
  readonly translate: any;
  readonly rotate: any;
  readonly flags: any;
  readonly framergb: any;
}

export interface MjvFigure extends ClassHandle {
  flg_legend: number;
  flg_extend: number;
  flg_barplot: number;
  flg_selection: number;
  flg_symmetric: number;
  legendoffset: number;
  subplot: number;
  highlightid: number;
  linewidth: number;
  gridwidth: number;
  selection: number;
  readonly flg_ticklabel: any;
  readonly gridsize: any;
  readonly gridrgb: any;
  readonly figurergba: any;
  readonly panergba: any;
  readonly legendrgba: any;
  readonly textrgb: any;
  readonly linergb: any;
  readonly range: any;
  readonly xformat: any;
  readonly yformat: any;
  readonly minwidth: any;
  readonly title: any;
  readonly xlabel: any;
  readonly linename: any;
  readonly highlight: any;
  readonly linepnt: any;
  readonly linedata: any;
  readonly xaxispixel: any;
  readonly yaxispixel: any;
  readonly xaxisdata: any;
  readonly yaxisdata: any;
  copy(): MjvFigure;
}

export interface MjSpec extends ClassHandle {
  option: MjOption;
  visual: MjVisual;
  stat: MjStatistic;
  element: MjsElement;
  compiler: MjsCompiler;
  strippath: number;
  hasImplicitPluginElem: number;
  nemax: number;
  nuserdata: number;
  nuser_body: number;
  nuser_jnt: number;
  nuser_geom: number;
  nuser_site: number;
  nuser_cam: number;
  nuser_tendon: number;
  nuser_actuator: number;
  nuser_sensor: number;
  nkey: number;
  njmax: number;
  nconmax: number;
  memory: bigint;
  nstack: bigint;
  get modelname(): string;
  set modelname(value: EmbindString);
  get comment(): string;
  set comment(value: EmbindString);
  get modelfiledir(): string;
  set modelfiledir(value: EmbindString);
}

export interface MjsElement extends ClassHandle {
  elemtype: mjtObj;
  signature: bigint;
}

export interface MjsCompiler extends ClassHandle {
  LRopt: MjLROpt;
  autolimits: number;
  balanceinertia: number;
  fitaabb: number;
  degree: number;
  discardvisual: number;
  usethread: number;
  fusestatic: number;
  saveinertial: number;
  inertiafromgeom: number;
  alignfree: number;
  boundmass: number;
  boundinertia: number;
  settotalmass: number;
  get meshdir(): string;
  set meshdir(value: EmbindString);
  get texturedir(): string;
  set texturedir(value: EmbindString);
  readonly eulerseq: any;
  readonly inertiagrouprange: any;
}

export interface MjsOrientation extends ClassHandle {
  type: mjtOrientation;
  readonly axisangle: any;
  readonly xyaxes: any;
  readonly zaxis: any;
  readonly euler: any;
  copy(): MjsOrientation;
}

export interface MjsBody extends ClassHandle {
  element: MjsElement;
  alt: MjsOrientation;
  ialt: MjsOrientation;
  plugin: MjsPlugin;
  readonly userdata: mjDoubleVec;
  mocap: number;
  explicitinertial: number;
  mass: number;
  gravcomp: number;
  get childclass(): string;
  set childclass(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly quat: any;
  readonly ipos: any;
  readonly iquat: any;
  readonly inertia: any;
  readonly fullinertia: any;
}

export interface MjsGeom extends ClassHandle {
  element: MjsElement;
  type: mjtGeom;
  alt: MjsOrientation;
  typeinertia: mjtGeomInertia;
  plugin: MjsPlugin;
  readonly userdata: mjDoubleVec;
  contype: number;
  conaffinity: number;
  condim: number;
  priority: number;
  group: number;
  solmix: number;
  margin: number;
  gap: number;
  mass: number;
  density: number;
  fluid_ellipsoid: number;
  fitscale: number;
  get material(): string;
  set material(value: EmbindString);
  get hfieldname(): string;
  set hfieldname(value: EmbindString);
  get meshname(): string;
  set meshname(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly quat: any;
  readonly fromto: any;
  readonly size: any;
  readonly friction: any;
  readonly solref: any;
  readonly solimp: any;
  readonly fluid_coefs: any;
  readonly rgba: any;
}

export interface MjsFrame extends ClassHandle {
  element: MjsElement;
  alt: MjsOrientation;
  get childclass(): string;
  set childclass(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly quat: any;
}

export interface MjsJoint extends ClassHandle {
  element: MjsElement;
  type: mjtJoint;
  readonly userdata: mjDoubleVec;
  actgravcomp: number;
  align: number;
  limited: number;
  actfrclimited: number;
  group: number;
  ref: number;
  stiffness: number;
  springref: number;
  margin: number;
  armature: number;
  damping: number;
  frictionloss: number;
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly axis: any;
  readonly springdamper: any;
  readonly range: any;
  readonly solref_limit: any;
  readonly solimp_limit: any;
  readonly actfrcrange: any;
  readonly solref_friction: any;
  readonly solimp_friction: any;
}

export interface MjsSite extends ClassHandle {
  element: MjsElement;
  alt: MjsOrientation;
  type: mjtGeom;
  readonly userdata: mjDoubleVec;
  group: number;
  get material(): string;
  set material(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly quat: any;
  readonly fromto: any;
  readonly size: any;
  readonly rgba: any;
}

export interface MjsCamera extends ClassHandle {
  element: MjsElement;
  alt: MjsOrientation;
  mode: mjtCamLight;
  readonly userdata: mjDoubleVec;
  orthographic: number;
  fovy: number;
  ipd: number;
  get targetbody(): string;
  set targetbody(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly quat: any;
  readonly intrinsic: any;
  readonly sensor_size: any;
  readonly resolution: any;
  readonly focal_length: any;
  readonly focal_pixel: any;
  readonly principal_length: any;
  readonly principal_pixel: any;
}

export interface MjsLight extends ClassHandle {
  element: MjsElement;
  mode: mjtCamLight;
  type: mjtLightType;
  active: number;
  castshadow: number;
  bulbradius: number;
  intensity: number;
  range: number;
  cutoff: number;
  exponent: number;
  get targetbody(): string;
  set targetbody(value: EmbindString);
  get texture(): string;
  set texture(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly pos: any;
  readonly dir: any;
  readonly attenuation: any;
  readonly ambient: any;
  readonly diffuse: any;
  readonly specular: any;
}

export interface MjsFlex extends ClassHandle {
  element: MjsElement;
  readonly nodebody: mjStringVec;
  readonly vertbody: mjStringVec;
  readonly elem: mjIntVec;
  readonly elemtexcoord: mjIntVec;
  readonly texcoord: mjFloatVec;
  readonly node: mjDoubleVec;
  readonly vert: mjDoubleVec;
  internal: number;
  flatskin: number;
  contype: number;
  conaffinity: number;
  condim: number;
  priority: number;
  dim: number;
  selfcollide: number;
  vertcollide: number;
  passive: number;
  activelayers: number;
  group: number;
  elastic2d: number;
  solmix: number;
  margin: number;
  gap: number;
  radius: number;
  edgestiffness: number;
  edgedamping: number;
  young: number;
  poisson: number;
  damping: number;
  thickness: number;
  get material(): string;
  set material(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly friction: any;
  readonly solref: any;
  readonly solimp: any;
  readonly rgba: any;
}

export interface MjsMesh extends ClassHandle {
  element: MjsElement;
  inertia: mjtMeshInertia;
  plugin: MjsPlugin;
  readonly userface: mjIntVec;
  readonly userfacenormal: mjIntVec;
  readonly userfacetexcoord: mjIntVec;
  readonly uservert: mjFloatVec;
  readonly usernormal: mjFloatVec;
  readonly usertexcoord: mjFloatVec;
  smoothnormal: number;
  needsdf: number;
  maxhullvert: number;
  get content_type(): string;
  set content_type(value: EmbindString);
  get file(): string;
  set file(value: EmbindString);
  get material(): string;
  set material(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly refpos: any;
  readonly refquat: any;
  readonly scale: any;
}

export interface MjsHField extends ClassHandle {
  element: MjsElement;
  readonly userdata: mjFloatVec;
  nrow: number;
  ncol: number;
  get content_type(): string;
  set content_type(value: EmbindString);
  get file(): string;
  set file(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly size: any;
}

export interface MjsSkin extends ClassHandle {
  element: MjsElement;
  readonly bodyname: mjStringVec;
  readonly face: mjIntVec;
  readonly vertid: mjIntVecVec;
  readonly vert: mjFloatVec;
  readonly texcoord: mjFloatVec;
  readonly bindpos: mjFloatVec;
  readonly bindquat: mjFloatVec;
  readonly vertweight: mjFloatVecVec;
  group: number;
  inflate: number;
  get file(): string;
  set file(value: EmbindString);
  get material(): string;
  set material(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly rgba: any;
}

export interface MjsTexture extends ClassHandle {
  element: MjsElement;
  type: mjtTexture;
  colorspace: mjtColorSpace;
  readonly cubefiles: mjStringVec;
  readonly data: mjByteVec;
  hflip: number;
  vflip: number;
  builtin: number;
  mark: number;
  height: number;
  width: number;
  nchannel: number;
  random: number;
  get content_type(): string;
  set content_type(value: EmbindString);
  get file(): string;
  set file(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly rgb1: any;
  readonly rgb2: any;
  readonly markrgb: any;
  readonly gridsize: any;
  readonly gridlayout: any;
}

export interface MjsMaterial extends ClassHandle {
  element: MjsElement;
  readonly textures: mjStringVec;
  texuniform: number;
  emission: number;
  specular: number;
  shininess: number;
  reflectance: number;
  metallic: number;
  roughness: number;
  get info(): string;
  set info(value: EmbindString);
  readonly texrepeat: any;
  readonly rgba: any;
}

export interface MjsPair extends ClassHandle {
  element: MjsElement;
  condim: number;
  margin: number;
  gap: number;
  get geomname1(): string;
  set geomname1(value: EmbindString);
  get geomname2(): string;
  set geomname2(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly solref: any;
  readonly solreffriction: any;
  readonly solimp: any;
  readonly friction: any;
}

export interface MjsExclude extends ClassHandle {
  element: MjsElement;
  get bodyname1(): string;
  set bodyname1(value: EmbindString);
  get bodyname2(): string;
  set bodyname2(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsEquality extends ClassHandle {
  element: MjsElement;
  type: mjtEq;
  objtype: mjtObj;
  active: number;
  get name1(): string;
  set name1(value: EmbindString);
  get name2(): string;
  set name2(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly data: any;
  readonly solref: any;
  readonly solimp: any;
}

export interface MjsTendon extends ClassHandle {
  element: MjsElement;
  readonly userdata: mjDoubleVec;
  limited: number;
  actfrclimited: number;
  group: number;
  stiffness: number;
  damping: number;
  frictionloss: number;
  armature: number;
  margin: number;
  width: number;
  get material(): string;
  set material(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly springlength: any;
  readonly solref_friction: any;
  readonly solimp_friction: any;
  readonly range: any;
  readonly actfrcrange: any;
  readonly solref_limit: any;
  readonly solimp_limit: any;
  readonly rgba: any;
}

export interface MjsWrap extends ClassHandle {
  element: MjsElement;
  type: mjtWrap;
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsActuator extends ClassHandle {
  element: MjsElement;
  gaintype: mjtGain;
  biastype: mjtBias;
  dyntype: mjtDyn;
  trntype: mjtTrn;
  plugin: MjsPlugin;
  readonly userdata: mjDoubleVec;
  actearly: number;
  actdim: number;
  ctrllimited: number;
  forcelimited: number;
  actlimited: number;
  group: number;
  cranklength: number;
  inheritrange: number;
  get target(): string;
  set target(value: EmbindString);
  get refsite(): string;
  set refsite(value: EmbindString);
  get slidersite(): string;
  set slidersite(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly gainprm: any;
  readonly biasprm: any;
  readonly dynprm: any;
  readonly gear: any;
  readonly lengthrange: any;
  readonly ctrlrange: any;
  readonly forcerange: any;
  readonly actrange: any;
}

export interface MjsSensor extends ClassHandle {
  element: MjsElement;
  type: mjtSensor;
  objtype: mjtObj;
  reftype: mjtObj;
  datatype: mjtDataType;
  needstage: mjtStage;
  plugin: MjsPlugin;
  readonly userdata: mjDoubleVec;
  dim: number;
  cutoff: number;
  noise: number;
  get objname(): string;
  set objname(value: EmbindString);
  get refname(): string;
  set refname(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
  readonly intprm: any;
}

export interface MjsNumeric extends ClassHandle {
  element: MjsElement;
  readonly data: mjDoubleVec;
  size: number;
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsText extends ClassHandle {
  element: MjsElement;
  get data(): string;
  set data(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsTuple extends ClassHandle {
  element: MjsElement;
  readonly objname: mjStringVec;
  readonly objtype: mjIntVec;
  readonly objprm: mjDoubleVec;
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsKey extends ClassHandle {
  element: MjsElement;
  readonly qpos: mjDoubleVec;
  readonly qvel: mjDoubleVec;
  readonly act: mjDoubleVec;
  readonly mpos: mjDoubleVec;
  readonly mquat: mjDoubleVec;
  readonly ctrl: mjDoubleVec;
  time: number;
  get info(): string;
  set info(value: EmbindString);
}

export interface MjsDefault extends ClassHandle {
  element: MjsElement;
  joint: MjsJoint;
  geom: MjsGeom;
  site: MjsSite;
  camera: MjsCamera;
  light: MjsLight;
  flex: MjsFlex;
  mesh: MjsMesh;
  material: MjsMaterial;
  pair: MjsPair;
  equality: MjsEquality;
  tendon: MjsTendon;
  actuator: MjsActuator;
}

export interface MjsPlugin extends ClassHandle {
  element: MjsElement;
  active: number;
  get name(): string;
  set name(value: EmbindString);
  get plugin_name(): string;
  set plugin_name(value: EmbindString);
  get info(): string;
  set info(value: EmbindString);
}

export interface MjVFS extends ClassHandle {
}

export interface MjSolverStatVec extends ClassHandle {
  push_back(_0: MjSolverStat): void;
  resize(_0: number, _1: MjSolverStat): void;
  size(): number;
  get(_0: number): MjSolverStat | undefined;
  set(_0: number, _1: MjSolverStat): boolean;
}

export interface MjTimerStatVec extends ClassHandle {
  push_back(_0: MjTimerStat): void;
  resize(_0: number, _1: MjTimerStat): void;
  size(): number;
  get(_0: number): MjTimerStat | undefined;
  set(_0: number, _1: MjTimerStat): boolean;
}

export interface MjWarningStatVec extends ClassHandle {
  push_back(_0: MjWarningStat): void;
  resize(_0: number, _1: MjWarningStat): void;
  size(): number;
  get(_0: number): MjWarningStat | undefined;
  set(_0: number, _1: MjWarningStat): boolean;
}

export interface MjContactVec extends ClassHandle {
  push_back(_0: MjContact): void;
  resize(_0: number, _1: MjContact): void;
  size(): number;
  get(_0: number): MjContact | undefined;
  set(_0: number, _1: MjContact): boolean;
}

export interface MjvLightVec extends ClassHandle {
  push_back(_0: MjvLight): void;
  resize(_0: number, _1: MjvLight): void;
  size(): number;
  get(_0: number): MjvLight | undefined;
  set(_0: number, _1: MjvLight): boolean;
}

export interface MjvGLCameraVec extends ClassHandle {
  push_back(_0: MjvGLCamera): void;
  resize(_0: number, _1: MjvGLCamera): void;
  size(): number;
  get(_0: number): MjvGLCamera | undefined;
  set(_0: number, _1: MjvGLCamera): boolean;
}

export interface MjvGeomVec extends ClassHandle {
  push_back(_0: MjvGeom): void;
  resize(_0: number, _1: MjvGeom): void;
  size(): number;
  get(_0: number): MjvGeom | undefined;
  set(_0: number, _1: MjvGeom): boolean;
}

export interface FloatBuffer extends ClassHandle {
  GetElementCount(): number;
  GetPointer(): number;
  GetView(): any;
}

export interface DoubleBuffer extends ClassHandle {
  GetElementCount(): number;
  GetPointer(): number;
  GetView(): any;
}

export interface IntBuffer extends ClassHandle {
  GetElementCount(): number;
  GetPointer(): number;
  GetView(): any;
}

export interface mjStringVec extends ClassHandle {
  size(): number;
  get(_0: number): EmbindString | undefined;
  push_back(_0: EmbindString): void;
  resize(_0: number, _1: EmbindString): void;
  set(_0: number, _1: EmbindString): boolean;
}

export interface mjIntVec extends ClassHandle {
  push_back(_0: number): void;
  resize(_0: number, _1: number): void;
  size(): number;
  get(_0: number): number | undefined;
  set(_0: number, _1: number): boolean;
}

export interface mjIntVecVec extends ClassHandle {
  push_back(_0: mjIntVec): void;
  resize(_0: number, _1: mjIntVec): void;
  size(): number;
  get(_0: number): mjIntVec | undefined;
  set(_0: number, _1: mjIntVec): boolean;
}

export interface mjFloatVec extends ClassHandle {
  size(): number;
  get(_0: number): number | undefined;
  push_back(_0: number): void;
  resize(_0: number, _1: number): void;
  set(_0: number, _1: number): boolean;
}

export interface mjFloatVecVec extends ClassHandle {
  push_back(_0: mjFloatVec): void;
  resize(_0: number, _1: mjFloatVec): void;
  size(): number;
  get(_0: number): mjFloatVec | undefined;
  set(_0: number, _1: mjFloatVec): boolean;
}

export interface mjDoubleVec extends ClassHandle {
  size(): number;
  get(_0: number): number | undefined;
  push_back(_0: number): void;
  resize(_0: number, _1: number): void;
  set(_0: number, _1: number): boolean;
}

export interface mjByteVec extends ClassHandle {
  push_back(_0: number): void;
  resize(_0: number, _1: number): void;
  size(): number;
  get(_0: number): number | undefined;
  set(_0: number, _1: number): boolean;
}

interface EmbindModule {
  mjtDisableBit: {mjDSBL_CONSTRAINT: mjtDisableBitValue<1>, mjDSBL_EQUALITY: mjtDisableBitValue<2>, mjDSBL_FRICTIONLOSS: mjtDisableBitValue<4>, mjDSBL_LIMIT: mjtDisableBitValue<8>, mjDSBL_CONTACT: mjtDisableBitValue<16>, mjDSBL_SPRING: mjtDisableBitValue<32>, mjDSBL_DAMPER: mjtDisableBitValue<64>, mjDSBL_GRAVITY: mjtDisableBitValue<128>, mjDSBL_CLAMPCTRL: mjtDisableBitValue<256>, mjDSBL_WARMSTART: mjtDisableBitValue<512>, mjDSBL_FILTERPARENT: mjtDisableBitValue<1024>, mjDSBL_ACTUATION: mjtDisableBitValue<2048>, mjDSBL_REFSAFE: mjtDisableBitValue<4096>, mjDSBL_SENSOR: mjtDisableBitValue<8192>, mjDSBL_MIDPHASE: mjtDisableBitValue<16384>, mjDSBL_EULERDAMP: mjtDisableBitValue<32768>, mjDSBL_AUTORESET: mjtDisableBitValue<65536>, mjDSBL_NATIVECCD: mjtDisableBitValue<131072>, mjDSBL_ISLAND: mjtDisableBitValue<262144>, mjNDISABLE: mjtDisableBitValue<19>};
  mjtEnableBit: {mjENBL_OVERRIDE: mjtEnableBitValue<1>, mjENBL_ENERGY: mjtEnableBitValue<2>, mjENBL_FWDINV: mjtEnableBitValue<4>, mjENBL_INVDISCRETE: mjtEnableBitValue<8>, mjENBL_MULTICCD: mjtEnableBitValue<16>, mjNENABLE: mjtEnableBitValue<5>};
  mjtJoint: {mjJNT_FREE: mjtJointValue<0>, mjJNT_BALL: mjtJointValue<1>, mjJNT_SLIDE: mjtJointValue<2>, mjJNT_HINGE: mjtJointValue<3>};
  mjtGeom: {mjGEOM_PLANE: mjtGeomValue<0>, mjGEOM_HFIELD: mjtGeomValue<1>, mjGEOM_SPHERE: mjtGeomValue<2>, mjGEOM_CAPSULE: mjtGeomValue<3>, mjGEOM_ELLIPSOID: mjtGeomValue<4>, mjGEOM_CYLINDER: mjtGeomValue<5>, mjGEOM_BOX: mjtGeomValue<6>, mjGEOM_MESH: mjtGeomValue<7>, mjGEOM_SDF: mjtGeomValue<8>, mjNGEOMTYPES: mjtGeomValue<9>, mjGEOM_ARROW: mjtGeomValue<100>, mjGEOM_ARROW1: mjtGeomValue<101>, mjGEOM_ARROW2: mjtGeomValue<102>, mjGEOM_LINE: mjtGeomValue<103>, mjGEOM_LINEBOX: mjtGeomValue<104>, mjGEOM_FLEX: mjtGeomValue<105>, mjGEOM_SKIN: mjtGeomValue<106>, mjGEOM_LABEL: mjtGeomValue<107>, mjGEOM_TRIANGLE: mjtGeomValue<108>, mjGEOM_NONE: mjtGeomValue<1001>};
  mjtCamLight: {mjCAMLIGHT_FIXED: mjtCamLightValue<0>, mjCAMLIGHT_TRACK: mjtCamLightValue<1>, mjCAMLIGHT_TRACKCOM: mjtCamLightValue<2>, mjCAMLIGHT_TARGETBODY: mjtCamLightValue<3>, mjCAMLIGHT_TARGETBODYCOM: mjtCamLightValue<4>};
  mjtLightType: {mjLIGHT_SPOT: mjtLightTypeValue<0>, mjLIGHT_DIRECTIONAL: mjtLightTypeValue<1>, mjLIGHT_POINT: mjtLightTypeValue<2>, mjLIGHT_IMAGE: mjtLightTypeValue<3>};
  mjtTexture: {mjTEXTURE_2D: mjtTextureValue<0>, mjTEXTURE_CUBE: mjtTextureValue<1>, mjTEXTURE_SKYBOX: mjtTextureValue<2>};
  mjtTextureRole: {mjTEXROLE_USER: mjtTextureRoleValue<0>, mjTEXROLE_RGB: mjtTextureRoleValue<1>, mjTEXROLE_OCCLUSION: mjtTextureRoleValue<2>, mjTEXROLE_ROUGHNESS: mjtTextureRoleValue<3>, mjTEXROLE_METALLIC: mjtTextureRoleValue<4>, mjTEXROLE_NORMAL: mjtTextureRoleValue<5>, mjTEXROLE_OPACITY: mjtTextureRoleValue<6>, mjTEXROLE_EMISSIVE: mjtTextureRoleValue<7>, mjTEXROLE_RGBA: mjtTextureRoleValue<8>, mjTEXROLE_ORM: mjtTextureRoleValue<9>, mjNTEXROLE: mjtTextureRoleValue<10>};
  mjtColorSpace: {mjCOLORSPACE_AUTO: mjtColorSpaceValue<0>, mjCOLORSPACE_LINEAR: mjtColorSpaceValue<1>, mjCOLORSPACE_SRGB: mjtColorSpaceValue<2>};
  mjtIntegrator: {mjINT_EULER: mjtIntegratorValue<0>, mjINT_RK4: mjtIntegratorValue<1>, mjINT_IMPLICIT: mjtIntegratorValue<2>, mjINT_IMPLICITFAST: mjtIntegratorValue<3>};
  mjtCone: {mjCONE_PYRAMIDAL: mjtConeValue<0>, mjCONE_ELLIPTIC: mjtConeValue<1>};
  mjtJacobian: {mjJAC_DENSE: mjtJacobianValue<0>, mjJAC_SPARSE: mjtJacobianValue<1>, mjJAC_AUTO: mjtJacobianValue<2>};
  mjtSolver: {mjSOL_PGS: mjtSolverValue<0>, mjSOL_CG: mjtSolverValue<1>, mjSOL_NEWTON: mjtSolverValue<2>};
  mjtEq: {mjEQ_CONNECT: mjtEqValue<0>, mjEQ_WELD: mjtEqValue<1>, mjEQ_JOINT: mjtEqValue<2>, mjEQ_TENDON: mjtEqValue<3>, mjEQ_FLEX: mjtEqValue<4>, mjEQ_DISTANCE: mjtEqValue<5>};
  mjtWrap: {mjWRAP_NONE: mjtWrapValue<0>, mjWRAP_JOINT: mjtWrapValue<1>, mjWRAP_PULLEY: mjtWrapValue<2>, mjWRAP_SITE: mjtWrapValue<3>, mjWRAP_SPHERE: mjtWrapValue<4>, mjWRAP_CYLINDER: mjtWrapValue<5>};
  mjtTrn: {mjTRN_JOINT: mjtTrnValue<0>, mjTRN_JOINTINPARENT: mjtTrnValue<1>, mjTRN_SLIDERCRANK: mjtTrnValue<2>, mjTRN_TENDON: mjtTrnValue<3>, mjTRN_SITE: mjtTrnValue<4>, mjTRN_BODY: mjtTrnValue<5>, mjTRN_UNDEFINED: mjtTrnValue<1000>};
  mjtDyn: {mjDYN_NONE: mjtDynValue<0>, mjDYN_INTEGRATOR: mjtDynValue<1>, mjDYN_FILTER: mjtDynValue<2>, mjDYN_FILTEREXACT: mjtDynValue<3>, mjDYN_MUSCLE: mjtDynValue<4>, mjDYN_USER: mjtDynValue<5>};
  mjtGain: {mjGAIN_FIXED: mjtGainValue<0>, mjGAIN_AFFINE: mjtGainValue<1>, mjGAIN_MUSCLE: mjtGainValue<2>, mjGAIN_USER: mjtGainValue<3>};
  mjtBias: {mjBIAS_NONE: mjtBiasValue<0>, mjBIAS_AFFINE: mjtBiasValue<1>, mjBIAS_MUSCLE: mjtBiasValue<2>, mjBIAS_USER: mjtBiasValue<3>};
  mjtObj: {mjOBJ_UNKNOWN: mjtObjValue<0>, mjOBJ_BODY: mjtObjValue<1>, mjOBJ_XBODY: mjtObjValue<2>, mjOBJ_JOINT: mjtObjValue<3>, mjOBJ_DOF: mjtObjValue<4>, mjOBJ_GEOM: mjtObjValue<5>, mjOBJ_SITE: mjtObjValue<6>, mjOBJ_CAMERA: mjtObjValue<7>, mjOBJ_LIGHT: mjtObjValue<8>, mjOBJ_FLEX: mjtObjValue<9>, mjOBJ_MESH: mjtObjValue<10>, mjOBJ_SKIN: mjtObjValue<11>, mjOBJ_HFIELD: mjtObjValue<12>, mjOBJ_TEXTURE: mjtObjValue<13>, mjOBJ_MATERIAL: mjtObjValue<14>, mjOBJ_PAIR: mjtObjValue<15>, mjOBJ_EXCLUDE: mjtObjValue<16>, mjOBJ_EQUALITY: mjtObjValue<17>, mjOBJ_TENDON: mjtObjValue<18>, mjOBJ_ACTUATOR: mjtObjValue<19>, mjOBJ_SENSOR: mjtObjValue<20>, mjOBJ_NUMERIC: mjtObjValue<21>, mjOBJ_TEXT: mjtObjValue<22>, mjOBJ_TUPLE: mjtObjValue<23>, mjOBJ_KEY: mjtObjValue<24>, mjOBJ_PLUGIN: mjtObjValue<25>, mjNOBJECT: mjtObjValue<26>, mjOBJ_FRAME: mjtObjValue<100>, mjOBJ_DEFAULT: mjtObjValue<101>, mjOBJ_MODEL: mjtObjValue<102>};
  mjtSensor: {mjSENS_TOUCH: mjtSensorValue<0>, mjSENS_ACCELEROMETER: mjtSensorValue<1>, mjSENS_VELOCIMETER: mjtSensorValue<2>, mjSENS_GYRO: mjtSensorValue<3>, mjSENS_FORCE: mjtSensorValue<4>, mjSENS_TORQUE: mjtSensorValue<5>, mjSENS_MAGNETOMETER: mjtSensorValue<6>, mjSENS_RANGEFINDER: mjtSensorValue<7>, mjSENS_CAMPROJECTION: mjtSensorValue<8>, mjSENS_JOINTPOS: mjtSensorValue<9>, mjSENS_JOINTVEL: mjtSensorValue<10>, mjSENS_TENDONPOS: mjtSensorValue<11>, mjSENS_TENDONVEL: mjtSensorValue<12>, mjSENS_ACTUATORPOS: mjtSensorValue<13>, mjSENS_ACTUATORVEL: mjtSensorValue<14>, mjSENS_ACTUATORFRC: mjtSensorValue<15>, mjSENS_JOINTACTFRC: mjtSensorValue<16>, mjSENS_TENDONACTFRC: mjtSensorValue<17>, mjSENS_BALLQUAT: mjtSensorValue<18>, mjSENS_BALLANGVEL: mjtSensorValue<19>, mjSENS_JOINTLIMITPOS: mjtSensorValue<20>, mjSENS_JOINTLIMITVEL: mjtSensorValue<21>, mjSENS_JOINTLIMITFRC: mjtSensorValue<22>, mjSENS_TENDONLIMITPOS: mjtSensorValue<23>, mjSENS_TENDONLIMITVEL: mjtSensorValue<24>, mjSENS_TENDONLIMITFRC: mjtSensorValue<25>, mjSENS_FRAMEPOS: mjtSensorValue<26>, mjSENS_FRAMEQUAT: mjtSensorValue<27>, mjSENS_FRAMEXAXIS: mjtSensorValue<28>, mjSENS_FRAMEYAXIS: mjtSensorValue<29>, mjSENS_FRAMEZAXIS: mjtSensorValue<30>, mjSENS_FRAMELINVEL: mjtSensorValue<31>, mjSENS_FRAMEANGVEL: mjtSensorValue<32>, mjSENS_FRAMELINACC: mjtSensorValue<33>, mjSENS_FRAMEANGACC: mjtSensorValue<34>, mjSENS_SUBTREECOM: mjtSensorValue<35>, mjSENS_SUBTREELINVEL: mjtSensorValue<36>, mjSENS_SUBTREEANGMOM: mjtSensorValue<37>, mjSENS_INSIDESITE: mjtSensorValue<38>, mjSENS_GEOMDIST: mjtSensorValue<39>, mjSENS_GEOMNORMAL: mjtSensorValue<40>, mjSENS_GEOMFROMTO: mjtSensorValue<41>, mjSENS_CONTACT: mjtSensorValue<42>, mjSENS_E_POTENTIAL: mjtSensorValue<43>, mjSENS_E_KINETIC: mjtSensorValue<44>, mjSENS_CLOCK: mjtSensorValue<45>, mjSENS_TACTILE: mjtSensorValue<46>, mjSENS_PLUGIN: mjtSensorValue<47>, mjSENS_USER: mjtSensorValue<48>};
  mjtStage: {mjSTAGE_NONE: mjtStageValue<0>, mjSTAGE_POS: mjtStageValue<1>, mjSTAGE_VEL: mjtStageValue<2>, mjSTAGE_ACC: mjtStageValue<3>};
  mjtDataType: {mjDATATYPE_REAL: mjtDataTypeValue<0>, mjDATATYPE_POSITIVE: mjtDataTypeValue<1>, mjDATATYPE_AXIS: mjtDataTypeValue<2>, mjDATATYPE_QUATERNION: mjtDataTypeValue<3>};
  mjtConDataField: {mjCONDATA_FOUND: mjtConDataFieldValue<0>, mjCONDATA_FORCE: mjtConDataFieldValue<1>, mjCONDATA_TORQUE: mjtConDataFieldValue<2>, mjCONDATA_DIST: mjtConDataFieldValue<3>, mjCONDATA_POS: mjtConDataFieldValue<4>, mjCONDATA_NORMAL: mjtConDataFieldValue<5>, mjCONDATA_TANGENT: mjtConDataFieldValue<6>, mjNCONDATA: mjtConDataFieldValue<7>};
  mjtSameFrame: {mjSAMEFRAME_NONE: mjtSameFrameValue<0>, mjSAMEFRAME_BODY: mjtSameFrameValue<1>, mjSAMEFRAME_INERTIA: mjtSameFrameValue<2>, mjSAMEFRAME_BODYROT: mjtSameFrameValue<3>, mjSAMEFRAME_INERTIAROT: mjtSameFrameValue<4>};
  mjtLRMode: {mjLRMODE_NONE: mjtLRModeValue<0>, mjLRMODE_MUSCLE: mjtLRModeValue<1>, mjLRMODE_MUSCLEUSER: mjtLRModeValue<2>, mjLRMODE_ALL: mjtLRModeValue<3>};
  mjtFlexSelf: {mjFLEXSELF_NONE: mjtFlexSelfValue<0>, mjFLEXSELF_NARROW: mjtFlexSelfValue<1>, mjFLEXSELF_BVH: mjtFlexSelfValue<2>, mjFLEXSELF_SAP: mjtFlexSelfValue<3>, mjFLEXSELF_AUTO: mjtFlexSelfValue<4>};
  mjtSDFType: {mjSDFTYPE_SINGLE: mjtSDFTypeValue<0>, mjSDFTYPE_INTERSECTION: mjtSDFTypeValue<1>, mjSDFTYPE_MIDSURFACE: mjtSDFTypeValue<2>, mjSDFTYPE_COLLISION: mjtSDFTypeValue<3>};
  mjtTaskStatus: {mjTASK_NEW: mjtTaskStatusValue<0>, mjTASK_QUEUED: mjtTaskStatusValue<1>, mjTASK_COMPLETED: mjtTaskStatusValue<2>};
  mjtState: {mjSTATE_TIME: mjtStateValue<1>, mjSTATE_QPOS: mjtStateValue<2>, mjSTATE_QVEL: mjtStateValue<4>, mjSTATE_ACT: mjtStateValue<8>, mjSTATE_WARMSTART: mjtStateValue<16>, mjSTATE_CTRL: mjtStateValue<32>, mjSTATE_QFRC_APPLIED: mjtStateValue<64>, mjSTATE_XFRC_APPLIED: mjtStateValue<128>, mjSTATE_EQ_ACTIVE: mjtStateValue<256>, mjSTATE_MOCAP_POS: mjtStateValue<512>, mjSTATE_MOCAP_QUAT: mjtStateValue<1024>, mjSTATE_USERDATA: mjtStateValue<2048>, mjSTATE_PLUGIN: mjtStateValue<4096>, mjNSTATE: mjtStateValue<13>, mjSTATE_PHYSICS: mjtStateValue<14>, mjSTATE_FULLPHYSICS: mjtStateValue<4111>, mjSTATE_USER: mjtStateValue<4064>, mjSTATE_INTEGRATION: mjtStateValue<8191>};
  mjtConstraint: {mjCNSTR_EQUALITY: mjtConstraintValue<0>, mjCNSTR_FRICTION_DOF: mjtConstraintValue<1>, mjCNSTR_FRICTION_TENDON: mjtConstraintValue<2>, mjCNSTR_LIMIT_JOINT: mjtConstraintValue<3>, mjCNSTR_LIMIT_TENDON: mjtConstraintValue<4>, mjCNSTR_CONTACT_FRICTIONLESS: mjtConstraintValue<5>, mjCNSTR_CONTACT_PYRAMIDAL: mjtConstraintValue<6>, mjCNSTR_CONTACT_ELLIPTIC: mjtConstraintValue<7>};
  mjtConstraintState: {mjCNSTRSTATE_SATISFIED: mjtConstraintStateValue<0>, mjCNSTRSTATE_QUADRATIC: mjtConstraintStateValue<1>, mjCNSTRSTATE_LINEARNEG: mjtConstraintStateValue<2>, mjCNSTRSTATE_LINEARPOS: mjtConstraintStateValue<3>, mjCNSTRSTATE_CONE: mjtConstraintStateValue<4>};
  mjtWarning: {mjWARN_INERTIA: mjtWarningValue<0>, mjWARN_CONTACTFULL: mjtWarningValue<1>, mjWARN_CNSTRFULL: mjtWarningValue<2>, mjWARN_VGEOMFULL: mjtWarningValue<3>, mjWARN_BADQPOS: mjtWarningValue<4>, mjWARN_BADQVEL: mjtWarningValue<5>, mjWARN_BADQACC: mjtWarningValue<6>, mjWARN_BADCTRL: mjtWarningValue<7>, mjNWARNING: mjtWarningValue<8>};
  mjtTimer: {mjTIMER_STEP: mjtTimerValue<0>, mjTIMER_FORWARD: mjtTimerValue<1>, mjTIMER_INVERSE: mjtTimerValue<2>, mjTIMER_POSITION: mjtTimerValue<3>, mjTIMER_VELOCITY: mjtTimerValue<4>, mjTIMER_ACTUATION: mjtTimerValue<5>, mjTIMER_CONSTRAINT: mjtTimerValue<6>, mjTIMER_ADVANCE: mjtTimerValue<7>, mjTIMER_POS_KINEMATICS: mjtTimerValue<8>, mjTIMER_POS_INERTIA: mjtTimerValue<9>, mjTIMER_POS_COLLISION: mjtTimerValue<10>, mjTIMER_POS_MAKE: mjtTimerValue<11>, mjTIMER_POS_PROJECT: mjtTimerValue<12>, mjTIMER_COL_BROAD: mjtTimerValue<13>, mjTIMER_COL_NARROW: mjtTimerValue<14>, mjNTIMER: mjtTimerValue<15>};
  mjtGeomInertia: {mjINERTIA_VOLUME: mjtGeomInertiaValue<0>, mjINERTIA_SHELL: mjtGeomInertiaValue<1>};
  mjtMeshInertia: {mjMESH_INERTIA_CONVEX: mjtMeshInertiaValue<0>, mjMESH_INERTIA_EXACT: mjtMeshInertiaValue<1>, mjMESH_INERTIA_LEGACY: mjtMeshInertiaValue<2>, mjMESH_INERTIA_SHELL: mjtMeshInertiaValue<3>};
  mjtMeshBuiltin: {mjMESH_BUILTIN_NONE: mjtMeshBuiltinValue<0>, mjMESH_BUILTIN_SPHERE: mjtMeshBuiltinValue<1>, mjMESH_BUILTIN_HEMISPHERE: mjtMeshBuiltinValue<2>, mjMESH_BUILTIN_CONE: mjtMeshBuiltinValue<3>, mjMESH_BUILTIN_SUPERSPHERE: mjtMeshBuiltinValue<4>, mjMESH_BUILTIN_SUPERTORUS: mjtMeshBuiltinValue<5>, mjMESH_BUILTIN_WEDGE: mjtMeshBuiltinValue<6>, mjMESH_BUILTIN_PLATE: mjtMeshBuiltinValue<7>};
  mjtBuiltin: {mjBUILTIN_NONE: mjtBuiltinValue<0>, mjBUILTIN_GRADIENT: mjtBuiltinValue<1>, mjBUILTIN_CHECKER: mjtBuiltinValue<2>, mjBUILTIN_FLAT: mjtBuiltinValue<3>};
  mjtMark: {mjMARK_NONE: mjtMarkValue<0>, mjMARK_EDGE: mjtMarkValue<1>, mjMARK_CROSS: mjtMarkValue<2>, mjMARK_RANDOM: mjtMarkValue<3>};
  mjtLimited: {mjLIMITED_FALSE: mjtLimitedValue<0>, mjLIMITED_TRUE: mjtLimitedValue<1>, mjLIMITED_AUTO: mjtLimitedValue<2>};
  mjtAlignFree: {mjALIGNFREE_FALSE: mjtAlignFreeValue<0>, mjALIGNFREE_TRUE: mjtAlignFreeValue<1>, mjALIGNFREE_AUTO: mjtAlignFreeValue<2>};
  mjtInertiaFromGeom: {mjINERTIAFROMGEOM_FALSE: mjtInertiaFromGeomValue<0>, mjINERTIAFROMGEOM_TRUE: mjtInertiaFromGeomValue<1>, mjINERTIAFROMGEOM_AUTO: mjtInertiaFromGeomValue<2>};
  mjtOrientation: {mjORIENTATION_QUAT: mjtOrientationValue<0>, mjORIENTATION_AXISANGLE: mjtOrientationValue<1>, mjORIENTATION_XYAXES: mjtOrientationValue<2>, mjORIENTATION_ZAXIS: mjtOrientationValue<3>, mjORIENTATION_EULER: mjtOrientationValue<4>};
  mjtCatBit: {mjCAT_STATIC: mjtCatBitValue<1>, mjCAT_DYNAMIC: mjtCatBitValue<2>, mjCAT_DECOR: mjtCatBitValue<4>, mjCAT_ALL: mjtCatBitValue<7>};
  mjtMouse: {mjMOUSE_NONE: mjtMouseValue<0>, mjMOUSE_ROTATE_V: mjtMouseValue<1>, mjMOUSE_ROTATE_H: mjtMouseValue<2>, mjMOUSE_MOVE_V: mjtMouseValue<3>, mjMOUSE_MOVE_H: mjtMouseValue<4>, mjMOUSE_ZOOM: mjtMouseValue<5>, mjMOUSE_MOVE_V_REL: mjtMouseValue<6>, mjMOUSE_MOVE_H_REL: mjtMouseValue<7>};
  mjtPertBit: {mjPERT_TRANSLATE: mjtPertBitValue<1>, mjPERT_ROTATE: mjtPertBitValue<2>};
  mjtCamera: {mjCAMERA_FREE: mjtCameraValue<0>, mjCAMERA_TRACKING: mjtCameraValue<1>, mjCAMERA_FIXED: mjtCameraValue<2>, mjCAMERA_USER: mjtCameraValue<3>};
  mjtLabel: {mjLABEL_NONE: mjtLabelValue<0>, mjLABEL_BODY: mjtLabelValue<1>, mjLABEL_JOINT: mjtLabelValue<2>, mjLABEL_GEOM: mjtLabelValue<3>, mjLABEL_SITE: mjtLabelValue<4>, mjLABEL_CAMERA: mjtLabelValue<5>, mjLABEL_LIGHT: mjtLabelValue<6>, mjLABEL_TENDON: mjtLabelValue<7>, mjLABEL_ACTUATOR: mjtLabelValue<8>, mjLABEL_CONSTRAINT: mjtLabelValue<9>, mjLABEL_FLEX: mjtLabelValue<10>, mjLABEL_SKIN: mjtLabelValue<11>, mjLABEL_SELECTION: mjtLabelValue<12>, mjLABEL_SELPNT: mjtLabelValue<13>, mjLABEL_CONTACTPOINT: mjtLabelValue<14>, mjLABEL_CONTACTFORCE: mjtLabelValue<15>, mjLABEL_ISLAND: mjtLabelValue<16>, mjNLABEL: mjtLabelValue<17>};
  mjtFrame: {mjFRAME_NONE: mjtFrameValue<0>, mjFRAME_BODY: mjtFrameValue<1>, mjFRAME_GEOM: mjtFrameValue<2>, mjFRAME_SITE: mjtFrameValue<3>, mjFRAME_CAMERA: mjtFrameValue<4>, mjFRAME_LIGHT: mjtFrameValue<5>, mjFRAME_CONTACT: mjtFrameValue<6>, mjFRAME_WORLD: mjtFrameValue<7>, mjNFRAME: mjtFrameValue<8>};
  mjtVisFlag: {mjVIS_CONVEXHULL: mjtVisFlagValue<0>, mjVIS_TEXTURE: mjtVisFlagValue<1>, mjVIS_JOINT: mjtVisFlagValue<2>, mjVIS_CAMERA: mjtVisFlagValue<3>, mjVIS_ACTUATOR: mjtVisFlagValue<4>, mjVIS_ACTIVATION: mjtVisFlagValue<5>, mjVIS_LIGHT: mjtVisFlagValue<6>, mjVIS_TENDON: mjtVisFlagValue<7>, mjVIS_RANGEFINDER: mjtVisFlagValue<8>, mjVIS_CONSTRAINT: mjtVisFlagValue<9>, mjVIS_INERTIA: mjtVisFlagValue<10>, mjVIS_SCLINERTIA: mjtVisFlagValue<11>, mjVIS_PERTFORCE: mjtVisFlagValue<12>, mjVIS_PERTOBJ: mjtVisFlagValue<13>, mjVIS_CONTACTPOINT: mjtVisFlagValue<14>, mjVIS_ISLAND: mjtVisFlagValue<15>, mjVIS_CONTACTFORCE: mjtVisFlagValue<16>, mjVIS_CONTACTSPLIT: mjtVisFlagValue<17>, mjVIS_TRANSPARENT: mjtVisFlagValue<18>, mjVIS_AUTOCONNECT: mjtVisFlagValue<19>, mjVIS_COM: mjtVisFlagValue<20>, mjVIS_SELECT: mjtVisFlagValue<21>, mjVIS_STATIC: mjtVisFlagValue<22>, mjVIS_SKIN: mjtVisFlagValue<23>, mjVIS_FLEXVERT: mjtVisFlagValue<24>, mjVIS_FLEXEDGE: mjtVisFlagValue<25>, mjVIS_FLEXFACE: mjtVisFlagValue<26>, mjVIS_FLEXSKIN: mjtVisFlagValue<27>, mjVIS_BODYBVH: mjtVisFlagValue<28>, mjVIS_MESHBVH: mjtVisFlagValue<29>, mjVIS_SDFITER: mjtVisFlagValue<30>, mjNVISFLAG: mjtVisFlagValue<31>};
  mjtRndFlag: {mjRND_SHADOW: mjtRndFlagValue<0>, mjRND_WIREFRAME: mjtRndFlagValue<1>, mjRND_REFLECTION: mjtRndFlagValue<2>, mjRND_ADDITIVE: mjtRndFlagValue<3>, mjRND_SKYBOX: mjtRndFlagValue<4>, mjRND_FOG: mjtRndFlagValue<5>, mjRND_HAZE: mjtRndFlagValue<6>, mjRND_SEGMENT: mjtRndFlagValue<7>, mjRND_IDCOLOR: mjtRndFlagValue<8>, mjRND_CULL_FACE: mjtRndFlagValue<9>, mjNRNDFLAG: mjtRndFlagValue<10>};
  mjtStereo: {mjSTEREO_NONE: mjtStereoValue<0>, mjSTEREO_QUADBUFFERED: mjtStereoValue<1>, mjSTEREO_SIDEBYSIDE: mjtStereoValue<2>};
  mjtPluginCapabilityBit: {mjPLUGIN_ACTUATOR: mjtPluginCapabilityBitValue<1>, mjPLUGIN_SENSOR: mjtPluginCapabilityBitValue<2>, mjPLUGIN_PASSIVE: mjtPluginCapabilityBitValue<4>, mjPLUGIN_SDF: mjtPluginCapabilityBitValue<8>};
  mjtGridPos: {mjGRID_TOPLEFT: mjtGridPosValue<0>, mjGRID_TOPRIGHT: mjtGridPosValue<1>, mjGRID_BOTTOMLEFT: mjtGridPosValue<2>, mjGRID_BOTTOMRIGHT: mjtGridPosValue<3>, mjGRID_TOP: mjtGridPosValue<4>, mjGRID_BOTTOM: mjtGridPosValue<5>, mjGRID_LEFT: mjtGridPosValue<6>, mjGRID_RIGHT: mjtGridPosValue<7>};
  mjtFramebuffer: {mjFB_WINDOW: mjtFramebufferValue<0>, mjFB_OFFSCREEN: mjtFramebufferValue<1>};
  mjtDepthMap: {mjDEPTH_ZERONEAR: mjtDepthMapValue<0>, mjDEPTH_ZEROFAR: mjtDepthMapValue<1>};
  mjtFontScale: {mjFONTSCALE_50: mjtFontScaleValue<50>, mjFONTSCALE_100: mjtFontScaleValue<100>, mjFONTSCALE_150: mjtFontScaleValue<150>, mjFONTSCALE_200: mjtFontScaleValue<200>, mjFONTSCALE_250: mjtFontScaleValue<250>, mjFONTSCALE_300: mjtFontScaleValue<300>};
  mjtFont: {mjFONT_NORMAL: mjtFontValue<0>, mjFONT_SHADOW: mjtFontValue<1>, mjFONT_BIG: mjtFontValue<2>};
  mjtButton: {mjBUTTON_NONE: mjtButtonValue<0>, mjBUTTON_LEFT: mjtButtonValue<1>, mjBUTTON_RIGHT: mjtButtonValue<2>, mjBUTTON_MIDDLE: mjtButtonValue<3>};
  mjtEvent: {mjEVENT_NONE: mjtEventValue<0>, mjEVENT_MOVE: mjtEventValue<1>, mjEVENT_PRESS: mjtEventValue<2>, mjEVENT_RELEASE: mjtEventValue<3>, mjEVENT_SCROLL: mjtEventValue<4>, mjEVENT_KEY: mjtEventValue<5>, mjEVENT_RESIZE: mjtEventValue<6>, mjEVENT_REDRAW: mjtEventValue<7>, mjEVENT_FILESDROP: mjtEventValue<8>};
  mjtItem: {mjITEM_END: mjtItemValue<-2>, mjITEM_SECTION: mjtItemValue<-1>, mjITEM_SEPARATOR: mjtItemValue<0>, mjITEM_STATIC: mjtItemValue<1>, mjITEM_BUTTON: mjtItemValue<2>, mjITEM_CHECKINT: mjtItemValue<3>, mjITEM_CHECKBYTE: mjtItemValue<4>, mjITEM_RADIO: mjtItemValue<5>, mjITEM_RADIOLINE: mjtItemValue<6>, mjITEM_SELECT: mjtItemValue<7>, mjITEM_SLIDERINT: mjtItemValue<8>, mjITEM_SLIDERNUM: mjtItemValue<9>, mjITEM_EDITINT: mjtItemValue<10>, mjITEM_EDITNUM: mjtItemValue<11>, mjITEM_EDITFLOAT: mjtItemValue<12>, mjITEM_EDITTXT: mjtItemValue<13>, mjNITEM: mjtItemValue<14>};
  mjtSection: {mjSECT_CLOSED: mjtSectionValue<0>, mjSECT_OPEN: mjtSectionValue<1>, mjSECT_FIXED: mjtSectionValue<2>};
  MjLROpt: {
    new(): MjLROpt;
  };
  MjModel: {
    new(_0: MjModel): MjModel;
    loadFromXML(_0: EmbindString): MjModel;
  };
  MjData: {
    new(_0: MjModel | null): MjData;
    new(_0: MjModel, _1: MjData): MjData;
  };
  MjOption: {
    new(): MjOption;
  };
  MjStatistic: {
    new(): MjStatistic;
  };
  MjVisualGlobal: {
    new(): MjVisualGlobal;
  };
  MjVisualQuality: {
    new(): MjVisualQuality;
  };
  MjVisualHeadlight: {
    new(): MjVisualHeadlight;
  };
  MjVisualMap: {
    new(): MjVisualMap;
  };
  MjVisualScale: {
    new(): MjVisualScale;
  };
  MjVisualRgba: {
    new(): MjVisualRgba;
  };
  MjVisual: {
    new(): MjVisual;
  };
  MjSolverStat: {
    new(): MjSolverStat;
  };
  MjTimerStat: {
    new(): MjTimerStat;
  };
  MjWarningStat: {
    new(): MjWarningStat;
  };
  MjContact: {
    new(): MjContact;
  };
  MjvPerturb: {
    new(): MjvPerturb;
  };
  MjvCamera: {
    new(): MjvCamera;
  };
  MjvGLCamera: {
    new(): MjvGLCamera;
  };
  MjvGeom: {
    new(): MjvGeom;
  };
  MjvLight: {
    new(): MjvLight;
  };
  MjvOption: {
    new(): MjvOption;
  };
  MjvScene: {
    new(): MjvScene;
    new(_0: MjModel | null, _1: number): MjvScene;
  };
  MjvFigure: {
    new(): MjvFigure;
  };
  MjSpec: {
    new(_0: MjSpec): MjSpec;
  };
  MjsElement: {};
  MjsCompiler: {};
  MjsOrientation: {};
  MjsBody: {};
  MjsGeom: {};
  MjsFrame: {};
  MjsJoint: {};
  MjsSite: {};
  MjsCamera: {};
  MjsLight: {};
  MjsFlex: {};
  MjsMesh: {};
  MjsHField: {};
  MjsSkin: {};
  MjsTexture: {};
  MjsMaterial: {};
  MjsPair: {};
  MjsExclude: {};
  MjsEquality: {};
  MjsTendon: {};
  MjsWrap: {};
  MjsActuator: {};
  MjsSensor: {};
  MjsNumeric: {};
  MjsText: {};
  MjsTuple: {};
  MjsKey: {};
  MjsDefault: {};
  MjsPlugin: {};
  MjVFS: {
    new(): MjVFS;
  };
  MjSolverStatVec: {
    new(): MjSolverStatVec;
  };
  MjTimerStatVec: {
    new(): MjTimerStatVec;
  };
  MjWarningStatVec: {
    new(): MjWarningStatVec;
  };
  MjContactVec: {
    new(): MjContactVec;
  };
  MjvLightVec: {
    new(): MjvLightVec;
  };
  MjvGLCameraVec: {
    new(): MjvGLCameraVec;
  };
  MjvGeomVec: {
    new(): MjvGeomVec;
  };
  FloatBuffer: {
    new(_0: number): FloatBuffer;
    FromArray(_0: any): FloatBuffer;
  };
  DoubleBuffer: {
    new(_0: number): DoubleBuffer;
    FromArray(_0: any): DoubleBuffer;
  };
  IntBuffer: {
    new(_0: number): IntBuffer;
    FromArray(_0: any): IntBuffer;
  };
  mjStringVec: {
    new(): mjStringVec;
  };
  mjIntVec: {
    new(): mjIntVec;
  };
  mjIntVecVec: {
    new(): mjIntVecVec;
  };
  mjFloatVec: {
    new(): mjFloatVec;
  };
  mjFloatVecVec: {
    new(): mjFloatVecVec;
  };
  mjDoubleVec: {
    new(): mjDoubleVec;
  };
  mjByteVec: {
    new(): mjByteVec;
  };
  mjs_attach(_0: MjsElement, _1: MjsElement, _2: string, _3: string): MjsElement | undefined;
  mjs_findElement(_0: MjSpec, _1: mjtObj, _2: string): MjsElement | undefined;
  mjs_firstElement(_0: MjSpec, _1: mjtObj): MjsElement | undefined;
  mjs_nextElement(_0: MjSpec, _1: MjsElement): MjsElement | undefined;
  mjs_getWrapTarget(_0: MjsWrap): MjsElement | undefined;
  mjs_addBody(_0: MjsBody, _1: MjsDefault): MjsBody | undefined;
  mjs_findBody(_0: MjSpec, _1: string): MjsBody | undefined;
  mjs_findChild(_0: MjsBody, _1: string): MjsBody | undefined;
  mjs_getParent(_0: MjsElement): MjsBody | undefined;
  mjs_asBody(_0: MjsElement): MjsBody | undefined;
  mjs_addSite(_0: MjsBody, _1: MjsDefault): MjsSite | undefined;
  mjs_getWrapSideSite(_0: MjsWrap): MjsSite | undefined;
  mjs_asSite(_0: MjsElement): MjsSite | undefined;
  mjs_addJoint(_0: MjsBody, _1: MjsDefault): MjsJoint | undefined;
  mjs_addFreeJoint(_0: MjsBody): MjsJoint | undefined;
  mjs_asJoint(_0: MjsElement): MjsJoint | undefined;
  mjs_addGeom(_0: MjsBody, _1: MjsDefault): MjsGeom | undefined;
  mjs_asGeom(_0: MjsElement): MjsGeom | undefined;
  mjs_addCamera(_0: MjsBody, _1: MjsDefault): MjsCamera | undefined;
  mjs_asCamera(_0: MjsElement): MjsCamera | undefined;
  mjs_addLight(_0: MjsBody, _1: MjsDefault): MjsLight | undefined;
  mjs_asLight(_0: MjsElement): MjsLight | undefined;
  mjs_addFrame(_0: MjsBody, _1: MjsFrame): MjsFrame | undefined;
  mjs_getFrame(_0: MjsElement): MjsFrame | undefined;
  mjs_findFrame(_0: MjSpec, _1: string): MjsFrame | undefined;
  mjs_asFrame(_0: MjsElement): MjsFrame | undefined;
  mjs_addActuator(_0: MjSpec, _1: MjsDefault): MjsActuator | undefined;
  mjs_asActuator(_0: MjsElement): MjsActuator | undefined;
  mjs_addSensor(_0: MjSpec): MjsSensor | undefined;
  mjs_asSensor(_0: MjsElement): MjsSensor | undefined;
  mjs_addFlex(_0: MjSpec): MjsFlex | undefined;
  mjs_asFlex(_0: MjsElement): MjsFlex | undefined;
  mjs_addPair(_0: MjSpec, _1: MjsDefault): MjsPair | undefined;
  mjs_asPair(_0: MjsElement): MjsPair | undefined;
  mjs_addExclude(_0: MjSpec): MjsExclude | undefined;
  mjs_asExclude(_0: MjsElement): MjsExclude | undefined;
  mjs_addEquality(_0: MjSpec, _1: MjsDefault): MjsEquality | undefined;
  mjs_asEquality(_0: MjsElement): MjsEquality | undefined;
  mjs_addTendon(_0: MjSpec, _1: MjsDefault): MjsTendon | undefined;
  mjs_asTendon(_0: MjsElement): MjsTendon | undefined;
  mjs_wrapSite(_0: MjsTendon, _1: string): MjsWrap | undefined;
  mjs_wrapGeom(_0: MjsTendon, _1: string, _2: string): MjsWrap | undefined;
  mjs_addNumeric(_0: MjSpec): MjsNumeric | undefined;
  mjs_asNumeric(_0: MjsElement): MjsNumeric | undefined;
  mjs_addText(_0: MjSpec): MjsText | undefined;
  mjs_asText(_0: MjsElement): MjsText | undefined;
  mjs_addTuple(_0: MjSpec): MjsTuple | undefined;
  mjs_asTuple(_0: MjsElement): MjsTuple | undefined;
  mjs_addKey(_0: MjSpec): MjsKey | undefined;
  mjs_asKey(_0: MjsElement): MjsKey | undefined;
  mjs_addPlugin(_0: MjSpec): MjsPlugin | undefined;
  mjs_asPlugin(_0: MjsElement): MjsPlugin | undefined;
  mjs_addDefault(_0: MjSpec, _1: string, _2: MjsDefault): MjsDefault | undefined;
  mjs_getDefault(_0: MjsElement): MjsDefault | undefined;
  mjs_findDefault(_0: MjSpec, _1: string): MjsDefault | undefined;
  mjs_getSpecDefault(_0: MjSpec): MjsDefault | undefined;
  mjs_addMesh(_0: MjSpec, _1: MjsDefault): MjsMesh | undefined;
  mjs_asMesh(_0: MjsElement): MjsMesh | undefined;
  mjs_addHField(_0: MjSpec): MjsHField | undefined;
  mjs_asHField(_0: MjsElement): MjsHField | undefined;
  mjs_addSkin(_0: MjSpec): MjsSkin | undefined;
  mjs_asSkin(_0: MjsElement): MjsSkin | undefined;
  mjs_addTexture(_0: MjSpec): MjsTexture | undefined;
  mjs_asTexture(_0: MjsElement): MjsTexture | undefined;
  mjs_addMaterial(_0: MjSpec, _1: MjsDefault): MjsMaterial | undefined;
  mjs_asMaterial(_0: MjsElement): MjsMaterial | undefined;
  mjs_getSpec(_0: MjsElement): MjSpec | undefined;
  mjs_findSpec(_0: MjSpec, _1: string): MjSpec | undefined;
  mj_resetCallbacks(): void;
  mj_step(_0: MjModel, _1: MjData): void;
  mj_step1(_0: MjModel, _1: MjData): void;
  mj_step2(_0: MjModel, _1: MjData): void;
  mj_forward(_0: MjModel, _1: MjData): void;
  mj_inverse(_0: MjModel, _1: MjData): void;
  mj_defaultLROpt(_0: MjLROpt): void;
  mj_defaultOption(_0: MjOption): void;
  mj_defaultVisual(_0: MjVisual): void;
  mj_resetData(_0: MjModel, _1: MjData): void;
  mj_setConst(_0: MjModel, _1: MjData): void;
  mj_printFormattedModel(_0: MjModel, _1: string, _2: string): void;
  mj_printModel(_0: MjModel, _1: string): void;
  mj_printFormattedData(_0: MjModel, _1: MjData, _2: string, _3: string): void;
  mj_printData(_0: MjModel, _1: MjData, _2: string): void;
  mj_printScene(_0: MjvScene, _1: string): void;
  mj_printFormattedScene(_0: MjvScene, _1: string, _2: string): void;
  mj_fwdPosition(_0: MjModel, _1: MjData): void;
  mj_fwdVelocity(_0: MjModel, _1: MjData): void;
  mj_fwdActuation(_0: MjModel, _1: MjData): void;
  mj_fwdAcceleration(_0: MjModel, _1: MjData): void;
  mj_fwdConstraint(_0: MjModel, _1: MjData): void;
  mj_Euler(_0: MjModel, _1: MjData): void;
  mj_implicit(_0: MjModel, _1: MjData): void;
  mj_invPosition(_0: MjModel, _1: MjData): void;
  mj_invVelocity(_0: MjModel, _1: MjData): void;
  mj_invConstraint(_0: MjModel, _1: MjData): void;
  mj_compareFwdInv(_0: MjModel, _1: MjData): void;
  mj_sensorPos(_0: MjModel, _1: MjData): void;
  mj_sensorVel(_0: MjModel, _1: MjData): void;
  mj_sensorAcc(_0: MjModel, _1: MjData): void;
  mj_energyPos(_0: MjModel, _1: MjData): void;
  mj_energyVel(_0: MjModel, _1: MjData): void;
  mj_checkPos(_0: MjModel, _1: MjData): void;
  mj_checkVel(_0: MjModel, _1: MjData): void;
  mj_checkAcc(_0: MjModel, _1: MjData): void;
  mj_kinematics(_0: MjModel, _1: MjData): void;
  mj_comPos(_0: MjModel, _1: MjData): void;
  mj_camlight(_0: MjModel, _1: MjData): void;
  mj_flex(_0: MjModel, _1: MjData): void;
  mj_tendon(_0: MjModel, _1: MjData): void;
  mj_transmission(_0: MjModel, _1: MjData): void;
  mj_crb(_0: MjModel, _1: MjData): void;
  mj_makeM(_0: MjModel, _1: MjData): void;
  mj_factorM(_0: MjModel, _1: MjData): void;
  mj_comVel(_0: MjModel, _1: MjData): void;
  mj_passive(_0: MjModel, _1: MjData): void;
  mj_subtreeVel(_0: MjModel, _1: MjData): void;
  mj_rnePostConstraint(_0: MjModel, _1: MjData): void;
  mj_collision(_0: MjModel, _1: MjData): void;
  mj_makeConstraint(_0: MjModel, _1: MjData): void;
  mj_island(_0: MjModel, _1: MjData): void;
  mj_projectConstraint(_0: MjModel, _1: MjData): void;
  mj_referenceConstraint(_0: MjModel, _1: MjData): void;
  mjv_defaultCamera(_0: MjvCamera): void;
  mjv_defaultFreeCamera(_0: MjModel, _1: MjvCamera): void;
  mjv_defaultPerturb(_0: MjvPerturb): void;
  mjv_initPerturb(_0: MjModel, _1: MjData, _2: MjvScene, _3: MjvPerturb): void;
  mjv_applyPerturbForce(_0: MjModel, _1: MjData, _2: MjvPerturb): void;
  mjv_defaultOption(_0: MjvOption): void;
  mjv_defaultFigure(_0: MjvFigure): void;
  mjv_makeLights(_0: MjModel, _1: MjData, _2: MjvScene): void;
  mjv_updateCamera(_0: MjModel, _1: MjData, _2: MjvCamera, _3: MjvScene): void;
  mjv_updateSkin(_0: MjModel, _1: MjData, _2: MjvScene): void;
  mju_writeLog(_0: string, _1: string): void;
  mjs_setDefault(_0: MjsElement, _1: MjsDefault): void;
  mjs_deleteUserValue(_0: MjsElement, _1: string): void;
  mjs_defaultSpec(_0: MjSpec): void;
  mjs_defaultOrientation(_0: MjsOrientation): void;
  mjs_defaultBody(_0: MjsBody): void;
  mjs_defaultFrame(_0: MjsFrame): void;
  mjs_defaultJoint(_0: MjsJoint): void;
  mjs_defaultGeom(_0: MjsGeom): void;
  mjs_defaultSite(_0: MjsSite): void;
  mjs_defaultCamera(_0: MjsCamera): void;
  mjs_defaultLight(_0: MjsLight): void;
  mjs_defaultFlex(_0: MjsFlex): void;
  mjs_defaultMesh(_0: MjsMesh): void;
  mjs_defaultHField(_0: MjsHField): void;
  mjs_defaultSkin(_0: MjsSkin): void;
  mjs_defaultTexture(_0: MjsTexture): void;
  mjs_defaultMaterial(_0: MjsMaterial): void;
  mjs_defaultPair(_0: MjsPair): void;
  mjs_defaultEquality(_0: MjsEquality): void;
  mjs_defaultTendon(_0: MjsTendon): void;
  mjs_defaultActuator(_0: MjsActuator): void;
  mjs_defaultSensor(_0: MjsSensor): void;
  mjs_defaultNumeric(_0: MjsNumeric): void;
  mjs_defaultText(_0: MjsText): void;
  mjs_defaultTuple(_0: MjsTuple): void;
  mjs_defaultKey(_0: MjsKey): void;
  mjs_defaultPlugin(_0: MjsPlugin): void;
  error(_0: string): void;
  mju_printMatSparse(_0: number[], _1: number[], _2: number[], _3: number[]): void;
  mj_resetDataDebug(_0: MjModel, _1: MjData, _2: number): void;
  mjMAXCONPAIR: number;
  mjNEQDATA: number;
  mjNDYN: number;
  mjNGAIN: number;
  mjNBIAS: number;
  mjNREF: number;
  mjNIMP: number;
  mjNSOLVER: number;
  mjNGROUP: number;
  mjMAXLIGHT: number;
  mjMAXOVERLAY: number;
  mjMAXLINE: number;
  mjMAXLINEPNT: number;
  mjMAXPLANEGRID: number;
  mjVERSION_HEADER: number;
  mj_version(): number;
  mju_bandDiag(_0: number, _1: number, _2: number, _3: number): number;
  mj_copyBack(_0: MjSpec, _1: MjModel): number;
  mj_forwardSkip(_0: MjModel, _1: MjData, _2: number, _3: number): void;
  mj_inverseSkip(_0: MjModel, _1: MjData, _2: number, _3: number): void;
  mj_sizeModel(_0: MjModel): number;
  mj_resetDataKeyframe(_0: MjModel, _1: MjData, _2: number): void;
  mjs_activatePlugin(_0: MjSpec, _1: string): number;
  mjs_setDeepCopy(_0: MjSpec, _1: number): number;
  mju_printMat(_0: number[], _1: number, _2: number): void;
  mj_RungeKutta(_0: MjModel, _1: MjData, _2: number): void;
  mj_setKeyframe(_0: MjModel, _1: MjData, _2: number): void;
  mj_addContact(_0: MjModel, _1: MjData, _2: MjContact): number;
  mj_isPyramidal(_0: MjModel): number;
  mj_isSparse(_0: MjModel): number;
  mj_isDual(_0: MjModel): number;
  mj_name2id(_0: MjModel, _1: number, _2: string): number;
  mjv_applyPerturbPose(_0: MjModel, _1: MjData, _2: MjvPerturb, _3: number): void;
  mjv_initGeom(_0: MjvGeom, _1: number, _2: number[], _3: number[], _4: number[], _5: number[]): void;
  mjv_updateScene(_0: MjModel, _1: MjData, _2: MjvOption, _3: MjvPerturb, _4: MjvCamera, _5: number, _6: MjvScene): void;
  mjv_addGeoms(_0: MjModel, _1: MjData, _2: MjvOption, _3: MjvPerturb, _4: number, _5: MjvScene): void;
  mjs_isWarning(_0: MjSpec): number;
  mju_str2Type(_0: string): number;
  mjs_delete(_0: MjSpec, _1: MjsElement): number;
  mjs_getId(_0: MjsElement): number;
  mjs_firstChild(_0: MjsBody, _1: mjtObj, _2: number): MjsElement | undefined;
  mjs_nextChild(_0: MjsBody, _1: MjsElement, _2: number): MjsElement | undefined;
  mjs_setName(_0: MjsElement, _1: string): number;
  mjs_getWrapNum(_0: MjsTendon): number;
  mjs_getWrap(_0: MjsTendon, _1: number): MjsWrap | undefined;
  mjs_setFrame(_0: MjsElement, _1: MjsFrame): number;
  mjs_sensorDim(_0: MjsSensor): number;
  mj_saveLastXML(_0: string, _1: MjModel): number;
  mj_setLengthRange(_0: MjModel, _1: MjData, _2: number, _3: MjLROpt): number;
  mj_stateSize(_0: MjModel, _1: number): number;
  mj_setState(_0: MjModel, _1: MjData, _2: number[], _3: number): void;
  mjPI: number;
  mjMAXVAL: number;
  mjMINMU: number;
  mjMINIMP: number;
  mjMAXIMP: number;
  mjMINVAL: number;
  mju_springDamper(_0: number, _1: number, _2: number, _3: number, _4: number): number;
  mju_min(_0: number, _1: number): number;
  mju_max(_0: number, _1: number): number;
  mju_clip(_0: number, _1: number, _2: number): number;
  mju_sign(_0: number): number;
  mju_round(_0: number): number;
  mju_isBad(_0: number): number;
  mju_Halton(_0: number, _1: number): number;
  mju_sigmoid(_0: number): number;
  mj_getTotalmass(_0: MjModel): number;
  mj_setTotalmass(_0: MjModel, _1: number): void;
  mj_rayHfield(_0: MjModel, _1: MjData, _2: number, _3: number[], _4: number[]): number;
  mj_rayMesh(_0: MjModel, _1: MjData, _2: number, _3: number[], _4: number[]): number;
  mju_rayGeom(_0: number[], _1: number[], _2: number[], _3: number[], _4: number[], _5: number): number;
  mjv_frustumHeight(_0: MjvScene): number;
  mjv_moveCamera(_0: MjModel, _1: number, _2: number, _3: number, _4: MjvScene, _5: MjvCamera): void;
  mjv_movePerturb(_0: MjModel, _1: MjData, _2: number, _3: number, _4: number, _5: MjvScene, _6: MjvPerturb): void;
  mjv_moveModel(_0: MjModel, _1: number, _2: number, _3: number, _4: number[], _5: MjvScene): void;
  mjv_connector(_0: MjvGeom, _1: number, _2: number, _3: number[], _4: number[]): void;
  mju_norm3(_0: number[]): number;
  mju_dot3(_0: number[], _1: number[]): number;
  mju_dist3(_0: number[], _1: number[]): number;
  mju_muscleGain(_0: number, _1: number, _2: number[], _3: number, _4: number[]): number;
  mju_muscleBias(_0: number, _1: number[], _2: number, _3: number[]): number;
  mju_muscleDynamics(_0: number, _1: number, _2: number[]): number;
  mjs_wrapJoint(_0: MjsTendon, _1: string, _2: number): MjsWrap | undefined;
  mjs_wrapPulley(_0: MjsTendon, _1: number): MjsWrap | undefined;
  mjs_getWrapDivisor(_0: MjsWrap): number;
  mjs_getWrapCoef(_0: MjsWrap): number;
  mju_sum(_0: number[]): number;
  mju_L1(_0: number[]): number;
  mju_norm(_0: number[]): number;
  mju_dot(_0: number[], _1: number[]): number;
  mju_mulVecMatVec(_0: number[], _1: number[], _2: number[]): number;
  parseXMLString(_0: EmbindString): MjSpec;
  mj_id2name(_0: MjModel, _1: number, _2: number): string;
  mj_versionString(): string;
  mjs_getError(_0: MjSpec): string;
  mju_type2Str(_0: number): string;
  mju_writeNumBytes(_0: number): string;
  mju_warningText(_0: number, _1: number): string;
  mjs_setToMotor(_0: MjsActuator): string;
  mjs_setToVelocity(_0: MjsActuator, _1: number): string;
  mjs_setToDamper(_0: MjsActuator, _1: number): string;
  mjs_setToCylinder(_0: MjsActuator, _1: number, _2: number, _3: number, _4: number): string;
  mjs_setToAdhesion(_0: MjsActuator, _1: number): string;
  mjs_getName(_0: MjsElement): string;
  get_mjDISABLESTRING(): any;
  get_mjENABLESTRING(): any;
  get_mjTIMERSTRING(): any;
  get_mjLABELSTRING(): any;
  get_mjFRAMESTRING(): any;
  get_mjVISSTRING(): any;
  get_mjRNDSTRING(): any;
  mj_defaultSolRefImp(_0: any, _1: any): void;
  mj_extractState(_0: MjModel, _1: number[], _2: number, _3: any, _4: number): void;
  mj_objectVelocity(_0: MjModel, _1: MjData, _2: number, _3: number, _4: any, _5: number): void;
  mj_objectAcceleration(_0: MjModel, _1: MjData, _2: number, _3: number, _4: any, _5: number): void;
  mj_contactForce(_0: MjModel, _1: MjData, _2: number, _3: any): void;
  mj_local2Global(_0: MjData, _1: any, _2: any, _3: number[], _4: number[], _5: number, _6: number): void;
  mj_ray(_0: MjModel, _1: MjData, _2: number[], _3: number[], _4: number[], _5: number, _6: number, _7: any): number;
  mju_rayFlex(_0: MjModel, _1: MjData, _2: number, _3: number, _4: number, _5: number, _6: number, _7: number, _8: number[], _9: number[], _10: any): number;
  mju_raySkin(_0: number, _1: number, _2: number[], _3: number[], _4: number[], _5: number[], _6: any): number;
  mjv_room2model(_0: any, _1: any, _2: number[], _3: number[], _4: MjvScene): void;
  mjv_model2room(_0: any, _1: any, _2: number[], _3: number[], _4: MjvScene): void;
  mjv_cameraInModel(_0: any, _1: any, _2: any, _3: MjvScene): void;
  mjv_cameraInRoom(_0: any, _1: any, _2: any, _3: MjvScene): void;
  mjv_alignToCamera(_0: any, _1: number[], _2: number[]): void;
  mjv_select(_0: MjModel, _1: MjData, _2: MjvOption, _3: number, _4: number, _5: number, _6: MjvScene, _7: any, _8: any, _9: any, _10: any): number;
  mju_zero3(_0: any): void;
  mju_copy3(_0: any, _1: number[]): void;
  mju_scl3(_0: any, _1: number[], _2: number): void;
  mju_add3(_0: any, _1: number[], _2: number[]): void;
  mju_sub3(_0: any, _1: number[], _2: number[]): void;
  mju_addTo3(_0: any, _1: number[]): void;
  mju_subFrom3(_0: any, _1: number[]): void;
  mju_addToScl3(_0: any, _1: number[], _2: number): void;
  mju_addScl3(_0: any, _1: number[], _2: number[], _3: number): void;
  mju_normalize3(_0: any): number;
  mju_mulMatVec3(_0: any, _1: number[], _2: number[]): void;
  mju_mulMatTVec3(_0: any, _1: number[], _2: number[]): void;
  mju_cross(_0: any, _1: number[], _2: number[]): void;
  mju_zero4(_0: any): void;
  mju_unit4(_0: any): void;
  mju_copy4(_0: any, _1: number[]): void;
  mju_normalize4(_0: any): number;
  mju_transformSpatial(_0: any, _1: number[], _2: number, _3: number[], _4: number[], _5: number[]): void;
  mju_rotVecQuat(_0: any, _1: number[], _2: number[]): void;
  mju_negQuat(_0: any, _1: number[]): void;
  mju_mulQuat(_0: any, _1: number[], _2: number[]): void;
  mju_mulQuatAxis(_0: any, _1: number[], _2: number[]): void;
  mju_axisAngle2Quat(_0: any, _1: number[], _2: number): void;
  mju_quat2Vel(_0: any, _1: number[], _2: number): void;
  mju_subQuat(_0: any, _1: number[], _2: number[]): void;
  mju_quat2Mat(_0: any, _1: number[]): void;
  mju_mat2Quat(_0: any, _1: number[]): void;
  mju_derivQuat(_0: any, _1: number[], _2: number[]): void;
  mju_quatIntegrate(_0: any, _1: number[], _2: number): void;
  mju_quatZ2Vec(_0: any, _1: number[]): void;
  mju_mat2Rot(_0: any, _1: number[]): number;
  mju_euler2Quat(_0: any, _1: number[], _2: string): void;
  mju_mulPose(_0: any, _1: any, _2: number[], _3: number[], _4: number[], _5: number[]): void;
  mju_negPose(_0: any, _1: any, _2: number[], _3: number[]): void;
  mju_trnVecPose(_0: any, _1: number[], _2: number[], _3: number[]): void;
  mju_eig3(_0: any, _1: any, _2: any, _3: number[]): number;
  mju_standardNormal(_0: any): number;
  mjd_quatIntegrate(_0: number[], _1: number, _2: any, _3: any, _4: any): void;
  mjs_setToPosition(_0: MjsActuator, _1: number, _2: any, _3: any, _4: any, _5: number): string;
  mjs_setToIntVelocity(_0: MjsActuator, _1: number, _2: any, _3: any, _4: any, _5: number): string;
  mjs_setToMuscle(_0: MjsActuator, _1: any, _2: number, _3: any, _4: number, _5: number, _6: number, _7: number, _8: number, _9: number, _10: number): string;
  mjs_makeMesh(_0: MjsMesh, _1: mjtMeshBuiltin, _2: any, _3: number): number;
  mjs_resolveOrientation(_0: any, _1: number, _2: string, _3: MjsOrientation): string;
  mj_solveM(_0: MjModel, _1: MjData, _2: any, _3: number[]): void;
  mj_solveM2(_0: MjModel, _1: MjData, _2: any, _3: number[], _4: number[]): void;
  mj_rne(_0: MjModel, _1: MjData, _2: number, _3: any): void;
  mj_constraintUpdate(_0: MjModel, _1: MjData, _2: number[], _3: any, _4: number): void;
  mj_getState(_0: MjModel, _1: MjData, _2: any, _3: number): void;
  mj_mulJacVec(_0: MjModel, _1: MjData, _2: any, _3: number[]): void;
  mj_mulJacTVec(_0: MjModel, _1: MjData, _2: any, _3: number[]): void;
  mj_jac(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number[], _5: number): void;
  mj_jacBody(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number): void;
  mj_jacBodyCom(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number): void;
  mj_jacSubtreeCom(_0: MjModel, _1: MjData, _2: any, _3: number): void;
  mj_jacGeom(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number): void;
  mj_jacSite(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number): void;
  mj_jacPointAxis(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number[], _5: number[], _6: number): void;
  mj_jacDot(_0: MjModel, _1: MjData, _2: any, _3: any, _4: number[], _5: number): void;
  mj_angmomMat(_0: MjModel, _1: MjData, _2: any, _3: number): void;
  mj_fullM(_0: MjModel, _1: any, _2: number[]): void;
  mj_mulM(_0: MjModel, _1: MjData, _2: any, _3: number[]): void;
  mj_mulM2(_0: MjModel, _1: MjData, _2: any, _3: number[]): void;
  mj_addM(_0: MjModel, _1: MjData, _2: any, _3: any, _4: any, _5: any): void;
  mj_applyFT(_0: MjModel, _1: MjData, _2: number[], _3: number[], _4: number[], _5: number, _6: any): void;
  mj_geomDistance(_0: MjModel, _1: MjData, _2: number, _3: number, _4: number, _5: any): number;
  mj_differentiatePos(_0: MjModel, _1: any, _2: number, _3: number[], _4: number[]): void;
  mj_integratePos(_0: MjModel, _1: any, _2: number[], _3: number): void;
  mj_normalizeQuat(_0: MjModel, _1: any): void;
  mj_multiRay(_0: MjModel, _1: MjData, _2: number[], _3: number[], _4: any, _5: number, _6: number, _7: any, _8: any, _9: number, _10: number): void;
  mju_zero(_0: any): void;
  mju_fill(_0: any, _1: number): void;
  mju_copy(_0: any, _1: number[]): void;
  mju_scl(_0: any, _1: number[], _2: number): void;
  mju_add(_0: any, _1: number[], _2: number[]): void;
  mju_sub(_0: any, _1: number[], _2: number[]): void;
  mju_addTo(_0: any, _1: number[]): void;
  mju_subFrom(_0: any, _1: number[]): void;
  mju_addToScl(_0: any, _1: number[], _2: number): void;
  mju_addScl(_0: any, _1: number[], _2: number[], _3: number): void;
  mju_normalize(_0: any): number;
  mju_mulMatVec(_0: any, _1: number[], _2: number[], _3: number, _4: number): void;
  mju_mulMatTVec(_0: any, _1: number[], _2: number[], _3: number, _4: number): void;
  mju_transpose(_0: any, _1: number[], _2: number, _3: number): void;
  mju_symmetrize(_0: any, _1: number[], _2: number): void;
  mju_eye(_0: any): void;
  mju_mulMatMat(_0: any, _1: number[], _2: number[], _3: number, _4: number, _5: number): void;
  mju_mulMatMatT(_0: any, _1: number[], _2: number[], _3: number, _4: number, _5: number): void;
  mju_mulMatTMat(_0: any, _1: number[], _2: number[], _3: number, _4: number, _5: number): void;
  mju_sqrMatTD(_0: any, _1: number[], _2: number[], _3: number, _4: number): void;
  mju_dense2sparse(_0: any, _1: number[], _2: number, _3: number, _4: any, _5: any, _6: any): number;
  mju_sparse2dense(_0: any, _1: number[], _2: number, _3: number, _4: number[], _5: number[], _6: number[]): void;
  mju_cholFactor(_0: any, _1: number): number;
  mju_cholSolve(_0: any, _1: number[], _2: number[]): void;
  mju_cholUpdate(_0: any, _1: any, _2: number): number;
  mju_cholFactorBand(_0: any, _1: number, _2: number, _3: number, _4: number, _5: number): number;
  mju_cholSolveBand(_0: any, _1: number[], _2: number[], _3: number, _4: number, _5: number): void;
  mju_band2Dense(_0: any, _1: number[], _2: number, _3: number, _4: number, _5: number): void;
  mju_dense2Band(_0: any, _1: number[], _2: number, _3: number, _4: number): void;
  mju_bandMulMatVec(_0: any, _1: number[], _2: number[], _3: number, _4: number, _5: number, _6: number, _7: number): void;
  mju_boxQP(_0: any, _1: any, _2: any, _3: number[], _4: number[], _5: number[], _6: number[]): number;
  mju_encodePyramid(_0: any, _1: number[], _2: number[]): void;
  mju_decodePyramid(_0: any, _1: number[], _2: number[]): void;
  mju_isZero(_0: any): number;
  mju_f2n(_0: any, _1: number[]): void;
  mju_n2f(_0: any, _1: number[]): void;
  mju_d2n(_0: any, _1: number[]): void;
  mju_n2d(_0: any, _1: number[]): void;
  mju_insertionSort(_0: any): void;
  mju_insertionSortInt(_0: any): void;
  mjd_transitionFD(_0: MjModel, _1: MjData, _2: number, _3: number, _4: any, _5: any, _6: any, _7: any): void;
  mjd_inverseFD(_0: MjModel, _1: MjData, _2: number, _3: number, _4: any, _5: any, _6: any, _7: any, _8: any, _9: any, _10: any): void;
  mjd_subQuat(_0: number[], _1: number[], _2: any, _3: any): void;
}

export type MainModule = WasmModule & typeof RuntimeExports & EmbindModule;
export default function MainModuleFactory (options?: unknown): Promise<MainModule>;
