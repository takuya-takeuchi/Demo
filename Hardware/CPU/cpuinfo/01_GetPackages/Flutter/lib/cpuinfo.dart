import 'package:flutter/material.dart';
import 'package:ffi/ffi.dart';
import 'dart:ffi';
import 'dart:io';

typedef cpuinfo_initialize_function = int Function();
typedef cpuinfo_get_packages_count_function = int Function();
typedef cpuinfo_get_package_function = Pointer<CpuinfoPackage> Function(int index);

const CPUINFO_PACKAGE_NAME_MAX = 48;

final class CpuinfoPackage extends Struct {
  @Array(CPUINFO_PACKAGE_NAME_MAX)
  external Array<Int8> name; // CChar is represented as Int8 in Dart

  @Uint32()
  external int processor_start; // Using int instead of Uint32
  @Uint32()
  external int processor_count; // Using int instead of Uint32
  @Uint32()
  external int core_start; // Using int instead of Uint32
  @Uint32()
  external int core_count; // Using int instead of Uint32
  @Uint32()
  external int cluster_start; // Using int instead of Uint32
  @Uint32()
  external int cluster_count; // Using int instead of Uint32
}

class CpuInfoService {

  DynamicLibrary? _library;
  cpuinfo_initialize_function? _cpuinfo_initialize;
  cpuinfo_get_packages_count_function? _cpuinfo_get_packages_count;
  cpuinfo_get_package_function? _cpuinfo_get_package;

  CpuInfoService() {
    var libraryPath = "";
    if (Platform.isIOS)
      libraryPath = "libcpuinfo.dylib";
    else if (Platform.isMacOS)
      libraryPath = "libcpuinfo.dylib";

    if (libraryPath.isEmpty) {
      assert(false, "Dynamic library is not compatible with this platform.");
      return;
    }

    this._library = DynamicLibrary.open(libraryPath);
    
    this._cpuinfo_initialize = this._library?.lookup<NativeFunction<Int32 Function()>>('cpuinfo_initialize')
                                            .asFunction<int Function()>();
    this._cpuinfo_get_packages_count = this._library?.lookup<NativeFunction<Uint32 Function()>>('cpuinfo_get_packages_count')
                                                     .asFunction<int Function()>();
    this._cpuinfo_get_package = this._library?.lookup<NativeFunction<Pointer<CpuinfoPackage> Function(Uint32)>>('cpuinfo_get_package')
                                              .asFunction<Pointer<CpuinfoPackage> Function(int)>();
  }

  bool Initialize() {
    final function = this._cpuinfo_initialize!;
    return function() > 0;
  }

  int GetPackagesCount() {
    final function = this._cpuinfo_get_packages_count!;
    return function();
  }

  Pointer<CpuinfoPackage> GetPackage(int index) {
    final function = this._cpuinfo_get_package!;
    return function(index);
  }

}