# How to check dependency

## Abstracts

* How to check dynamic link error

## Requirements

* Powershell 7 or later
* Android Studio
* Android CMake 3.22.1
* Android NDK 26.1.10909125

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.9.0
  * Apache License 2.0 license

## How to build?

Before build, set android home and android ndk path.

````shell
$ export ANDROID_SDK_ROOT=/Users/xxxxxxxx/Library/Android/sdk
$ export ANDROID_NDK_ROOT=/Users/xxxxxxxx/Library/Android/sdk/ndk/26.1.10909125
````

##### No link error

````shell
$ pwsh build_and_run.ps1
...
General configuration for OpenCV 4.9.0 =====================================
  Version control:               4.9.0

...

  Install to:                    /Users/xxxxxxxx/Work/OpenSource/Demo/Mobile/Android/03_CheckNativeLibraryDependency/install/opencv/arm64-v8a/21/Release
-----------------------------------------------------------------
````

##### link error

Before run script, connect to target device and launch logcat in other shell.

````shell
$ adb shell
$ logcat -s linker
````

Then, kick script.

````shell
$ pwsh build_and_run_with_error.ps1
...
CANNOT LINK EXECUTABLE "/data/local/tmp/hello": library "libomp.so" not found: needed by /data/local/tmp/libwrapper.so in namespace (default)
````

And you can see detail of linker error in other shell.

````shell
07-15 06:23:31.598 19251 19251 W linker  : [ Android dynamic linker (64-bit) ]
07-15 06:23:31.598 19251 19251 W linker  : [ LD_LIBRARY_PATH set to "/data/local/tmp" ]
07-15 06:23:31.598 19251 19251 W linker  : [ Linking executable "/data/local/tmp/hello" ]
07-15 06:23:31.598 19251 19251 I linker  : name /data/local/tmp/hello: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.598 19251 19251 I linker  : name /data/local/tmp/hello: allocated soinfo @ 0x73aa2e5010
07-15 06:23:31.598 19251 19251 I linker  : "/data/local/tmp/hello" PT_GNU_PROPERTY: not found
07-15 06:23:31.598 19251 19251 I linker  : name [vdso]: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.598 19251 19251 I linker  : name [vdso]: allocated soinfo @ 0x73aa2e5268
07-15 06:23:31.598 19251 19251 W linker  : [ Linking "[vdso]" ]
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: si->base = 0x73ab56a000 si->flags = 0x40000000
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: dynamic = 0x73ab56a8b0
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a8b0, d[0](tag) = 0xe d[1](val) = 0x5a
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a8c0, d[0](tag) = 0x1e d[1](val) = 0x2
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a8d0, d[0](tag) = 0x70000001 d[1](val) = 0x0
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a8e0, d[0](tag) = 0x6 d[1](val) = 0x118
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a8f0, d[0](tag) = 0xb d[1](val) = 0x18
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a900, d[0](tag) = 0x5 d[1](val) = 0x190
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a910, d[0](tag) = 0xa d[1](val) = 0x77
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a920, d[0](tag) = 0x4 d[1](val) = 0xe8
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a930, d[0](tag) = 0x6ffffff0 d[1](val) = 0x208
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a940, d[0](tag) = 0x6ffffffc d[1](val) = 0x214
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: d = 0x73ab56a950, d[0](tag) = 0x6ffffffd d[1](val) = 0x2
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: si->base = 0x73ab56a000, si->strtab = 0x73ab56a190, si->symtab = 0x73ab56a118
07-15 06:23:31.598 19251 19251 D linker  : DEBUG: [ finished linking [vdso] ]
07-15 06:23:31.598 19251 19251 W linker  : [ Reading linker config "/linkerconfig/ld.config.txt" ]
07-15 06:23:31.599 19251 19251 W linker  : [ Using config section "unrestricted" ]
07-15 06:23:31.600 19251 19251 I linker  : Trying zip file open from path "/odm/lib64/vndk-sp" -> normalized "/odm/lib64/vndk-sp"
07-15 06:23:31.600 19251 19251 I linker  : Trying zip file open from path "/vendor/lib64/vndk-sp" -> normalized "/vendor/lib64/vndk-sp"
07-15 06:23:31.600 19251 19251 I linker  : Trying zip file open from path "/odm/lib64" -> normalized "/odm/lib64"
07-15 06:23:31.601 19251 19251 W linker  : [ Linking "/data/local/tmp/hello" ]
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: si->base = 0x5efdf4c000 si->flags = 0x40000004
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: dynamic = 0x5efdf4d700
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d700, d[0](tag) = 0x1 d[1](val) = 0x3c
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d710, d[0](tag) = 0x1 d[1](val) = 0x4a
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d720, d[0](tag) = 0x1 d[1](val) = 0x52
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d730, d[0](tag) = 0x1 d[1](val) = 0x2f
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d740, d[0](tag) = 0x1e d[1](val) = 0x8
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d750, d[0](tag) = 0x6ffffffb d[1](val) = 0x8000001
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d760, d[0](tag) = 0x15 d[1](val) = 0x0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d770, d[0](tag) = 0x7 d[1](val) = 0x470
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d780, d[0](tag) = 0x8 d[1](val) = 0x60
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d790, d[0](tag) = 0x9 d[1](val) = 0x18
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7a0, d[0](tag) = 0x6ffffff9 d[1](val) = 0x4
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7b0, d[0](tag) = 0x17 d[1](val) = 0x4d0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7c0, d[0](tag) = 0x2 d[1](val) = 0x48
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7d0, d[0](tag) = 0x3 d[1](val) = 0x1910
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7e0, d[0](tag) = 0x14 d[1](val) = 0x7
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d7f0, d[0](tag) = 0x6 d[1](val) = 0x348
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d800, d[0](tag) = 0xb d[1](val) = 0x18
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d810, d[0](tag) = 0x5 d[1](val) = 0x414
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d820, d[0](tag) = 0xa d[1](val) = 0x5b
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d830, d[0](tag) = 0x6ffffef5 d[1](val) = 0x3d0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d840, d[0](tag) = 0x4 d[1](val) = 0x3ec
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d850, d[0](tag) = 0x20 d[1](val) = 0x16d0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: /data/local/tmp/hello constructors (DT_PREINIT_ARRAY) found at 0x5efdf4d6d0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d860, d[0](tag) = 0x21 d[1](val) = 0x10
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d870, d[0](tag) = 0x19 d[1](val) = 0x16e0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: /data/local/tmp/hello constructors (DT_INIT_ARRAY) found at 0x5efdf4d6e0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d880, d[0](tag) = 0x1b d[1](val) = 0x10
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d890, d[0](tag) = 0x1a d[1](val) = 0x16f0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: /data/local/tmp/hello destructors (DT_FINI_ARRAY) found at 0x5efdf4d6f0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d8a0, d[0](tag) = 0x1c d[1](val) = 0x10
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d8b0, d[0](tag) = 0x6ffffff0 d[1](val) = 0x3a8
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d8c0, d[0](tag) = 0x6ffffffe d[1](val) = 0x3b0
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: d = 0x5efdf4d8d0, d[0](tag) = 0x6fffffff d[1](val) = 0x1
07-15 06:23:31.601 19251 19251 D linker  : DEBUG: si->base = 0x5efdf4c000, si->strtab = 0x5efdf4c414, si->symtab = 0x5efdf4c348
07-15 06:23:31.601 19251 19251 I linker  : [ "libwrapper.so" find_loaded_library_by_soname failed (*candidate=n/a@0x0). Trying harder... ]
07-15 06:23:31.601 19251 19251 I linker  : [ opening libwrapper.so from namespace (default) ]
07-15 06:23:31.601 19251 19251 I linker  : name /data/local/tmp/libwrapper.so: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.601 19251 19251 I linker  : name /data/local/tmp/libwrapper.so: allocated soinfo @ 0x73aa2e54c0
07-15 06:23:31.601 19251 19251 I linker  : [ "libm.so" find_loaded_library_by_soname failed (*candidate=n/a@0x0). Trying harder... ]
07-15 06:23:31.601 19251 19251 I linker  : [ opening libm.so from namespace (default) ]
07-15 06:23:31.601 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libm.so: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.602 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libm.so: allocated soinfo @ 0x73aa2e5718
07-15 06:23:31.602 19251 19251 I linker  : [ "libdl.so" find_loaded_library_by_soname failed (*candidate=n/a@0x0). Trying harder... ]
07-15 06:23:31.602 19251 19251 I linker  : [ opening libdl.so from namespace (default) ]
07-15 06:23:31.602 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libdl.so: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.602 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libdl.so: allocated soinfo @ 0x73aa2e5970
07-15 06:23:31.602 19251 19251 I linker  : [ "libc.so" find_loaded_library_by_soname failed (*candidate=n/a@0x0). Trying harder... ]
07-15 06:23:31.602 19251 19251 I linker  : [ opening libc.so from namespace (default) ]
07-15 06:23:31.602 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libc.so: allocating soinfo for ns=0x73ab5725f0
07-15 06:23:31.602 19251 19251 I linker  : name /apex/com.android.runtime/lib64/bionic/libc.so: allocated soinfo @ 0x73aa2e5bc8
07-15 06:23:31.603 19251 19251 I linker  : [ "libomp.so" find_loaded_library_by_soname failed (*candidate=n/a@0x0). Trying harder... ]
07-15 06:23:31.603 19251 19251 I linker  : [ opening libomp.so from namespace (default) ]
07-15 06:23:31.616 19251 19251 F linker  : CANNOT LINK EXECUTABLE "/data/local/tmp/hello": library "libomp.so" not found: needed by /data/local/tmp/libwrapper.so in namespace (default)
````

You can see `libomp.so` is missing in android system.
And you will find out that we should make disable openmp for opnecv.
Indeed, [build_and_run_with_error.ps1](./build_and_run_with_error.ps1) makes enable `WITH_OPENMP`.