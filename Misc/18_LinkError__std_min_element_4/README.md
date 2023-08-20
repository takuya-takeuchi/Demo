# unresolved external symbol __std_min_element_4

## Abstracts

* How to resolve `unresolved external symbol __std_min_element_4`

## Requirements

### Windows

* Visual Studio
* CMake 3.0 or later

## How to resolve?

### Japanese message

````bat
$ pwsh Build.ps1

opencv_world470.lib(ocl.obj) : error LNK2019: 未解決の外部シンボル __std_min_element_4 が関数 "int * __cdecl __std_min_element<int>(int *,int *)" (??$__std_min_element@H@@YAPEAHPEAH0@Z) で参照されました [D:\Demo\build\win\program\vs2019\Demo.vcxproj]
````

### English message

````bat
$ pwsh Build.ps1

opencv_world470.lib(ocl.obj) : error LNK2019: unresolved external symbol __std_min_element_4 referenced in function "int * __cdecl __std_min_element<int>(int *,int *)" (??$__std_min_element@H@@YAPEAHPEAH0@Z) [D:\Demo\build\win\program\vs2019\Demo.vcxproj]
````

The root cause of this issue is linking prebuild static libraries built by Visual Studio 2022 to program built by Visual Studio 2019.
It is design rather than bug.

This is reported on [__imp___std_init_once_complete unresolved external symbol after linking msvcprtd.lib](https://developercommunity.visualstudio.com/t/-imp-std-init-once-complete-unresolved-external-sy/1684365#T-N10080626).
But developer of Microsoft says 

> it sounds like you’re compiling some object files/static libraries with VS 2022, but performing the final link with VS 2019. This is not a supported scenario

And let me know document about this specification [C++ binary compatibility between Visual Studio versions](https://learn.microsoft.com/en-us/cpp/porting/binary-compat-2015-2017?view=msvc-170)

But Microsoft fixed some bugs about it on Visual Studio 2022 17.3, so we should use latest Visual Studio 2022.

