# Change RuntimeLibrary Property for Visual Studio

## Abstracts

* How to change RuntimeLibrary propery
  * Change from /MD to /MT

## Requirements

### Windows

* Visual Studio
* CMake 3.15 or later

## How to check?

````bat
$ pwsh Build.ps1
Use: C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat
**********************************************************************
** Visual Studio 2022 Developer Command Prompt v17.14.17
** Copyright (c) 2025 Microsoft Corporation
**********************************************************************
[vcvarsall.bat] Environment initialized for: 'x64'
Active code page: 65001
-- Selecting Windows SDK version 10.0.26100.0 to target Windows 10.0.19045.
-- Configuring done (0.1s)
-- Generating done (0.2s)
-- Build files have been written to: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/build/win/program/add_compile_options
MSBuild のバージョン 17.14.23+b0019275e (.NET Framework)

  Demo1.vcxproj -> E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\add_compile_options\Release\Demo1.exe
  1>
  -- Install configuration: "Release"
  -- Up-to-date: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/install/win/program/bin/Demo1.exe
-- Selecting Windows SDK version 10.0.26100.0 to target Windows 10.0.19045.
-- Configuring done (0.1s)
-- Generating done (0.2s)
-- Build files have been written to: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/build/win/program/CMAKE_CXX_FLAGS
MSBuild のバージョン 17.14.23+b0019275e (.NET Framework)

  Demo2.vcxproj -> E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\CMAKE_CXX_FLAGS\Release\Demo2.exe
  1>
  -- Install configuration: "Release"
  -- Up-to-date: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/install/win/program/bin/Demo2.exe
-- Selecting Windows SDK version 10.0.26100.0 to target Windows 10.0.19045.
-- Configuring done (0.1s)
-- Generating done (0.2s)
-- Build files have been written to: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/build/win/program/MSVC_RUNTIME_LIBRARY
MSBuild のバージョン 17.14.23+b0019275e (.NET Framework)

  Demo3.vcxproj -> E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\MSVC_RUNTIME_LIBRARY\Release\Demo3.exe
  1>
  -- Install configuration: "Release"
  -- Up-to-date: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/install/win/program/bin/Demo3.exe
-- Selecting Windows SDK version 10.0.26100.0 to target Windows 10.0.19045.
-- Configuring done (0.1s)
-- Generating done (0.1s)
-- Build files have been written to: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/build/win/program/CMAKE_MSVC_RUNTIME_LIBRARY
MSBuild のバージョン 17.14.23+b0019275e (.NET Framework)

  1>Checking Build System
  Building Custom Rule E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/CMAKE_MSVC_RUNTIME_LIBRARY/CMakeLists.txt
  main.cpp
  Demo4.vcxproj -> E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Release\Demo4.exe
  Building Custom Rule E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/CMAKE_MSVC_RUNTIME_LIBRARY/CMakeLists.txt
  1>
  -- Install configuration: "Release"
  -- Installing: E:/Works/OpenSource/Demo2/CMake/16_RuntimeLibrary/install/win/program/bin/Demo4.exe
E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\add_compile_options\Demo1.vcxproj

build\win\program\add_compile_options\Demo1.vcxproj:96:      <RuntimeLibrary>MultiThreadedDebug</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:135:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:176:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:218:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:92:      <RuntimeLibrary>MultiThreadedDebugDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:131:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:170:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:209:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:96:      <RuntimeLibrary>MultiThreadedDebug</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:135:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:176:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:218:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
E:\Works\OpenSource\Demo2\CMake\16_RuntimeLibrary\build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Demo4.vcxproj
build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Demo4.vcxproj:96:      <RuntimeLibrary>MultiThreadedDebug</RuntimeLibrary>
build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Demo4.vcxproj:135:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Demo4.vcxproj:176:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\CMAKE_MSVC_RUNTIME_LIBRARY\Demo4.vcxproj:218:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
````

|Directory|Description|
|---|---|
|[add_compile_options/CMakeLists.txt](add_compile_options/CMakeLists.txt)|OK|
|[CMAKE_CXX_FLAGS/CMakeLists.txt](CMAKE_CXX_FLAGS/CMakeLists.txt)|NG|
|[MSVC_RUNTIME_LIBRARY/CMakeLists.txt](MSVC_RUNTIME_LIBRARY/CMakeLists.txt)|OK|
|[CMAKE_MSVC_RUNTIME_LIBRARY/CMakeLists.txt](CMAKE_MSVC_RUNTIME_LIBRARY/CMakeLists.txt)|OK|

Most of web pages says, `set(CMAKE_CXX_FLAGS_RELEASE "/MT")` would work fine.
But it does not work and this syntax is already obsolete.