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
-- Selecting Windows SDK version 10.0.22621.0 to target Windows 10.0.19045.
-- The C compiler identification is MSVC 19.39.33523.0
-- The CXX compiler identification is MSVC 19.39.33523.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.39.33519/bin/Hostx64/x64/cl.exe - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.39.33519/bin/Hostx64/x64/cl.exe - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done (5.4s)
-- Generating done (0.3s)
-- Build files have been written to: E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/build/win/program
MSBuild のバージョン 17.9.8+b34f75857 (.NET Framework)

  1>Checking Build System
  Building Custom Rule E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/add_compile_options/CMakeLists.txt
  main.cpp
  Demo1.vcxproj -> E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\add_compile_options\Release\Demo1.exe
  Building Custom Rule E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/CMAKE_CXX_FLAGS/CMakeLists.txt
  main.cpp
  Demo2.vcxproj -> E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\CMAKE_CXX_FLAGS\Release\Demo2.exe
  Building Custom Rule E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/MSVC_RUNTIME_LIBRARY/CMakeLists.txt
  main.cpp
  Demo3.vcxproj -> E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\MSVC_RUNTIME_LIBRARY\Release\Demo3.exe
  Building Custom Rule E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/CMakeLists.txt
  1>
  -- Install configuration: "Release"
  -- Installing: E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/install/win/program/bin/Demo1.exe
  -- Installing: E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/install/win/program/bin/Demo2.exe
  -- Installing: E:/Works/OpenSource/Demo/CMake/16_RuntimeLibrary/install/win/program/bin/Demo3.exe
E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\add_compile_options\Demo1.vcxproj

build\win\program\add_compile_options\Demo1.vcxproj:96:      <RuntimeLibrary>MultiThreadedDebug</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:135:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:176:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\add_compile_options\Demo1.vcxproj:218:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:92:      <RuntimeLibrary>MultiThreadedDebugDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:131:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:170:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
build\win\program\CMAKE_CXX_FLAGS\Demo2.vcxproj:209:      <RuntimeLibrary>MultiThreadedDLL</RuntimeLibrary>
E:\Works\OpenSource\Demo\CMake\16_RuntimeLibrary\build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:96:      <RuntimeLibrary>MultiThreadedDebug</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:135:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:176:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
build\win\program\MSVC_RUNTIME_LIBRARY\Demo3.vcxproj:218:      <RuntimeLibrary>MultiThreaded</RuntimeLibrary>
````

|Directory|Description|
|---|---|
|[add_compile_options/CMakeLists.txt](add_compile_options/CMakeLists.txt)|OK|
|[CMAKE_CXX_FLAGS/CMakeLists.txt](CMAKE_CXX_FLAGS/CMakeLists.txt)|NG|
|[MSVC_RUNTIME_LIBRARY/CMakeLists.txt](MSVC_RUNTIME_LIBRARY/CMakeLists.txt)|OK|

Most of web pages says, `set(CMAKE_CXX_FLAGS_RELEASE "/MT")` would work fine.
But it does not work and this syntax is already obsolete.