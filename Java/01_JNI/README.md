# Minimal JNI (Java Native Interface) project

## Abstracts

* How to build and use JNI

## Requirements

### Common

* CMake 3.13 or later
* Powershell 7
* Java

### Windows 

* Visual Studio

### Linux

* gcc

## How to usage?

### Windows

````bat
> gradlew.bat run  

> Task :jni-lib:compileJNI
-- Selecting Windows SDK version 10.0.22000.0 to target Windows 10.0.19044.
-- The CXX compiler identification is MSVC 19.34.31937.0
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.34.31933/bin/Hostx64/x64/cl.exe - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Found Java: C:/Program Files/Java/jdk-14/bin/java.exe (found version "14.0.0")
-- Found JNI: C:/Program Files/Eclipse Foundation/jdk-8.0.302.8-hotspot/include  found components: AWT JVM 
-- Configuring done
-- Generating done
-- Build files have been written to: E:/Works/OpenSource/Demo/Java/01_JNI/jni-lib/build/natives
MSBuild version 17.4.1+9a89d02ff for .NET Framework
  Checking Build System
  Building Custom Rule E:/Works/OpenSource/Demo/Java/01_JNI/jni-lib/src/main/cpp/CMakeLists.txt
  CMakeCXXCompilerId.cpp
  hello.cpp
  コードを生成中...
     ライブラリ E:/Works/OpenSource/Demo/Java/01_JNI/jni-lib/build/natives/lib/Release/hello.lib とオブジェクト E:/Works/OpenSource/Demo/Java/01_JNI/jni-lib/build/natives/lib/Release/hello.exp を作成中
  hello.vcxproj -> E:\Works\OpenSource\Demo\Java\01_JNI\jni-lib\build\natives\bin\Release\hello.dll
  Building Custom Rule E:/Works/OpenSource/Demo/Java/01_JNI/jni-lib/CMakeLists.txt

> Task :jni-runner:run
hello, world!!        

BUILD SUCCESSFUL in 9s
6 actionable tasks: 6 executed
````

### Linux

````sh
$ ./gradlew run

> Task :jni-lib:compileJNI
-- The CXX compiler identification is GNU 9.4.0
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Found Java: /usr/bin/java (found version "11.0.17") 
-- Found JNI: NotNeeded  
-- Configuring done
-- Generating done
-- Build files have been written to: /home/t-takeuchi/Work/OpenSource/Demo/Java/01_JNI/jni-lib/build/natives
Scanning dependencies of target hello
[ 33%] Building CXX object src/main/cpp/CMakeFiles/hello.dir/__/__/__/CMakeFiles/3.16.3/CompilerIdCXX/CMakeCXXCompilerId.cpp.o
[ 66%] Building CXX object src/main/cpp/CMakeFiles/hello.dir/hello.cpp.o
[100%] Linking CXX shared library ../../../lib/libhello.so
[100%] Built target hello

> Task :jni-runner:run
hello, world!!

BUILD SUCCESSFUL in 2s
6 actionable tasks: 6 executed
````

