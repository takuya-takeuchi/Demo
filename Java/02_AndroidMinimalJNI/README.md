# Minimal Android JNI (Java Native Interface) project

## Abstracts

* How to build and use Android JNI

## Requirements

This project use Android SDK 33.

### Common

* Java 17
  * Can not use Java 19. Java 19 occurs `General error during conversion: Unsupported class file major version 63` when build.

### Windows 

* Visual Studio 2022

### Linux

* gcc

## Setup

At first, you must update `local.properties` to set sdk.
However, you should not use Android SDK manager of `Visual Studio` on Windows.
Because it can not install `CMake` which supports Generators `Gradle`.
Therefore, you should install `Android Studio` or `Command Line Tools of SDK Manager`.

You can download them from [Android Studio](https://developer.android.com/studio).

Here, use command line tools.

### Common

Set `JAVA_HOME`.
You can not use Java 19.

### Windows

Download command line tools and extract it into `C:\Android`. Of course, you can change directory. 
And you must specify SDK root directory. Here, use `C:\Android\SDK`.

````bat
> set JAVA_HOME=C:\Program Files\Java\jdk-17.0.6
> cd C:\Android\cmdline-tools\bin
> sdkmanager --update --sdk_root=C:\Android\SDK
> sdkmanager build-tools;33.0.1 --sdk_root=C:\Android\SDK
> sdkmanager platforms;android-33 --sdk_root=C:\Android\SDK
> sdkmanager cmake;3.22.1 --sdk_root=C:\Android\SDK
> sdkmanager ndk;25.1.8937393 --sdk_root=C:\Android\SDK
> sdkmanager patcher;v4 --sdk_root=C:\Android\SDK
> sdkmanager --sdk_root=C:\Android\SDK --licenses
````

Then, update `local.properties` like this.

````txt
sdk.dir = C:\\Android\\SDK
ndk.dir = C:\\Android\\SDK\\ndk\\25.1.8937393
cmake.dir = C:\\Android\\SDK\\cmake\\3.22.1
````

## How to usage?


### Windows

````bat
> set JAVA_HOME=C:\Program Files\Java\jdk-17.0.6
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

