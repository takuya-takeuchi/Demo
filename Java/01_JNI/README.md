# JNI Project With Gradle 

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
> 
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

