# Use OpenMP

## Abstracts

* Use OpenMP from CMake
  * Try simple for loop

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

N/A

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ ./install/win/bin/Demo
 Enable OpenMP: 23025 ms
Disable OpenMP: 74437 ms
````