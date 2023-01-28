# cpprestsdk from CMake

## Abstracts

* Build and link cpprestsdk by cmake
  * static build
* Minimal example program

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio

### Ubuntu

* g++
* libz-dev
  * via `apt` command

### OSX

* Xcode

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
  * MIT License
* [Boost](https://www.boost.org/)
  * v1.81.0
  * Boost License

## How to usage?

````shell
$ pwsh build-boost.ps1 <Debug/Release>
$ pwsh build-cpprestsdk.ps1  <Debug/Release>
$ pwsh build.ps1  <Debug/Release>
````