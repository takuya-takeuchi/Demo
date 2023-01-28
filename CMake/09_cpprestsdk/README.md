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
* libssl-dev
  * via `apt` command

### OSX

* Xcode
* websocketpp
  * via `brew` command
    * cpprestsdk can not build due to https://github.com/zaphoyd/websocketpp/issues/1004

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
    * use e1b6a8e61d6b3ab98734b0532bad1da46458212a for only osx because [Fix likely typo in SafeInt3.hpp, that results in error with clang 15](https://github.com/microsoft/cpprestsdk/pull/1711)
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