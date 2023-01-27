# libsodium from CMake

## Abstracts

* Build and link libsodium by cmake
  * static build
* Minimal example program

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio 2019

### Ubuntu

* g++
* libssl-dev

### OSX

* Xcode

## Dependencies

* [libsodium](https://github.com/jedisct1/libsodium)
  * 1.0.18
  * ISC License

## How to usage?

````shell
$ pwsh build-libsodium.ps1  <Debug/Release>
$ pwsh build.ps1  <Debug/Release>
[Info] sodium_init: 0
````