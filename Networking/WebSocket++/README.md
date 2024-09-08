# WebSocket++

## Abstracts

* Build WebSocket++
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 2.8.8 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Boost](https://www.boost.org/)
  * v1.86.0
  * Boost License
* [WebSocket++](https://github.com/zaphoyd/websocketpp)
  * 0.8.2
  * BSD-3-Clause License

## How to build?

````shell
$ git submodule update --init --recursive .
$ pwsh build-boost.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````