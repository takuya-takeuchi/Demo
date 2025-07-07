# PcapPlusPlus

## Abstracts

* Build PcapPlusPlus
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

* [WebSocPcapPlusPlus](https://github.com/seladb/PcapPlusPlus)
  * v25.05
  * Unlicense License

## How to build?

````shell
$ git submodule update --init --recursive .
$ pwsh build-boost.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````