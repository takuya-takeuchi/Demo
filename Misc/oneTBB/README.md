# oneAPI Threading Building Blocks (oneTBB)

## Abstracts

* Build oneTBB
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [oneTBB](https://github.com/uxlfoundation/oneTBB)
  * v2022.2.0
  * Apache-2.0 License

## How to build?

### Windows

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1 <Debug/Release>
````

### Linux

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1 <Debug/Release>
````

### OSX

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1 <Debug/Release>
````