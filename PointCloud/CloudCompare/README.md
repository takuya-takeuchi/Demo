# CloudCompare

## Abstracts

* Build PcapPlusPlus
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.10 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [CloudCompare](https://github.com/CloudCompare/CloudCompare)
  * v2.13.2
  * GNU General Public License (GPL) version 2.0

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