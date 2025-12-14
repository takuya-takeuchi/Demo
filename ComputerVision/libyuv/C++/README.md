# libyuv (C++)

## Abstracts

* Build libyuv
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.16.0 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [libyuv](https://chromium.googlesource.com/libyuv/libyuv)
  * main (deeb764bb92b6305ee0cb5dae7a5940fdb457fa9, Wed Dec 10 03:46:41 2025 +0000)
  * BSD 3-Clause License

## How to use?

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````