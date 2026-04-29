# Command line parser

## Abstracts

* Example of program how to use command line parser by boost library

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Boost](https://www.boost.org/)
  * v1.84.0
  * Boost License

## How to usage?

````shell
$ pwsh build-boost.ps1 <Debug/Release> <BoostVersion, like 1.84.0>
$ pwsh build.ps1 <Debug/Release> <BoostVersion, like 1.84.0>
[Info]    loop: 3
[Info] message: lower
[Info]   upper: false
lower
lower
lower
[Info]    loop: 2
[Info] message: upper
[Info]   upper: true
UPPER
UPPER
````