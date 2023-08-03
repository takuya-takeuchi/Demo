# Get Packages

## Abstracts

* Get cpu packages

## Requirements

### Common

* Powershell 7 or later
* CMake 3.5.0 or later
* C++ Compiler supports C++14

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [cpuinfo](https://github.com/pytorch/cpuinfo)
  * d7069b3919d1b65da5e8e333cb5817570a30b49a
  * BSD 2-Clause "Simplified" License

## How to use?

````bat
$ pwsh build.ps1 <win/linux/osx/ios/android> <x86_64/arm64> <Release/Debug>
[info] name: Intel Core i7-8700
````