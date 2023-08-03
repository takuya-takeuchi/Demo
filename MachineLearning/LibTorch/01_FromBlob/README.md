# Tensor from blob

## Abstracts

* Create torch::Tensor from float memory array

## Requirements

### Common

* Powershell 7 or later
* CMake 3.5.0 or later
* C++ Compiler supports C++14
  * Do not support C++17

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [pytroch](https://github.com/pytorch/pytorch)
  * 1.8.2 (for windows) or 1.12.0 (for others)
  * 3-clause BSD License

## How to use?

````bat
$ pwsh build.ps1 <win/linux/osx/ios/android> <x86_64/arm64> Release
1
 2
 3
 4
 5
 6
 7
 8
[ CPUFloatType{8} ]
 1  2  3  4
 5  6  7  8
[ CPUFloatType{2,4} ]
(1,.,.) =
  1  2
  3  4

(2,.,.) =
  5  6
  7  8
[ CPUFloatType{2,2,2} ]
````