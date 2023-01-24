# spdlog from CMake

## Abstracts

* Build and link spdlog by cmake

## Requirements

## Common

* Powershell
* CMake 3.20.0 or later

### Windows

* Visual Studio

### Ubuntu

* g++
* build-essential
* gdb

### OSX

* Xcode

## Dependencies

* [spdlog](https://github.com/gabime/spdlog)
  * v1.11.0
  * MIT License

## How to usage?

1. Open [15_VSCodeDebugWithCMake](./) as workspace on Visual Studio Code
2. Build opencv by using `pwsh build-opencv.ps1`
2. Press F5 key
3. You can attach to [main.cpp](./main.cpp)

<img src="./images/attach.png" />

## NOTE

In `build.ps1`, you must pass `-D CMAKE_BUILD_TYPE` parameter.
You can not attach to program if missing it.