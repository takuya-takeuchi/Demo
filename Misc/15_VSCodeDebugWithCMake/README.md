# Debug CMake program with opencv from Visual Studio Code

## Abstracts

* Build and attach program from Visual Studio Code
  * Build by cmake

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

And if you see error message `ERROR: Unable to start debugging. Unexpected LLDB output from command "-exec-run". process exited with status -1 (developer mode is not enabled on this machine and this is a non-interactive debug session.)` when start debug, you must enable developer mode.

````shell
$ sudo DevToolsSecurity -enable
Developer mode is now enabled.
````

## How to usage?

1. Open [15_VSCodeDebugWithCMake](./) as workspace on Visual Studio Code
2. Build opencv by using `pwsh build-opencv.ps1`
2. Press F5 key
3. You can attach to [main.cpp](./main.cpp)

<img src="./images/attach.png" />

## NOTE

In `build.ps1`, you must pass `-D CMAKE_BUILD_TYPE` parameter.
You can not attach to program if missing it.