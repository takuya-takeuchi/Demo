# simple usage of googletest

## Abstacts

* How to use googletest from source code

## Requirements

* Visual Studio 2022
* CMake version 3.10 or newer
  * You can install it via `winget install -e --id Kitware.CMake`

## Dependencies

* [googletest](https://github.com/google/googletest)
  * BSD-3-Clause license

## How to usage?

You must execute `Build.ps1` to build dependencies and demo program.<br>
This script builds all project and you need not to launch Visual Studio.

````cmd
$ cd sources\Demo\bin\x64\Release
$ Demo.exe
[==========] Running 1 test from 1 test suite.
[----------] Global test environment set-up.
[----------] 1 test from TestTarget
[ RUN      ] TestTarget.CalcAdd
[       OK ] TestTarget.CalcAdd (0 ms)
[----------] 1 test from TestTarget (0 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (29 ms total)
[  PASSED  ] 1 test.
````
