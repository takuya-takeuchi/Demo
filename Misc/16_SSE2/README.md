# Use SSE2 (Streaming SIMD Extensions 2) instructions

## Abstracts

* Use SSE2 instructions
  * Run 128 bit instruction by 1 clock

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17
* Intel or AMD supports SSE2

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

N/A

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ ./install/linux/bin/Demo
[Info] Get lower 24bit and store as 32bit
0 : 0
1 : 1
2 : 2
3 : 0
4 : 4
5 : 5
6 : 6
7 : 0
[Info] Get lower 24bit and store as 24bit
0 : 0
1 : 1
2 : 2
3 : 4
4 : 5
5 : 6
6 : 0
7 : 0
````