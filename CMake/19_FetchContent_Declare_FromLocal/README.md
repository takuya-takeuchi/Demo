# FetchContent_Declare from local file

## Abstracts

* Usage of `FetchContent_Declare`
  * Speficy local file path ([./patch/v3.12.0.zip](v3.12.0.zip)) for URL. You can modify original code without exposing to internet.

## Requirements

### Common

* Powershell 7 or later
* CMake 3.15 or later
* C++ Compiler supports C++ 11 or later

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [json](https://github.com/nlohmann/json)
  * v3.12.0
  * MIT License

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
   pi: 3.141
happy: true
````

### Linux

````bash
$ ./install/linux/bin/Demo
   pi: 3.141
happy: true
````

### OSX

````bash
$ ./install/osx/bin/Demo
   pi: 3.141
happy: true
````