# OpenCV

## Abstracts

* Build OpenCV
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License

## How to use?

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````

Then you can try samples. For example [10_ResizeWithPadding](./10_ResizeWithPadding).