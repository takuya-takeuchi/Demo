# cpprestsdk from CMake

## Abstracts

* Build and link cpprestsdk by cmake
  * static build
* Minimal example program

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

Not supported

### Ubuntu

* g++
* libtar
  * via `apt` command
    * `apt install libtar-dev`

### OSX

* Xcode
* websocketpp
  * via `brew` command
    * cpprestsdk can not build due to https://github.com/zaphoyd/websocketpp/issues/1004

## Dependencies

* [libtar](https://repo.or.cz/libtar.git)
  * BSD 3-Clause License

## How to usage?

````shell
$ pwsh build.ps1 <Debug/Release>
succeeded to create tar file
Enumerete contents in /home/user/Demo/CMake/10_libtar/lenna.tar
images/lenna.png
````