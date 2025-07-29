# CloudCompare

## Abstracts

* Build PcapPlusPlus
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.10 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [CloudCompare](https://github.com/CloudCompare/CloudCompare)
  * v2.13.2
  * GNU General Public License (GPL) version 2.0
* [Qt]
  * 5.15.16
  * Multiple licenses
    * GNU Free Documentation License (GFDL) version 1.3
    * GNU General Public License (GPL) version 2.0
    * GNU General Public License (GPL) version 3.0
    * GNU Lesser General Public License (LGPL) version 2.1
    * GNU Lesser General Public License (LGPL) version 3.0

## How to build?

### Windows

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1 <Debug/Release>
````

### Linux

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1 <Debug/Release>
````

### OSX

````shell
$ git submodule update --init --recursive .
$ brew install qt@5
$ pwsh build.ps1 <Debug/Release>  
$ sudo codesign --force --deep --sign - ./install/osx/CloudCompare/ccViewer/ccViewer.app    
````

<img src="./images/osx.png" />