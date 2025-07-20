# ExternalProject_Add

## Abstracts

* Usage of `ExternalProject_Add`

## Requirements

### Common

* Powershell 7 or later
* CMake 3.15 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [libjpeg-turbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
  * 3.1.1
  * IJG (Independent JPEG Group) License or BSD-3-Clause License License

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
test.jpg has been output
````

You will see this jpeg image file.

<img src="./images/test.jpg" />

### Linux

````bash
$ ./install/linux/static/bin/Demo 
test.jpg has been output
````

<img src="./images/test.jpg" />

### OSX

````bash
$  ./install/osx/static/bin/Demo
test.jpg has been output
````

<img src="./images/test.jpg" />