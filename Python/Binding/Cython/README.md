# Cython

## Abstracts

* Binding C++/Python by Cython
  * To be precise, C++ side uses only python libs

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17
* Python 3

### Windows

* Visual Studio

### Ubuntu

* g++
* `apt install python3-venv`

### OSX

* Xcode

## Dependencies

* [Cython](https://github.com/cython/cython)
  * Version 2.13.6
  * Apache-2.0 License

## How to setup?

This script create virtual environmental.

````shell
$ pwsh setup.ps1 <PythonPath, like C:\Python\3.10\x64\python.exe>
````

Then, you can activate virtual environmental.

#### Windows

````shell
$ .\venv\Scripts\activate
````

#### Linux

````shell
$ source
````