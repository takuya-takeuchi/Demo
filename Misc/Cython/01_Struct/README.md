# Simple code for Cython

## Abstracts

* Define `struct` and declare it

## Requirements

### Common

* Powershell
* CMake 3.12.0 or later
* C++ Compiler supports C++17
* Python 3

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Cython](https://github.com/cython/cython)
  * Apache License 2.0

## How to build?

### Windows

````bat
$ python -m venv .venv
$ .venv\Scripts\activate
$ python -m pip install cython setuptools
$ python setup.py build_ext --inplace
````

#### Linux

### OSX

````shell
$ python -m venv .venv
$ source .venv/bin/activate
$ python -m pip install cython setuptools
$ python setup.py build_ext --inplace
````

## How to run?

#### Windows

Set `PYTHONHOME` and `PYTHONPATH` before run program.

````bat
$ .venv\Scripts\activate
$ set PYTHONHOME=C:\Python\3.11\x64
$ set PYTHONPATH=C:\Python\3.11\x64\python.exe
$ set PATH=%PATH%;%PYTHONHOME% 
$ pwsh build.ps1 <Debug/Release>
$ .\install\win\bin\Demo.exe
p.x=3.50, p.y=7.00
````

#### Linux

#### OSX

Set `PYTHONHOME` and `PYTHONPATH` before run program.

````shell
$ source .venv/bin/activate
$ set PYTHONHOME=C:\Python\3.11\x64
$ set PYTHONPATH=C:\Python\3.11\x64\python.exe
$ set PATH=%PATH%;%PYTHONHOME% 
$ pwsh build.ps1 <Debug/Release>
$ ./install/osx/bin/Demo
p.x=3.50, p.y=7.00
````