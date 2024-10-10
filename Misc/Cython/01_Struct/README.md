# Simple code for Cython

## Abstracts

* Define `struct` and declare it

## Requirements

### Common

* Powershell
* CMake 3.12.0 or later
* C++ Compiler supports C++17

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
$ python -m pip install cython
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
$ .\install\win\bin\Demo.exe
p.x=3.500000, p.y=7.000000
````

#### OSX

````bat
$ ./install/osx/bin/Test "http://localhost:9000"
[Info] message: Hello!!
[Info]    date: 2024-04-13T13:02:18.355794Z
````

#### Linux

````bat
$ ./install/linux/bin/Test "http://localhost:9000"
[Info] message: Hello!!
[Info]    date: 2024-04-13T13:52:37.9457844Z
````