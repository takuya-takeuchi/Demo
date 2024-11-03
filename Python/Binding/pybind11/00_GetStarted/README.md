# Get started

## Abstracts

* Binding C++/Python by pybind11

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.0 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [pybind11](https://github.com/pybind/pybind11)
  * Version 2.13.6
  * BSD-3-Clause License

## How to build?

### Boost

Go to [pybind11](..).
Once time you built `pybind11`, you need not to do again.

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release> <version of pybind11, like 2.13.6> <PythonPath, like C:\Python\3.11\x64\python.exe>
````

## How to test?

#### Windows

````bat
$ set PYTHONHOME=C:\Python\3.11\x64
$ set PYTHONPATH=C:\Python\3.11\x64\python.exe
$ .\install\win\bin\Demo.exe
[Info] import: PythonMod
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````

#### Linux

````bat
$ ./install/linux/bin/Demo 
[Info] exec_file: PythonMod.py
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````

#### OSX

````bat
$ ./install/osx/bin/Demo 
[Info] exec_file: PythonMod.py
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````