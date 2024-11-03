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

### Virtual Environmental

Go to [Cython](..).

## How to build?

````shell
$ python setup.py build_ext --inplace
$ pwsh build.ps1 <Debug/Release> <PythonPath, like C:\Python\3.11\x64\python.exe>
````

## How to test?

#### Windows

````bat
$ set PYTHONHOME=C:\Python\3.11\x64
$ set PYTHONPATH=C:\Python\3.11\x64\python.exe
[Info] execution directory: "D:\\Works\\OpenSource\\Demo\\Python\\Binding\\Cython\\00_GetStarted\\install\\win\\bin"
[Info] python script: PythonMod
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````

#### Linux

````bat
$ ./install/linux/bin/Demo 
[Info] import: PythonMod
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````

#### OSX

````bat
$ ./install/osx/bin/Demo 
[Info] execution directory: "/Users/xxxxxxx/Work/OpenSource/Demo2/Python/Binding/Cython/00_GetStarted/./install/osx/bin"
[Info] python script: PythonMod
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````