# Get started

## Abstracts

* Binding C++/Python by Boost.Python

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

* [Boost](https://www.boost.org/)
  * v1.86.0
  * Boost License

## How to build?

### Boost

Go to [Boost.Python](..).
Once time you built `Boost`, you need not to do again.

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release> <BoostVersion, like 1.86.0> <PythonPath, like C:\Python\3.10\x64\python.exe>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
[Info] exec_file: PythonMod.py
[Info] invoke: add funtion
[Info] 2 + 5 = 7
````