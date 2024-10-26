# Hash

# Abstracts

* How to use Botan library
* How to calc hash of argument text messgae

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++20
* Python 3 or later

### Windows

* Visual Studio 2022

### Linux

* g++

### Mac

* Xcode

## Dependencies

* [Botan](https://github.com/randombit/botan)
  * BSD-2-Clause license

## How to usage?

At first, you must build Botan on [Botan](..) directory.

````shell
$ pwsh build-Botan.ps1 <Debug/Release>
````

After that, script generates artifacts so you can run the following command.

````shell
$ pwsh build.ps1 <Debug/Release>
````