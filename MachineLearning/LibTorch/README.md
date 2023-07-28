# LibTorch

## Abstracts

* Build LibTorch as Static Library
* Run C++ code for LibTorch

## Requirements

### Common

* Powershell 7 or later
* CMake 3.5.0 or later
* C++ Compiler supports C++14
* Python 3.10 or later

### Windows

* Visual Studio 2022
  * I confirmed only v17.6.5 so I'm not sure other VS and compiler version works

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [pytroch](https://github.com/pytorch/pytorch)
  * 1.8.2
  * 3-clause BSD License

## How to use?

At first, you must build [pytroch](https://github.com/pytorch/pytorch).

````shell
$ python3 -m venv .venv
$ .venv\Scripts\activate
$ python -m pip install pip --upgrade
$ python -m pip install numpy pyyaml typing_extensions
$ pwsh build.ps1 <Debug/Release>
````

Then you can try samples. For example [00_Minimum](./00_Minimum).