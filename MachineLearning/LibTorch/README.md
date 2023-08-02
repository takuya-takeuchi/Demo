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
  * I confirmed only 14.1 and 14.2. At least, 14.3 does not work.

## Dependencies

* [pytroch](https://github.com/pytorch/pytorch)
  * 1.8.2
  * 3-clause BSD License

## How to use?

At first, you must build [pytroch](https://github.com/pytorch/pytorch).

### Windows

````shell
$ python3 -m venv .venv
$ .venv\Scripts\activate
$ python -m pip install pip --upgrade
$ python -m pip install numpy pyyaml typing_extensions
$ pwsh build.ps1 <Debug/Release>
````

### Linux/OSX

````shell
$ python3 -m venv .venv
$ source .venv/bin/activate
$ python -m pip install pip --upgrade
$ python -m pip install numpy pyyaml typing_extensions
$ pwsh build.ps1 <Debug/Release>
````

For OSX, you can change build arch by changing `$env:CMAKE_OSX_ARCHITECTURES="x86_64"` to `$env:CMAKE_OSX_ARCHITECTURES="arm64"` in [build.ps1](./build.ps1).