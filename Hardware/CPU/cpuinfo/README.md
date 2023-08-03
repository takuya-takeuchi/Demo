# NumCpp

## Abstracts

* Usage of NumCpp

## Requirements

* Powershell 7 or later
* CMake 3.14.0 or later
* C++ compiler support C++ 17

## Windows

* Visual Studio 2022

## Linux

* GCC

## OSX

* XCode

## Dependencies

* [cpuinfo](https://github.com/pytorch/cpuinfo)
  * d7069b3919d1b65da5e8e333cb5817570a30b49a
  * BSD 2-Clause "Simplified" License

## How to usage?

````cmd
$ pwsh Build.ps1 <Release/Debug>
````

If enable `boost`, you have to change `-D NUMCPP_NO_USE_BOOST="ON"` to `-D NUMCPP_NO_USE_BOOST="OFF"` in [Build.ps1](./Build.ps1).