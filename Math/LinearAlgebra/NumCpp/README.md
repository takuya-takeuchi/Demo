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

* [NumCpp](https://github.com/dpilger26/NumCpp)
  * 2.11.0
  * MIT License

## How to usage?

````cmd
$ pwsh Build.ps1 <Release/Debug>
````

If enable `boost`, you have to change `-D NUMCPP_NO_USE_BOOST="ON"` to `-D NUMCPP_NO_USE_BOOST="OFF"` in [Build.ps1](./Build.ps1).