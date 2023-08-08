# broaty

## Abstracts

* Usage of broaty

## Requirements

* Powershell 7 or later
* CMake 3.5.0 or later

## Linux

* GCC

## OSX

* XCode
  * Latest clang compiler can not build `gtest` so made disable building it
    * [Doesn't build on Ubuntu 21.10 / clang-13](https://github.com/google/bloaty/issues/296)

## Dependencies

* [broaty](https://github.com/google/bloaty)
  * v1.1
  * Apache-2.0 License

## How to usage?

````cmd
$ pwsh build.ps1 <Release/Debug>
````