# cpprestsdk from CMake

## Abstracts

* Build and link cpprestsdk by cmake
  * static build
* Minimal example program

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio
* OpenSSL
  * via `vcpkg` command with `--triplet x64-windows-static`
* zlib
  * via `vcpkg` command with `--triplet x64-windows-static`

### Ubuntu

* g++
* libz-dev
  * via `apt` command
* libssl-dev
  * via `apt` command

### OSX

* Xcode
* websocketpp
  * via `brew` command
    * cpprestsdk can not build due to https://github.com/zaphoyd/websocketpp/issues/1004

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
    * use e1b6a8e61d6b3ab98734b0532bad1da46458212a for only osx because [Fix likely typo in SafeInt3.hpp, that results in error with clang 15](https://github.com/microsoft/cpprestsdk/pull/1711)
  * MIT License
* [Boost](https://www.boost.org/)
  * v1.81.0
  * Boost License

## How to usage?

You have to set `VCPKG_ROOT_DIR` envrironmental variable, like `C:\vcpkg` before build cpprestsdk on windows machine.

````shell
$ pwsh build-boost.ps1 <Debug/Release>
$ pwsh build-cpprestsdk.ps1  <Debug/Release>
$ pwsh build.ps1  <Debug/Release>
{
  "args": {},
  "headers": {
    "Host": "httpbin.org",
    "User-Agent": "cpprestsdk/2.10.18",
    "X-Amzn-Trace-Id": "Root=1-63d54a6e-3724cf68667705a472c789c0"
  },
  "origin": "XXX.XXX.XXX.XXX",
  "url": "https://httpbin.org/get"
}
````