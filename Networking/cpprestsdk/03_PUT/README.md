# PUT

## Abstracts

* POST sample program
  * Invoke /put of https://httpbin.org/

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio
* cpprestsdk
  * via `vcpkg` command with `--triplet x64-windows-static`

### Ubuntu

* g++
* libcpprest-dev
  * via `apt` command

### OSX

* Xcode
* cpprestsdk
  * via `brew` command

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
  * MIT License
* [Boost](https://www.boost.org/)
  * v1.81.0
  * Boost License

## How to usage?

You have to set `VCPKG_ROOT_DIR` envrironmental variable, like `C:\vcpkg` before build cpprestsdk on windows machine.

````shell
$ pwsh build.ps1  <Debug/Release>
{
  "args": {},
  "data": "{\"message\":\"Hello http\"}",
  "files": {},
  "form": {},
  "headers": {
    "Accept": "application/json",
    "Content-Length": "24",
    "Content-Type": "application/json",
    "Host": "httpbin.org",
    "User-Agent": "cpprestsdk/2.10.18",
    "X-Amzn-Trace-Id": "Root=1-63d5717c-67cc51ca6800e8345c5d8879"
  },
  "json": {
    "message": "Hello http"
  },
  "origin": "XXX.XXX.XXX.XXX",
  "url": "https://httpbin.org/put"
}
````