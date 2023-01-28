# PATCH

## Abstracts

* POST sample program
  * Invoke /patch of https://httpbin.org/

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
    "X-Amzn-Trace-Id": "Root=1-63d57682-6b54bf902b4b5ccd258c79e1"
  },
  "json": {
    "message": "Hello http"
  },
  "origin": "XXX.XXX.XXX.XXX",
  "url": "https://httpbin.org/patch"
}
````