# libpqxx

## Abstracts

* Build libpqxx

## Requirements

### Common

* Powershell 7 or later
* (Optional) The libpqxx 8.x versions require at least C++20. Make sure your compiler is up to date.

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [libpqxx](https://github.com/jtv/libpqxx)
  * 7.10.4
  * BSD-3-Clause license

## How to use?

````shell
$ pwsh download-libpq.ps1
$ pwsh build.ps1 <Debug/Release>
````