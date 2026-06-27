# libpqxx

## Abstracts

* Build libpqxx

## Requirements

### Common

* CMake 3.12 or higher
* Powershell 7 or later
* (Optional) The libpqxx 8.x versions require at least C++20. Make sure your compiler is up to date.

### Windows

* Visual Studio 2022

### Ubuntu

* g++
* `krb5-user`, `libgssapi-krb5-2`, `libkrb5-dev` and `libldap-dev`

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