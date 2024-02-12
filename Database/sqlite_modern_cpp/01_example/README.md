# Example

## Abstracts

* How to use `sqlite_modern_cpp`

## Requirements

### Common

* Powershell
* CMake 3.5.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio 2022

### Linux

* g++

### Mac

* Xcode

## Dependencies

* [SQLite](https://www.sqlite.org/)
  * Public Domain license
* [sqlite_modern_cpp](https://github.com/SqliteModernCpp/sqlite_modern_cpp)
  * MIT license

## How to usage?

At first, you must build Botan on [sqlite_modern_cpp](..) directory.

````shell
$ pwsh build.ps1 <Debug/Release>
The new record got assigned id 2
20 bob 83.25
21 jack 68.5
count : 2
Age = 20, name = bob
scount : 2
````