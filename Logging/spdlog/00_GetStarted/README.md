# Get started

## Abstracts

* Link spdlog binary
* Show spdlog version

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [spdlog](https://github.com/gabime/spdlog)
  * MIT license

## How to build?

### GStreamer

Go to [spdlog](..).

Once time you built `spdlog`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
spdlog Version: 11700
````

#### Linux

````shell
# If libpqxx is built as dynamic library
$ export LD_LIBRARY_PATH=./install/linux/bin
$ ./install/linux/bin/Demo
libpqxx Version: 7.10.4
````

#### OSX

````shell
# If libpqxx is built as dynamic library
$ export DYLD_LIBRARY_PATH=./install/osx/bin
$ ./install/osx/bin/Demo
libpqxx Version: 7.10.4
````