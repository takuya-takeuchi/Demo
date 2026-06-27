# Get started

## Abstracts

* Link libpqxx binary
* Show libpqxx version

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

* [libpqxx](https://github.com/jtv/libpqxx)
  * BSD-3-Clause license

## How to build?

### GStreamer

Go to [libpqxx](..).

Once time you built `libpqxx`, you need not to do again.

````shell
$ pwsh download-libpq.ps1
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
libpqxx Version: 7.10.4
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