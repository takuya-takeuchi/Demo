# Get started

## Abstracts

* Listing unnecessary `#include`

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

* [include-what-you-use](https://github.com/include-what-you-use/include-what-you-use)
  * LLVM Release License

## How to build?

### GStreamer

Go to [libpqxx](..).

Once time you built `libpqxx`, you need not to do again.

````shell
$ pwsh download-libpq.ps1
$ pwsh build.ps1 <Debug/Release>
````

## How to do?

This demo lists unnecessary `#include` when build.

#### Windows

````bat
$ .\install\win\bin\Demo.exe
libpqxx Version: 7.10.4
````

#### Linux

````shell
$ pwsh build.ps1 <Debug/Release>
-- Install configuration: "Release"
-- Up-to-date: /data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/install/linux/bin/Demo

/data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should add these lines:

/data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp should remove these lines:
- #include <fstream>  // lines 1-1

The full include-list for /data/work/oss/Demo/StaticProgramAnalysis/C++/IncludeWhatYouUse/00_GetStarted/main.cpp:
#include <iostream>  // for char_traits, basic_ostream, cout, endl, operator<<
---
````

#### OSX

````shell
# If libpqxx is built as dynamic library
$ export DYLD_LIBRARY_PATH=./install/osx/bin
$ ./install/osx/bin/Demo
libpqxx Version: 7.10.4
````