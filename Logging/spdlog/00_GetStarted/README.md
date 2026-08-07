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
[2026-08-07 14:29:42.291] [info] spdlog Version: 11700
````

#### Linux

````shell
$ ./install/linux/bin/Demo 
[2026-08-07 21:49:21.892] [info] spdlog Version: 11700
````

#### OSX

````shell
$ ./install/osx/bin/Demo 
[2026-08-07 22:48:15.160] [info] spdlog Version: 11700
````