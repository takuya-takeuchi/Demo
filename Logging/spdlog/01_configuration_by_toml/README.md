# Configuration by toml file

## Abstracts

* Use spdlog via toml file

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
* [spdlog_setup](https://github.com/guangie88/spdlog_setup)
  * MIT license

## How to build?

### GStreamer

Go to [spdlog](..).

Once time you built `spdlog` and `spdlog_setup`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe
[2026-08-07 15:28:28.750] [root] [info] Hello World!
````

#### Linux

````shell
$ ./install/linux/bin/Demo 
[2026-08-07 23:07:16.253] [root] [info] Hello World!
````

#### OSX

````shell
$ ./install/osx/bin/Demo 
[2026-08-07 22:53:24.531] [root] [info] Hello World
````