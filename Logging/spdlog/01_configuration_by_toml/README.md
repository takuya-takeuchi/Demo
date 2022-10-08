# spdlog with configuration toml file

## Abstracts

* How to control behavior of spdlog by toml configuration file

## Requirements

* Visual Studio 2022
* Windows 10
* CMake version 3.10 or newer
  * You can install it via `winget install -e --id Kitware.CMake`

## Dependencies

* [spdlog](https://github.com/gabime/spdlog)
  * MIT License
* [spdlog_setup](https://github.com/guangie88/spdlog_setup)
  * MIT License

## How to usage?

You must execute `Build.ps1` to build dependencies and demo program.<br>
This script builds all project and you need not to launch Visual Studio.

````cmd
$ cd sources\Demo\bin\x64\Release
$ Demo.exe
[2022-06-18 22:32:55.047] [root] [info] Hello World!
````
