# spdlog with configuration json file

## Abstacts

* How to control behavior of spdlog by toml configuration file

## Requirements

* Visual Studio 2022
  * You must install `C++/WinRT` on Visual Studio Installer

[![workload](./images/workload.png "workload")](./images/workload.png)

* Windows 10
* CMake version 3.14 or newer
  * You can install it via `winget install -e --id Kitware.CMake`
* Language Pack
  * e.g. Install english language if you want to recognize english text

## Dependencies

* [Microsoft.Windows.CppWinRT](https://github.com/Microsoft/cppwinrt)
  * MIT License
* [spdlog](https://github.com/gabime/spdlog)
  * MIT License

## How to usage?

You must execute `Build.ps1` to build dependencies.

````cmd
$ sources\Demo\bin\x64\Release\Demo.exe ja testdata\ja.png

````
