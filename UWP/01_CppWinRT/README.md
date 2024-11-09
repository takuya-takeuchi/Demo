# C++/WinRT

## Abstracts

* Miminal C++/WinRT without UI

## Requirements

### Common

* Powershell
* CMake 3.11.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio
* Windows SDK 10.0.17134.0 (Windows 10, version 1803) or later
  * C++/WinRT was introduces since 10.0.17134.0

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

This example does not launch from system.
Because app will terminate immediately.

But you can check behavior by Visual Studio Debbuger.