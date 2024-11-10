# C++/WinRT with co_await

## Abstracts

* Miminal C++/WinRT wait 5 sec when start up
  * App will terminate but you can see console window

## Requirements

### Common

* Powershell
* CMake 3.11.0 or later
* C++ Compiler supports C++20
  * **coroutine** is supported since C++20

### Windows

* Visual Studio
* Windows SDK 10.0.22000.0 (Windows 11 21H2) or later

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

Kick `install\win\10.0.22000.0\AppPackages\Demo_1.0.0.0_x64_Test\Install.ps1` and launch Demo app from system.

<img src="./images/app.gif" width="800" />