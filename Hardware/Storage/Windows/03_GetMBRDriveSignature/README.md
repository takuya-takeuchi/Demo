# Get Drive Signature of MBR

## Abstracts

* Get Drive Signature of MBR from Physical DriveName (e.g. `\\.\PhysicalDrive0`)

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

This program requires administrative right.
And this program does not work for GPT drive.

````bat
$ install\win\bin\Test \\.\PhysicalDrive0
MBR Drive Signature: 7a55d92b
````