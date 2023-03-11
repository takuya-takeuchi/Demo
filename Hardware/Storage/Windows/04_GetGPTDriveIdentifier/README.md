# Get Drive Identifier of GPT

## Abstracts

* Get Drive Identifier of GPT from Physical DriveName (e.g. `\\.\PhysicalDrive0`)

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

````bat
$ install\win\bin\Test.exe \\.\PhysicalDrive0
GPT Drive Identifier: 8db8ba8b-adaf-473d-a9cb-64579d8028ae

$ install\win\bin\Test.exe \\.\PhysicalDrive1
GPT Drive Identifier: 4222cdcb-f8ec-4236-9f5c-c5c22be233b9
````