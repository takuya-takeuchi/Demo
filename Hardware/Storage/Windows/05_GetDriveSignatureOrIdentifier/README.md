# Get Drive Identifier or Drive Signature

## Abstracts

* Get Drive Identifier of GPT or Drive Signature of MBR from Physical DriveName (e.g. `\\.\PhysicalDrive0`)
  * This program does not require administrative right

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

### for GPT

````bat
$ install\win\bin\Test.exe \\.\PhysicalDrive0
GPT Drive Identifier: 8db8ba8b-adaf-473d-a9cb-64579d8028ae

$ install\win\bin\Test.exe \\.\PhysicalDrive1
GPT Drive Identifier: 4222cdcb-f8ec-4236-9f5c-c5c22be233b9
````

### for MBR

````bat
$ install\win\bin\Test.exe \\.\PhysicalDrive0
MBR Drive Signature: 7a55d92b-15d2-75c1-0000-000000000000
````