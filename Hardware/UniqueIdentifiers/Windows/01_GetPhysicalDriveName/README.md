# Get Physical DriveName

## Abstracts

* Get Physical DriveName (e.g. `\\.\PhysicalDrive0`) from dirve name (e.g. `C:\`)

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
$ install\win\bin\Test  "C:\"
\\.\PhysicalDrive1 
````