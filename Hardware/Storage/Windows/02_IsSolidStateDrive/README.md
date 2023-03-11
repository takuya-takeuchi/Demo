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
$ install\win\bin\Test \\.\PhysicalDrive0
\\.\PhysicalDrive0 has seek penalty. It's not solid state drive.

$ install\win\bin\Test \\.\PhysicalDrive1
\\.\PhysicalDrive1 has no seek penalty. It's solid state drive.

$ install\win\bin\Test \\.\PhysicalDrive 
Failed to retrieve the status.
````