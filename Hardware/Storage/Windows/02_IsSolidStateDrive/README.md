# Is Solid State Drive?

## Abstracts

* Determine physical drive is solid state drive or not
  * Check whether specified physical drive incurs a seek penalty 
    * Refer [DEVICE_SEEK_PENALTY_DESCRIPTOR structure](https://learn.microsoft.com/en-us/windows/win32/api/winioctl/ns-winioctl-device_seek_penalty_descriptor)

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