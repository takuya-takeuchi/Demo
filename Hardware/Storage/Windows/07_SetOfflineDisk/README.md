# Make disk offline 

## Abstracts

* Make connected drive offline

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
$ install\win\bin\Test "\\?\usbstor#disk&ven_tdk_lor&prod_tf10&rev_pmap#0703448b91511325&0#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}"
IOCTL_VOLUME_OFFLINE failed: この要求はサポートされていません。
````