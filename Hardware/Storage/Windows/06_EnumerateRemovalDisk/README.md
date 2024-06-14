# Enumerate Removal Disk

## Abstracts

* Enumerate removal disk device path

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
$ install\win\bin\Test
Found Removable Disk: \\?\scsi#disk&ven_buffalo&prod_ssd-pgu3#nl#6&ce1a17c&1&000000#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}
Found Removable Disk: \\?\usbstor#disk&ven_tdk_lor&prod_tf10&rev_pmap#0703448b91511325&0#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}
Found Removable Disk: \\?\scsi#disk&ven_realtek&prod_rtl9210b-cg#6&560aeb3&0&000000#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}
````