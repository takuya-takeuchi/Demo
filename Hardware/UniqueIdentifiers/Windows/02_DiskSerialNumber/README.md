# Show Disk Serial Number

## Abstracts

* Print disk serial number

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
$ install\win\bin\Test.exe 1
serial number: 00A0_7563_E000_0163.
````

You can use `wmic` to check just in case.

````bat
$ wmic diskdrive get index,model,serialnumber                                                                                       
Index  Model                                 SerialNumber
2      BUFFALO SSD-PGU3/NL SCSI Disk Device
0      WDC WD30EFRX-68EUZN0                  WD-WCC4N3RKP08A
4      TDK LoR TF10 USB Device               017302354040
3      Realtek RTL9210B-CG SCSI Disk Device  0000000000000000
1      CT1000P2SSD8                          00A0_7563_E000_0163.
````