# Get System Information (Type 1)

## Abstracts

* Show System Information (Type 1)
  * Refer P36 of [System Management BIOS (SMBIOS) Reference Specification](https://www.dmtf.org/sites/default/files/standards/documents/DSP0134_3.6.0.pdf)

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

## How to test?

````bat
$ install\win\bin\Test
SMBIOS version: 3.1
  DMI Revision: 0
  Total length: 5298
DMI at address: 000001815FE05228

System Information (Type 1)
         Manufacturer: Alienware
         Product Name: Alienware Aurora R7
              Version: 1.0.26
        Serial Number: XXXXXXX
                 UUID: 44454c4c-5900-104d-8056-c3c04f365032
           SKU Number: 0858
               Family: Alienware
````