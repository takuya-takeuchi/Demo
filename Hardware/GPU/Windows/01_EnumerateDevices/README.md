# Enumerating GPUs

## Abstracts

* Enumerating GPU device information
  * Check Support FP16 and Int8

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio
* Windows SDK 10

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ .\install\win\bin\Test.exe
Enumerating GPUs:
Adapter 0: NVIDIA GeForce GTX 1080
        VendorId: 4318
        DeviceId: 7040
        SubSysId: 862326824
        Revision: 161
        AdapterLuid: 89271-0
        Dedicated Video Memory: 8072 MB
        Dedicated System Memory: 0 MB
        Shared System Memory: 32673 MB
        FP16 (half precision) operations are supported: No
        INT8 operations (wave operations) operations are supported: Yes
Adapter 1: Intel(R) UHD Graphics 630
        VendorId: 32902
        DeviceId: 16018
        SubSysId: 139989032
        Revision: 0
        AdapterLuid: 92585-0
        Dedicated Video Memory: 128 MB
        Dedicated System Memory: 0 MB
        Shared System Memory: 32673 MB
        FP16 (half precision) operations are supported: Yes
        INT8 operations (wave operations) operations are supported: Yes
Adapter 2: NVIDIA GeForce GTX 1080
        VendorId: 4318
        DeviceId: 7040
        SubSysId: 862326824
        Revision: 161
        AdapterLuid: 4524789-0
        Dedicated Video Memory: 8072 MB
        Dedicated System Memory: 0 MB
        Shared System Memory: 32673 MB
        FP16 (half precision) operations are supported: No
        INT8 operations (wave operations) operations are supported: Yes
Adapter 3: Microsoft Basic Render Driver
        VendorId: 5140
        DeviceId: 140
        SubSysId: 0
        Revision: 0
        AdapterLuid: 92541-0
        Dedicated Video Memory: 0 MB
        Dedicated System Memory: 0 MB
        Shared System Memory: 32673 MB
        FP16 (half precision) operations are supported: Yes
        INT8 operations (wave operations) operations are supported: Yes
````