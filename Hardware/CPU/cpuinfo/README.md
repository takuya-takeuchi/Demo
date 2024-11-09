# cpuinfo

## Abstracts

* Usage of cpuinfo

## Requirements

* Powershell 7 or later
* CMake 3.5.0 or later
* C++ Compiler supports C++17

## Windows

* Visual Studio 2022

## Linux

* GCC

## OSX

* XCode

## iOS

* XCode

## Android

* Android SDK
* Ninja

## Dependencies

* [cpuinfo](https://github.com/pytorch/cpuinfo)
  * d7069b3919d1b65da5e8e333cb5817570a30b49a
  * BSD 2-Clause "Simplified" License

## How to usage?

````cmd
$ pwsh build.ps1 <win/linux/osx/iphoneos/iphonesimulator/android> <x86_64/armv7/arm64> <Release/Debug>
````

If you want to test on Mac, iPhone and iPhone simulator, you should build for all configuration and create xcframework.

````sh
$ pwsh build.ps1 osx x86_64 Release
$ pwsh build.ps1 osx arm64 Release
$ pwsh build.ps1 iphoneos arm64 Release
$ pwsh build.ps1 iphonesimulator x86_64 Release
$ pwsh build.ps1 iphonesimulator arm64 Release
$ pwsh CreateXCFramework.ps1
````

### Intel Core i7

````cmd
Packages:
        0: Intel Core i7-8700
Microarchitectures:
        6x Sky Lake
Cores:
        0: 2 processors (0-1), Intel Sky Lake
        1: 2 processors (2-3), Intel Sky Lake
        2: 2 processors (4-5), Intel Sky Lake
        3: 2 processors (6-7), Intel Sky Lake
        4: 2 processors (8-9), Intel Sky Lake
        5: 2 processors (10-11), Intel Sky Lake
Logical processors:
        0: APIC ID 0x00000000
        1: APIC ID 0x00000001
        2: APIC ID 0x00000002
        3: APIC ID 0x00000003
        4: APIC ID 0x00000004
        5: APIC ID 0x00000005
        6: APIC ID 0x00000006
        7: APIC ID 0x00000007
        8: APIC ID 0x00000008
        9: APIC ID 0x00000009
        10: APIC ID 0x0000000a
        11: APIC ID 0x0000000b
````

### Intel Xeon E5

````cmd
Packages:
        0: Intel Xeon E5-1607 v3
Microarchitectures:
        4x Haswell
Cores:
        0: 1 processor (0), Intel Haswell
        1: 1 processor (1), Intel Haswell
        2: 1 processor (2), Intel Haswell
        3: 1 processor (3), Intel Haswell
Logical processors (System ID):
        0 (0): APIC ID 0x00000000
        1 (1): APIC ID 0x00000001
        2 (2): APIC ID 0x00000002
        3 (3): APIC ID 0x00000003
````