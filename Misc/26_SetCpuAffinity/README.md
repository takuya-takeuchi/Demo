# Set cpu affinity

## Description

* How to set cpu affinity mask for C++ std library

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Build

````sh
$ pwsh Build.ps1 <Release/Debug>
````

## Run

````bash
$ ./install/linux/bin/Demo 0 0 0 0
Thread '1' is assigned to cpu '0'
Thread '2' is assigned to cpu '0'
Thread '3' is assigned to cpu '0'
Thread '4' is assigned to cpu '0'
````

Check cpu usages.

````bash
$ mpstat -P ALL 1
Linux 6.6.51+rpt-rpi-v8 (raspberrypi)   01/03/25        _aarch64_       (4 CPU)

12:34:16     CPU    %usr   %nice    %sys %iowait    %irq   %soft  %steal  %guest  %gnice   %idle
12:34:17     all   25.13    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00   74.87
12:34:17       0  100.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
12:34:17       1    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
12:34:17       2    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
12:34:17       3    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
````