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

Not supported

## Build

````sh
$ pwsh Build.ps1 <Release/Debug>
````

## Run

### Consume 1 cpu by all threads

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

### Consume 2 cpu by all threads

````bash
$ ./install/linux/bin/Demo 0 1 0 1 0 1
Thread '1' is assigned to cpu '0'
Thread '2' is assigned to cpu '1'
Thread '3' is assigned to cpu '0'
Thread '4' is assigned to cpu '1'
Thread '5' is assigned to cpu '0'
Thread '6' is assigned to cpu '1'
````

Check cpu usages.

````bash
$ mpstat -P ALL 1
Linux 6.6.51+rpt-rpi-v8 (raspberrypi)   01/03/25        _aarch64_       (4 CPU)

16:19:54     CPU    %usr   %nice    %sys %iowait    %irq   %soft  %steal  %guest  %gnice   %idle
16:19:55     all   49.87    0.00    0.50    0.00    0.00    0.00    0.00    0.00    0.00   49.62
16:19:55       0   98.02    0.00    1.98    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:19:55       1  100.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:19:55       2    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
16:19:55       3    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
````

### Consume all cpu by all threads

````bash
$ ./install/linux/bin/Demo
Create 4 threads
````

Check cpu usages.

````bash
$ mpstat -P ALL 1
Linux 6.6.51+rpt-rpi-v8 (raspberrypi)   01/03/25        _aarch64_       (4 CPU)

16:22:34     CPU    %usr   %nice    %sys %iowait    %irq   %soft  %steal  %guest  %gnice   %idle
16:22:35     all   98.75    0.00    1.25    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:22:35       0   96.00    0.00    4.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:22:35       1  100.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:22:35       2  100.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
16:22:35       3   99.00    0.00    1.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
````