# Show BackTrace

## Abstracts

* How to show BtackTrace by gdb

## Requirements

* Powershell 7 or later
* CMake 3.5.0 or later

## Linux

* GCC

## Dependencies

N/A

## How to usage?

````bash
$ pwsh build.ps1 Debug
````

Then, make enable core dump.

````bash
$ ulimit -c unlimited
````

At last, kick program with `gdb`.

````bash
$ gdb -batch -ex "bt" ./install/linux/Debug/bin/Test my_core.dump
0
1
2
3
4
5
0
0
...
0
0

Program received signal SIGSEGV, Segmentation fault.
0x000000000040228b in main () at /home/t-takeuchi/Work/Demo/DevTools/gdb/01_ShowBackTrace/main.cpp:8
8               std::cout << vec[i] << std::endl;
#0  0x000000000040228b in main () at /home/t-takeuchi/Work/Demo/DevTools/gdb/01_ShowBackTrace/main.cpp:8
````