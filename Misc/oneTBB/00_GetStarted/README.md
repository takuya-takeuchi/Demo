# Get started

## Abstracts

* Benchmark of Parallel pipeline and sequential process

## Requirements

### Common

* Powershell 7 or later
* CMake 3.5 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [oneTBB](https://github.com/uxlfoundation/oneTBB)
  * v2022.2.0
  * Apache-2.0 License

## How to build?

First, go to [oneTBB](..).
Once time you built `oneTBB`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ install\win\dynamic\bin\Demo.exe
[Parallel] token_num = 1
Processed 100 items in 18.6983 seconds.
Throughput: 5.34808 items/sec
[Parallel] token_num = 2
Processed 100 items in 9.4208 seconds.
Throughput: 10.6148 items/sec
[Parallel] token_num = 4
Processed 100 items in 6.35091 seconds.
Throughput: 15.7458 items/sec
[Parallel] token_num = 8
Processed 100 items in 6.34747 seconds.
Throughput: 15.7543 items/sec
[Sequential] (for loop)
Processed 100 items in 18.6695 seconds.
Throughput: 5.35634 items/sec
````

### Linux

````bash
$ LD_LIBRARY_PATH=./install/linux/dynamic/bin  ./install/linux/dynamic/bin/Demo
[Parallel] token_num = 1
Processed 100 items in 15.0517 seconds.
Throughput: 6.64379 items/sec
[Parallel] token_num = 2
Processed 100 items in 7.57552 seconds.
Throughput: 13.2004 items/sec
[Parallel] token_num = 4
Processed 100 items in 5.28019 seconds.
Throughput: 18.9387 items/sec
[Parallel] token_num = 8
Processed 100 items in 5.29055 seconds.
Throughput: 18.9016 items/sec
[Sequential] (for loop)
Processed 100 items in 15.048 seconds.
Throughput: 6.64539 items/sec
````

### OSX

````bash
$  ./install/osx/static/bin/Demo
en0 :
ap1 :
en1 :
llw0 :
bridge100 :
utun0 :
utun1 :
utun2 :
utun3 :
utun4 :
utun5 :
utun6 :
utun7 :
utun8 :
utun9 :
lo0 :
anpi0 :
anpi1 :
en4 :
en5 :
en2 :
en3 :
bridge0 :
gif0 :
stf0 :
awdl0 :
````