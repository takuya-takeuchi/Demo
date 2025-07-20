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
$ $ ./install/linux/static/bin/Demo 
ens160 : 
any : Pseudo-device that captures on all interfaces
lo : 
docker0 : 
bluetooth-monitor : Bluetooth Linux Monitor
nflog : Linux netfilter log (NFLOG) interface
nfqueue : Linux netfilter queue (NFQUEUE) interface
dbus-system : D-Bus system bus
dbus-session : D-Bus session bus
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