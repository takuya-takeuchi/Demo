# Enumerate MAC address

## Abstracts

* Enumerate MAC address

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Linux

* g++

### OSX

* Xcode

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to use?

#### Linux

````shell
$ ./install/linux/bin/Demo 
Interface: lo - MAC Address: 00:00:00:00:00:00
Interface: enp3s0 - MAC Address: f8:bc:12:5a:1f:5d
Interface: docker0 - MAC Address: 02:42:f2:69:c1:5e
````

#### OSX

````shell
$ ./install/linux/bin/Demo 
Interface: lo - MAC Address: 00:00:00:00:00:00
Interface: enp3s0 - MAC Address: f8:bc:12:5a:1f:5d
Interface: docker0 - MAC Address: 02:42:f2:69:c1:5e
````