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

#### Windows

````bat
$ install\win\bin\Demo.exe
Interface: Hyper-V Virtual Ethernet Adapter - MAC Address: 00:15:5d:06:f6:b4
Interface: Hyper-V Virtual Ethernet Adapter #2 - MAC Address: 00:15:5d:ad:70:42
Interface: Hyper-V Virtual Ethernet Adapter #7 - MAC Address: 00:15:5d:8f:a1:19
Interface: Hyper-V Virtual Ethernet Adapter #3 - MAC Address: 00:15:5d:0b:15:1b
Interface: Hyper-V Virtual Ethernet Adapter #4 - MAC Address: 00:15:5d:0b:15:74
Interface: Hyper-V Virtual Ethernet Adapter #5 - MAC Address: 00:15:5d:0b:15:0d
Interface: Hyper-V Virtual Ethernet Adapter #6 - MAC Address: 00:15:5d:0b:15:8b
Interface: VirtualBox Host-Only Ethernet Adapter - MAC Address: 0a:00:27:00:00:2d
Interface: VMware Virtual Ethernet Adapter for VMnet1 - MAC Address: 00:50:56:c0:00:01
Interface: VMware Virtual Ethernet Adapter for VMnet8 - MAC Address: 00:50:56:c0:00:08
Interface: Bluetooth Device (Personal Area Network) #4 - MAC Address: 9c:b6:d0:fb:c2:74
Interface: Hyper-V Virtual Ethernet Adapter #9 - MAC Address: 00:15:5d:76:4c:f4
Interface: Hyper-V Virtual Ethernet Adapter #8 - MAC Address: d8:9e:f3:8a:17:5a
Interface: Hyper-V Virtual Ethernet Adapter #10 - MAC Address: 00:15:5d:00:62:9d
Interface: Hyper-V Virtual Ethernet Adapter #11 - MAC Address: 00:15:5d:e0:fb:32
Interface: Hyper-V Virtual Ethernet Adapter #12 - MAC Address: 00:15:5d:e4:68:57
Interface: Hyper-V Virtual Ethernet Adapter #13 - MAC Address: 00:15:5d:4c:59:f0
Interface: Hyper-V Virtual Ethernet Adapter #14 - MAC Address: 00:15:5d:2f:41:36
Interface: Hyper-V Virtual Ethernet Adapter #15 - MAC Address: 00:15:5d:78:ce:fd
Interface: Microsoft Network Adapter Multiplexor Driver - MAC Address: 9c:b6:d0:fb:c2:73
Interface: Hyper-V Virtual Ethernet Adapter #16 - MAC Address: 00:15:5d:87:ca:9b
Interface: Microsoft Wi-Fi Direct Virtual Adapter #3 - MAC Address: 9e:b6:d0:fb:c2:73
Interface: Microsoft Wi-Fi Direct Virtual Adapter #4 - MAC Address: ae:b6:d0:fb:c2:73
````

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