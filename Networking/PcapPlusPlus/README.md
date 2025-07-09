# PcapPlusPlus

## Abstracts

* Build PcapPlusPlus
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.12 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Npcap](https://npcap.com/#download)
  * 1.15
  * [License](https://npcap.com/oem/)
* [PcapPlusPlus](https://github.com/seladb/PcapPlusPlus)
  * v25.05
  * Unlicense License

## How to build?

### Windows

````shell
$ git submodule update --init --recursive .
$ pwsh download-npcap.ps1
$ pwsh build.ps1 <Debug/Release>
````

### OSX

````shell
$ git submodule update --init --recursive .
$ brew install libpcap
$ pwsh build.ps1 <Debug/Release>
````