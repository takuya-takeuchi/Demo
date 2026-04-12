# gRPC (C API)

## Abstracts

* Build gRPC

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.16 or higher

### Windows

* Visual Studio 2022

##### NOTE

Not supported.

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [gRPC](https://github.com/grpc/grpc)
  * v1.80.0
  * Apache-2.0 license
* [protobuf](https://github.com/protocolbuffers/protobuf)
  * v34.1
  * BSD-3-Clause license
* [zlib](https://github.com/madler/zlib)
  * v1.3.2
  * zlib License

## How to use?

````shell
$ pwsh build-protobuf.ps1 <Debug/Release>
$ pwsh build-zlib.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````