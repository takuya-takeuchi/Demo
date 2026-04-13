# Get started

## Abstracts

* Simple client and server program

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## How to build?

### ONNX Runtime

Go to [gRPC/C](..).

Once time you built `abseil`, `zlib`, `protobuf` and `gRPC`, you need not to do again.

````shell
$ pwsh build-abseil.ps1 <Debug/Release>
$ pwsh build-zlib.ps1 <Debug/Release>
$ pwsh build-protobuf.ps1 <Debug/Release>
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

At first, launch server program.

````bat
$ .\install\win\bin\server.exe
gRPC server listening on 0.0.0.0:50051
````

Next, kick client program.

````bat
$ .\install\win\bin\client.exe
````

Then, server side console show message from client.

````bat
Received request: name=client
````

#### Linux

At first, launch server program.

````bash
$ ./install/linux/bin/server
gRPC server listening on 0.0.0.0:50051
````

Next, kick client program.

````bash
$ ./install/linux/bin/client
````

Then, server side console show message from client.

````bash
Received request: name=client
````

#### OSX

At first, launch server program.

````bash
$ ./install/osx/bin/server
gRPC server listening on 0.0.0.0:50051
````

Next, kick client program.

````bash
$ ./install/osx/bin/client
````

Then, server side console show message from client.

````bash
Received request: name=client
````