# UDP Server

## Abstracts

* Simple UDP server
  * Specify listen ip address and port

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* N/A

## How to build?

````shell
$ pwsh build.ps1  <Debug/Release>
````

## How to use?

Start program to wait data.

````shell
$ ./build/osx/program/Demo 127.0.0.1 3000
````

Send UDP data.

````shell
$ echo "Hello, UDP Server" | nc -u -w1 127.0.0.1 3000
````

Program receive data and output message on console.

````shell
$ Received: Hello, UDP Server
````