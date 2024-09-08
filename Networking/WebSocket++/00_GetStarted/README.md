# Get started

## Abstracts

* Connect to WSS (WebSocket over SSL/TLS) server

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.5 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [Boost](https://www.boost.org/)
  * v1.86.0
  * Boost License
* [OpenSSL](https://www.openssl.org/)
  * 3.0 or later: Apache License 2.0
  * 1.x and earlier: OpenSSL License
* [WebSocket++](https://github.com/zaphoyd/websocketpp)
  * 0.8.2
  * BSD-3-Clause License

## How to build?

### WebSocket++ and Boost

Go to [WebSocket++](..).
Once time you built `WebSocket++` and `Boost`, you need not to do again.

### OpenSSL

For Windows, we can build it by myself.
Go to [OpenSSL](../../../Security/OpenSSL).

After built it, you must set `OpenSSL_DIR` environmental variable.
For example,

````shell
$ set OpenSSL_DIR=D:\Works\OpenSource\Demo\Security\OpenSSL\install\win\openssl\3.3.2\static
````

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\static\bin\Demo.ex
[2024-09-08 21:54:35] [application] Connecting to wss://echo.websocket.org
[2024-09-08 21:54:35] [connect] Successful connection
[2024-09-08 21:54:36] [connect] WebSocket Connection 66.241.124.119:443 v-2 "WebSocket++/0.8.2" / 101
Request served by 7811941c69e658
````

#### Linux

````bat
$ ./install/osx/static/bin/Demo
[Info] output tensor: {1, 10}
````

#### OSX

````shell
$ ./install/osx/static/bin/Demo
[2024-09-08 21:52:08] [application] Connecting to wss://echo.websocket.org
[2024-09-08 21:52:09] [connect] Successful connection
[2024-09-08 21:52:10] [connect] WebSocket Connection [2a09:8280:1::37:b5c3]:443 v-2 "WebSocket++/0.8.2" / 101
Request served by 1781505b56ee58
````