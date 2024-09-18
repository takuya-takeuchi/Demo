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

we can build it by myself.
Go to [OpenSSL](../../../Security/OpenSSL).

After built it, you must set `OPENSSL_ROOT_DIR` environmental variable to ignore system installed `OpenSSL`.
For example,

````shell
$ set OPENSSL_ROOT_DIR=D:\Works\OpenSource\Demo\Security\OpenSSL\install\win\openssl\3.3.2\static
````

## How to build?

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\static\bin\Demo.exe
[2024-09-08 21:54:35] [application] Connecting to wss://echo.websocket.org
[2024-09-08 21:54:35] [connect] Successful connection
[2024-09-08 21:54:36] [connect] WebSocket Connection 66.241.124.119:443 v-2 "WebSocket++/0.8.2" / 101
Request served by 7811941c69e658
````

#### Linux

````shell
$ uname -v
#46-Ubuntu SMP Tue Jul 12 10:30:17 UTC 2022
$ ./install/linux/static/bin/Demo
[2024-09-08 22:30:13] [application] Connecting to wss://echo.websocket.org
[2024-09-08 22:30:13] [connect] Successful connection
[2024-09-08 22:30:13] [connect] WebSocket Connection [2a09:8280:1::37:b5c3]:443 v-2 "WebSocket++/0.8.2" / 101
Request served by 1781505b56ee58
````

#### OSX

````shell
$  uname -v
Darwin Kernel Version 23.2.0: Wed Nov 15 21:59:33 PST 2023; root:xnu-10002.61.3~2/RELEASE_ARM64_T8112
$ ./install/osx/static/bin/Demo
[2024-09-08 21:52:08] [application] Connecting to wss://echo.websocket.org
[2024-09-08 21:52:09] [connect] Successful connection
[2024-09-08 21:52:10] [connect] WebSocket Connection [2a09:8280:1::37:b5c3]:443 v-2 "WebSocket++/0.8.2" / 101
Request served by 1781505b56ee58
````