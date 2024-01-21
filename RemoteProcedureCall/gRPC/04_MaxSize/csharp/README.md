# Control max message of Grpc

## Abstracts

* How to control max send/recerive message size

## Requirements

* .NET 8.0
* Powershell

## Dependencies

* [CommandLineParser](https://github.com/commandlineparser/commandline)
  * MIT License
* [Google.Protobuf](https://github.com/protocolbuffers/protobuf)
  * BSD-3-Clause License
* [Grpc](https://github.com/grpc/grpc)
  * Apache-2.0 License
* [Grpc.Net.Client](https://github.com/grpc/grpc-dotnet)
  * Apache-2.0 License
* [Grpc.Tools](https://github.com/grpc/grpc)
  * Apache-2.0 License
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License

## How to build

### 1. Restore nuget package

````shell
$ cd sources/Server
$ dotnet restore
````

### 2. Generate grpc client/server code

````shell
$ pwsh generate.ps1
````

### 3. Build Client/Server

For server side,

````shell
$ cd sources/Client
$ dotnet build
````

For client side,

````shell
$ cd sources/Server
$ dotnet build
````

## How to run?

At first, you must run server app with max size of receive and send message.
For example,

````shell
$ cd sources/Server
$ dotnet run -- -r <max length of receive message> `
                -s <max length of send message>
````

|Argument|Verb|Optional|Description|
|---|---|---|---|
|max length of receive message|-r|Yes|The maximum message size in bytes that can be received by the server. If the server receives a message that exceeds this limit, it throws an exception. If the client receives a message that exceeds this limit, it throws an exception. Default size of Grpc is 4194304. -1 means unlimited.|
|max length of send message|-s|Yes|The maximum message size in bytes that can be sent from the server. Attempting to send a message that exceeds the configured maximum message size results in an exception. Default size of Grpc is 4194304. -1 means unlimited.|

````shell
$ cd sources/Client
$ dotnet run -- -r <max length of receive message> `
                -s <max length of send message> `
                -q <size of request message> `
                -p <size of response message>
````

|Argument|Verb|Optional|Description|
|---|---|---|---|
|max length of receive message|-r|Yes|The maximum message size in bytes that can be received by the client. If the client receives a message that exceeds this limit, it throws an exception.|
|max length of send message|-s|Yes|The maximum message size in bytes that can be sent from the client. Attempting to send a message that exceeds the configured maximum message size results in an exception.|
|size of request message|-q|No|The size in bytes of `date` property of request message.|
|size of response message|-p|No|The size in bytes of `date` property of reeponse message.|

### Examples

#### 1. Limit send binary size by server side

On server side,

````shell
$ cd sources/Server
$ dotnet run -- -r 1024 -s 512
````

On client side,

````shell
$ cd sources/Client
$ dotnet run -- -r 20480 -s 10240 -q 2048 -p 1024
2024-01-21 22:16:42.6128 [INFO ] Start UnaryCall
2024-01-21 22:16:42.6343 [INFO ] Request Message size: 2054
2024-01-21 22:16:42.6343 [INFO ] Response Message size: 1027
2024-01-21 22:16:42.7661 [ERROR] UnaryCall: Status(StatusCode="ResourceExhausted", Detail="Received message larger than max (2054 vs. 1024)")
````

#### 2. Limit send binary size by client side

On server side,

````shell
$ cd sources/Server
$ dotnet run -- -r -1 -s -1
````

On client side,

````shell
$ cd sources/Client
$ dotnet run -- -r 20480 -s 10240 -q 204800 -p 1024
2024-01-21 22:17:45.1193 [INFO ] Start UnaryCall
2024-01-21 22:17:45.1504 [INFO ] Request Message size: 204807
2024-01-21 22:17:45.1504 [INFO ] Response Message size: 1027
2024-01-21 22:17:45.2585 [ERROR] UnaryCall: Status(StatusCode="ResourceExhausted", Detail="Sending message exceeds the maximum configured message size.")
````

#### 3. Limit receive binary size by server side

On server side,

````shell
$ cd sources/Server
$ dotnet run -- -r 1024 -s 512
````

On client side,

````shell
$ cd sources/Client
$ dotnet run -- -r 20480 -s 10240 -q 100 -p 10240
2024-01-21 22:19:05.1496 [INFO ] Start UnaryCall
2024-01-21 22:19:05.1830 [INFO ] Request Message size: 105
2024-01-21 22:19:05.1830 [INFO ] Response Message size: 10243
2024-01-21 22:19:05.3353 [ERROR] UnaryCall: Status(StatusCode="ResourceExhausted", Detail="Sent message larger than max (10243 vs. 512)")
````

#### 4. Limit receive binary size by client side

On server side,

````shell
$ cd sources/Server
$ dotnet run -- -r -1 -s -1
````

On client side,

````shell
$ cd sources/Client
$ dotnet run -- -r 2048 -s 1024 -q 512 -p 10240
2024-01-21 22:22:42.1995 [INFO ] Start UnaryCall
2024-01-21 22:22:42.2347 [INFO ] Request Message size: 518
2024-01-21 22:22:42.2347 [INFO ] Response Message size: 10243
2024-01-21 22:22:42.3790 [ERROR] UnaryCall: Status(StatusCode="ResourceExhausted", Detail="Received message exceeds the maximum configured message size.")
````