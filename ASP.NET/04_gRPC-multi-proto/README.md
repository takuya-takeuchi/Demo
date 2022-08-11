# gRPC with multiple proto files

## Abstacts

* How to reference multiple proto files
* How to import other proto file
* How to test gRPC service from unit test project

## Requirements

* Visual Studio 2022
* .NET 6.0 SDK
* PowerShell Core 6.2 or later

## Dependencies

* [Grpc.AspNetCore](https://github.com/grpc/grpc-dotnet)
  * Apache License 2.0
* [Grpc.AspNetCore.Server.Reflection](https://github.com/grpc/grpc-dotnet)
  * Apache License 2.0
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
* [NLog.Web.AspNetCore](https://github.com/NLog/NLog.Web)
  * BSD-3-Clause License

### for Test only

* [converlet.coolector](https://github.com/coverlet-coverage/coverlet)
  * MIT License
* [Google.Protobuf](https://github.com/protocolbuffers/protobuf)
  * BSD-3-Clause License
* [Grpc.Net.Client](https://github.com/grpc/grpc-dotnet)
  * Apache License 2.0
* [Microsoft.NET.Test.Sdk](https://github.com/microsoft/vstest/)
  * MIT License
* [MSTest.TestAdapter](https://github.com/microsoft/testfx)
  * MIT License
* [MSTest.TestFramework](https://github.com/microsoft/testfx)
  * MIT License

## How to use

````bat
$ dotnet run -c Release --project sources\Demo\Demo.csproj
````

### gRPC Web UI

````bat
$ tools\grpcui\windows\v1.3.0\grpcui.exe localhost:5001
````

## How to test

At first, launch service and then,

````bat
$ pwsh GeneratePrrotoCode.ps1
$ cd tests\Demo.Test
$ dotnet test -c Release
  復元対象のプロジェクトを決定しています...
  復元対象のすべてのプロジェクトは最新です。
  Demo.Test -> E:\Works\OpenSource\Demo\ASP.NET\04_gRPC-multi-proto\tests\Demo.Test\bin\Release\net6.0\Demo.Test.dll
E:\Works\OpenSource\Demo\ASP.NET\04_gRPC-multi-proto\tests\Demo.Test\bin\Release\net6.0\Demo.Test.dll (.NETCoreApp,Version=v6.0) のテスト実行
Microsoft (R) Test Execution Command Line Tool Version 17.2.0 (x64)
Copyright (c) Microsoft Corporation.  All rights reserved.

テスト実行を開始しています。お待ちください...
合計 1 個のテスト ファイルが指定されたパターンと一致しました。

成功!   -失敗:     0、合格:     4、スキップ:     0、合計:     4、期間: 202 ms - Demo.Test.dll (net6.0)
````
