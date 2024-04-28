# Input Health Check Log and Output as File

## Abstracts

* Input Windows Event Log and output it as File
  * Use input `health` and output `file` plugins
    * `health` plugins is not enabled for Windows. Please refer [windows-setup.cmake](https://github.com/fluent/fluent-bit/blob/v3.0.2/cmake/windows-setup.cmake#L36).

## Requirements

* Powershell 7 or later
* .NET 6.0 SDK

## Dependencies

* [FluentBit](https://fluentbit.io/)
  * 3.0.2
  * Apache License 2.0

## Configuration

Here is configuration of input and output plugins.

#### INPUT

|Key|Value|
|---|---|
|name|health|
|host|127.0.0.1|
|port|5001|
|interval_sec|1|
|interval_nsec|0|

#### OUTPUT

|Key|Value|
|---|---|
|Name|file|
|Path|${PWD}/logs|
|Mkdir|True|
|Workers|1|

## How to run?

Af first, you must download or install fluentbit.
Please refer [README.md](../00_GetStarted/README.md).

Then, launch sample http application.

````shell
$ cd http-service
$ dotnet run -c Release --urls http://localhost:5001
Building...
info: Microsoft.Hosting.Lifetime[14]
      Now listening on: http://localhost:5001
info: Microsoft.Hosting.Lifetime[0]
      Application started. Press Ctrl+C to shut down.
info: Microsoft.Hosting.Lifetime[0]
      Hosting environment: Development
info: Microsoft.Hosting.Lifetime[0]
      Content root path: /Users/t-takeuchi/Work/OpenSource/Demo/Logging/FluentBit/03_HealthCheckToFile/http-service/
^Cinfo: Microsoft.Hosting.Lifetime[0]
      Application is shutting down...
````

Lastly, kick `Run.ps1`

````shell
$ pwsh Run.ps1
Creating output directory for fluentbit plugins
Starting ...
Fluent Bit v3.0.3
* Copyright (C) 2015-2024 The Fluent Bit Authors
* Fluent Bit is a CNCF sub-project under the umbrella of Fluentd
* https://fluentbit.io

___________.__                        __    __________.__  __          ________  
\_   _____/|  |  __ __   ____   _____/  |_  \______   \__|/  |_  ___  _\_____  \ 
 |    __)  |  | |  |  \_/ __ \ /    \   __\  |    |  _/  \   __\ \  \/ / _(__  < 
 |     \   |  |_|  |  /\  ___/|   |  \  |    |    |   \  ||  |    \   / /       \
 \___  /   |____/____/  \___  >___|  /__|    |______  /__||__|     \_/ /______  /
     \/                     \/     \/               \/                        \/ 

[2024/04/28 17:56:57] [error] [/tmp/fluent-bit-20240427-5610-m93cau/fluent-bit-3.0.3/src/config_format/flb_cf_fluentbit.c:289 errno=2] No such file or directory
[2024/04/28 17:56:57] [error] file=/Users/t-takeuchi/Work/OpenSource/Demo/Logging/FluentBit/03_HealthCheckToFile/parsers.conf
[2024/04/28 17:56:57] [error] [/tmp/fluent-bit-20240427-5610-m93cau/fluent-bit-3.0.3/src/config_format/flb_cf_fluentbit.c:289 errno=2] No such file or directory
[2024/04/28 17:56:57] [error] file=/Users/t-takeuchi/Work/OpenSource/Demo/Logging/FluentBit/03_HealthCheckToFile/plugins.conf
[2024/04/28 17:56:57] [ info] [fluent bit] version=3.0.3, commit=, pid=69755
[2024/04/28 17:56:57] [ info] [storage] ver=1.5.2, type=memory, sync=normal, checksum=off, max_chunks_up=128
[2024/04/28 17:56:57] [ info] [cmetrics] version=0.9.0
[2024/04/28 17:56:57] [ info] [ctraces ] version=0.5.1
[2024/04/28 17:56:57] [ info] [input:health:health.0] initializing
[2024/04/28 17:56:57] [ info] [input:health:health.0] storage_strategy='memory' (memory only)
[2024/04/28 17:56:57] [ info] [sp] stream processor started
[2024/04/28 17:56:57] [ info] [output:file:file.0] worker #0 started
````

When http service is running, fluent-bit does not show error log.
But it show error log after shutdown http service.

````shell
[2024/04/28 17:57:11] [error] [net] TCP connection failed: 127.0.0.1:5001 (Connection refused)
[2024/04/28 17:57:12] [error] [net] TCP connection failed: 127.0.0.1:5001 (Connection refused)
[2024/04/28 17:57:13] [error] [net] TCP connection failed: 127.0.0.1:5001 (Connection refused)
````

Outout log file is [health.0](./logs/health.0)].

````txt
health.0: [1714294626.943048000, {"alive":true}]
health.0: [1714294627.942897000, {"alive":true}]
health.0: [1714294628.946010000, {"alive":true}]
health.0: [1714294629.950573000, {"alive":true}]
health.0: [1714294630.945229000, {"alive":true}]
health.0: [1714294631.941151000, {"alive":false}]
health.0: [1714294632.939160000, {"alive":false}]
````