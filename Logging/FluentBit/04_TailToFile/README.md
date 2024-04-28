# Input application log files and Output as File

## Abstracts

* Input application log files and output it as File
  * Use input `tail` and output `file` plugins
  * Application log files are normal text logs. `tail` plugin observe them and fluent-bit forward them to output plugin
  * This approach should not modify application side

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
|name|tail|
|tag|tail.app_logs.*|
|path|${PROJECT_ROOT}\http-service\Logs\access*.log|
|path_key|application_log_file_path|
|read_from_head|true|
|refresh_interval|30|

##### Important

You SHALL use `\` as path delimiter for `path` on Windows.

#### OUTPUT

|Key|Value|
|---|---|
|Name|file|
|Path|${PROJECT_ROOT}\logs|
|Mkdir|True|
|Workers|1|

##### Important

You SHALL use `\` as path delimiter for `path` on Windows.

## How to run?

Af first, you must download or install fluentbit.
Please refer [README.md](../00_GetStarted/README.md).

Then, launch sample http application.

````shell
$ cd http-service
$ dotnet run -c Release --urls http://localhost:5001
ビルドしています...
2024-04-28T19:42:40.1115198+09:00 [INF] [] [] Starting web application
````

Lastly, kick `Run.ps1`

````bat
$ pwsh Run.ps1
Creating output directory for fluentbit plugins
Starting fluentbit...
Fluent Bit v3.0.2
* Copyright (C) 2015-2024 The Fluent Bit Authors
* Fluent Bit is a CNCF sub-project under the umbrella of Fluentd
* https://fluentbit.io

___________.__                        __    __________.__  __          ________
\_   _____/|  |  __ __   ____   _____/  |_  \______   \__|/  |_  ___  _\_____  \
 |    __)  |  | |  |  \_/ __ \ /    \   __\  |    |  _/  \   __\ \  \/ / _(__  <
 |     \   |  |_|  |  /\  ___/|   |  \  |    |    |   \  ||  |    \   / /       \
 \___  /   |____/____/  \___  >___|  /__|    |______  /__||__|     \_/ /______  /
     \/                     \/     \/               \/                        \/

[2024/04/28 19:43:58] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 19:43:58] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 19:43:58] [ info] [fluent bit] version=3.0.2, commit=33ce918351, pid=19804
[2024/04/28 19:43:58] [ info] [storage] ver=1.5.2, type=memory, sync=normal, checksum=off, max_chunks_up=128
[2024/04/28 19:43:58] [ info] [cmetrics] version=0.7.3
[2024/04/28 19:43:58] [ info] [ctraces ] version=0.4.0
[2024/04/28 19:43:58] [ info] [input:tail:tail.0] initializing
[2024/04/28 19:43:58] [ info] [input:tail:tail.0] storage_strategy='memory' (memory only)
[2024/04/28 19:43:58] [ info] [sp] stream processor started
[2024/04/28 19:43:58] [ info] [output:file:file.0] worker #0 started
````

### Output samples

For example, `access20240428_001.log` will be created in [logs](./logs).

````txt
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365247900, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.3180458+09:00 [INF] [0HN37E0N8J1HV:00000008] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/index.html\" responded 200 in 0.2911 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365256300, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.3579885+09:00 [INF] [0HN37E0N8J1HV:00000009] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/swagger/v1/swagger.json\" responded 200 in 0.1534 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365257000, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.5182874+09:00 [INF] [0HN37E0N8J1HV:0000000A] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/index.html\" responded 200 in 0.2726 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365257500, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.5591774+09:00 [INF] [0HN37E0N8J1HV:0000000B] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/swagger/v1/swagger.json\" responded 200 in 0.1257 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365258100, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.7027801+09:00 [INF] [0HN37E0N8J1HV:0000000C] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/index.html\" responded 200 in 0.2584 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365258700, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.7455219+09:00 [INF] [0HN37E0N8J1HV:0000000D] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/swagger/v1/swagger.json\" responded 200 in 0.1302 ms"}]
tail.app_logs.E.Works.OpenSource.Demo2.Logging.FluentBit.04_TailToFile.http-service.Logs.access20240428_001.log: [1714301098.365259200, {"application_log_file_path":"E:\\Works\\OpenSource\\Demo2\\Logging\\FluentBit\\04_TailToFile\\http-service\\Logs\\access20240428_001.log","log":"2024-04-28T19:44:31.8379317+09:00 [INF] [0HN37E0N8J1HV:0000000E] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP \"GET\" \"/index.html\" responded 200 in 0.2224 ms"}]
````

Whenever access to `http://localhost:5001`, access log will be forwarded.