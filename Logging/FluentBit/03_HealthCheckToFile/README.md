# Input Health Check Log and Output as File

## Abstracts

* Input Windows Event Log and output it as File
  * Use input `health` and output `file` plugins
    * `health` plugins is not enabled for Windows. Please refer [windows-setup.cmake](https://github.com/fluent/fluent-bit/blob/v3.0.2/cmake/windows-setup.cmake#L36).

## Requirements

* Powershell 7 or later

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
|port|80|
|interval_sec|1|
|interval_nsec|0|

##### Important

You can use `\` as path delimiter for `DB` on Windows.

#### OUTPUT

|Key|Value|
|---|---|
|Name|file|
|Path|C:\fluentbit\health|
|Mkdir|True|
|Workers|1|

##### Important

You can use `\` as path delimiter for `Path` on Windows.

## How to run?

Af first, you must download fluentbit package.
Please execute [windows/Download.ps1](../00_GetStarted/windows/Download.ps1).

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

[2024/04/28 16:39:19] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 16:39:19] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 16:39:19] [ info] [fluent bit] version=3.0.2, commit=33ce918351, pid=24032
[2024/04/28 16:39:19] [ info] [storage] ver=1.5.2, type=memory, sync=normal, checksum=off, max_chunks_up=128
[2024/04/28 16:39:19] [ info] [cmetrics] version=0.7.3
[2024/04/28 16:39:19] [ info] [ctraces ] version=0.4.0
[2024/04/28 16:39:19] [ info] [input:windows_exporter_metrics:windows_exporter_metrics.0] initializing
[2024/04/28 16:39:19] [ info] [input:windows_exporter_metrics:windows_exporter_metrics.0] storage_strategy='memory' (memory only)
[2024/04/28 16:39:19] [ info] [sp] stream processor started
[2024/04/28 16:39:19] [ info] [output:file:file.0] worker #0 started
````

### Output samples

For example, `windows_exporter_metrics.0` will be created in `C:\fluentbit\windows_exporter_metrics`.

````txt
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,0",state="c1"} = 397.72298169999999
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,0",state="c2"} = 53091.023496100002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,0",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,1",state="c1"} = 197.1708639
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,1",state="c2"} = 54750.680195399997
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,1",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,2",state="c1"} = 1568.5331524000001
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,2",state="c2"} = 49750.142568299998
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,2",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,3",state="c1"} = 265.43935199999999
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,3",state="c2"} = 54390.592284500002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,3",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,4",state="c1"} = 644.43338110000002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,4",state="c2"} = 52696.985993000002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,4",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,5",state="c1"} = 315.53428580000002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,5",state="c2"} = 54294.0685122
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,5",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,6",state="c1"} = 548.29093720000003
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,6",state="c2"} = 53046.921896699998
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,6",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,7",state="c1"} = 359.9918389
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,7",state="c2"} = 54251.192052400002
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,7",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,8",state="c1"} = 404.54995789999998
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,8",state="c2"} = 53383.478928800003
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,8",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,9",state="c1"} = 320.21822539999999
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,9",state="c2"} = 54363.311065200003
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,9",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,10",state="c1"} = 522.44706110000004
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,10",state="c2"} = 52894.482066500001
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,10",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,11",state="c1"} = 330.43421189999998
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,11",state="c2"} = 54424.403525399997
2024-04-28T07:39:24.322836300Z windows_cpu_cstate_seconds_total{core="0,11",state="c3"} = 0
2024-04-28T07:39:24.322836300Z windows_cpu_time_total{core="0,0",mode="idle"} = 54457.4375
2024-04-28T07:39:24.322836300Z windows_cpu_time_total{core="0,0",mode="interrupt"} = 505.125
2024-04-28T07:39:24.322836300Z windows_cpu_time_total{core="0,0",mode="dpc"} = 270.8125
2024-04-28T07:39:24.322836300Z windows_cpu_time_total{core="0,0",mode="privileged"} = 1694.546875
````