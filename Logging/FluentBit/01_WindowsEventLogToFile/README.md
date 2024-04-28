# Input Windows Event Log and Output as File

## Abstracts

* Input Windows Event Log and output it as File
  * Use input `winevtlog` and output `file` plugins

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
|Name|winevtlog|
|Channels|System|
|Interval_Sec|10|
|DB|C:\fluentbit\winevtlog\winevtlog.sqlite|
|Use_ANSI|False|True|

##### Important

You can use `\` as path delimiter for `DB` on Windows.

#### OUTPUT

|Key|Value|
|---|---|
|Name|file|
|Path|C:\fluentbit\winevtlog|
|Mkdir|True|
|Workers|1|

##### Important

You can use `\` as path delimiter for `Path` on Windows.

## How to run?

Af first, you must download or install fluentbit.
Please refer [README.md](../00_GetStarted/README.md).

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

[2024/04/28 04:11:49] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 04:11:49] [error] [D:\a\fluent-bit\fluent-bit\src\config_format\flb_cf_fluentbit.c:458 errno=2] No such file or directory
[2024/04/28 04:11:49] [ info] [fluent bit] version=3.0.2, commit=33ce918351, pid=23836
[2024/04/28 04:11:49] [ info] [storage] ver=1.5.2, type=memory, sync=normal, checksum=off, max_chunks_up=128
[2024/04/28 04:11:49] [ info] [cmetrics] version=0.7.3
[2024/04/28 04:11:49] [ info] [ctraces ] version=0.4.0
[2024/04/28 04:11:49] [ info] [input:winevtlog:winevtlog.0] initializing
[2024/04/28 04:11:49] [ info] [input:winevtlog:winevtlog.0] storage_strategy='memory' (memory only)
[2024/04/28 04:11:49] [ info] [sp] stream processor started
[2024/04/28 04:11:49] [ info] [output:file:file.0] worker #0 started
````

### Output samples

For example, `winevtlog.0` will be created in `C:\fluentbit\winevtlog`.

````txt
winevtlog.0: [1714245129.705932500, {"ProviderName":"Microsoft-Windows-Eventlog","ProviderGuid":"{FC65DDD8-D6EF-4962-83D5-6E5CFE9CE148}","Qualifiers":"","EventID":104,"Version":1,"Level":4,"Task":104,"Opcode":0,"Keywords":"0x8000000000000000","TimeCreated":"2024-04-28 04:12:00 +0900","EventRecordID":123500,"ActivityID":"","RelatedActivityID":"","ProcessID":1236,"ThreadID":18020,"Channel":"System","Computer":"TAKUYA-PC2","UserID":"S-1-5-21-1366212534-334562847-3460492818-1001","Message":"Application ログ ファイルが消去されました。"}]
````