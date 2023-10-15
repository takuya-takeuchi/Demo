# Logging with daily archive (also current log file with date)

## Abstracts

* How to control condtion of daily logging

## Requirements

* .NET Framework 4.6.1

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
  * 4.0.0
    * 3.2.1 or older version does not work.

## How to usage?

Build and run from Visul Studio.

## Note

[sources/Demo/NLog.config](./sources/Demo/NLog.config) is configured according to

* current log file name is `Application.<today:yyyyMMdd>.log`
* archive log file name is `Application.<yyyyMMdd>.log`

NLog uses LastWriteTime of file to archive files. Refer to [GetArchiveDate](https://github.com/NLog/NLog/blob/d2b872449bc29326456000a5393b1ef050de48d7/src/NLog/Targets/FileTarget.cs#L1473).
For examples, today is 2023/10/15 and if LastWriteTime of `Application.log` is 2023/10/15 or `Application.log` is missing, achive will not be executed.

Logging files are the following. 

````
Application.20231007.log (LastWriteTime: 2023/10/07)
Application.20231008.log (LastWriteTime: 2023/10/08)
Application.20231009.log (LastWriteTime: 2023/10/09)
Application.20231010.log (LastWriteTime: 2023/10/10)
Application.20231011.log (LastWriteTime: 2023/10/11)
Application.20231012.log (LastWriteTime: 2023/10/12)
Application.20231013.log (LastWriteTime: 2023/10/13)
Application.20231014.log (LastWriteTime: 2023/10/14)
````

After do archive, `Application.20231007.log` is removed and `Application.20231015.log` is generated.

You can create test log files by using [CreateTestLogFiles.ps1](./CreateTestLogFiles.ps1).