# Get status of SmartCard

## Abstracts

* Retrieve status of SmartCard by using `PCSC`

## Requirements

### Windows

* .NET 8.0 SDK
* SmartCard Reader

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * 5.2.8
  * BSD-3-Clause License
* [PCSC](https://github.com/danm-de/pcsc-sharp)
  * 6.2.0
  * Custom License (But very similar with BSD-2-Clause License)

## How to build?

#### Devices found

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:28:30.3644 [INFO ] Reader names: SONY FeliCa Port/PaSoRi 4.0 0 
2024-04-06 18:28:30.3991 [INFO ] Protocol: T1 
2024-04-06 18:28:30.3991 [INFO ] State: Specific
2024-04-06 18:28:30.3991 [INFO ] ATR: 3B-88-80-01-00-4B-51-FF-00-81-D1-00-BC
2024-04-06 18:28:30.4043 [INFO ] Press any key to exit.
````

#### Devices found but card is empty

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:31:12.1991 [ERROR] The smart card has been removed, so further communication is not possible.
````

#### No devices

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:29:25.0777 [ERROR] No reader connected. 
````