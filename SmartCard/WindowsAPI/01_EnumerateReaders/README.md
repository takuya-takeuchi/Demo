# Enumerate SmartCard Readers

## Abstracts

* Enumerate SmartCard Readers connected to machine by using `Windows API`

## Requirements

### Windows

* Windows 10.0.19041.0 or later
* .NET 8.0 SDK
* SmartCard Reader

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * 5.2.8
  * BSD-3-Clause License

## How to build?

#### Devices found

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:51:20.7197 [INFO ] Connected device: SONY FeliCa Port/PaSoRi 4.0 0 
2024-04-06 18:51:20.7543 [INFO ] Press any key to exit. 
````

#### No devices

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:51:08.3615 [INFO ] Press any key to exit.
````