# Enumerate SmartCard Readers

## Abstracts

* Enumerate SmartCard Readers connected to machine by using `PCSC`

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
2024-04-06 18:21:50.5982 [INFO ] Currently connected readers:  
2024-04-06 18:21:50.6361 [INFO ]        SONY FeliCa Port/PaSoRi 4.0 0 
2024-04-06 18:21:50.6501 [INFO ] Currently configured readers groups:  
2024-04-06 18:21:50.6501 [INFO ]        SCard$DefaultReaders
2024-04-06 18:21:50.6501 [INFO ] Group SCard$DefaultReaders contains
2024-04-06 18:21:50.6557 [INFO ]        SONY FeliCa Port/PaSoRi 4.0 0
2024-04-06 18:21:50.6557 [INFO ] Press any key to exit.
````

#### No devices

````bat
$ dotnet run --project .\sources\Demo -c Release
2024-04-06 18:23:01.0103 [INFO ] Currently connected readers:  
2024-04-06 18:23:01.0480 [INFO ] Currently configured readers groups:  
2024-04-06 18:23:01.0480 [INFO ]        SCard$DefaultReaders
2024-04-06 18:23:01.0480 [INFO ] Group SCard$DefaultReaders contains
2024-04-06 18:23:01.0480 [INFO ] Press any key to exit.
````