# WCF Console sample (Client and Server)

## Abstracts

* .NET Framework's client and server apps
  * No use `*.exe.config` to config server

## Requirements

* Visual Studio 2019 or later
* .NET Framework 4.8 SDK

## Dependencies

* [NLog](https://github.com/NLog/NLog)
  * 5.0.0
  * BSD-3-Clause License

## How to try?

Build by `Visual Studio`.
Then, launch `Server.exe` with privilege at first.
Or execute `netsh http add urlacl url=http://+:5001/ user=everyone` with privilege.

````bat
$ cd sources\Server\bin\Debug
$ Server.exe http://localhost:5001
2024-04-27 21:15:38.0161 [INFO ] Start on http://localhost:5001 
2024-04-27 21:15:38.5078 [INFO ] The service is ready at 0 
2024-04-27 21:15:38.5128 [INFO ] Press <Enter> to stop the service.
````

You can also see service description by browser.

<img src="./images/service.png" />

At last, launch `Client.exe`.

````bat
$ cd sources\Client\bin\Debug
$ Client.exe http://localhost:5001
2024-04-28 00:17:17.1230 [INFO ] Start 
2024-04-28 00:17:17.1470 [INFO ] Connect to http://localhost:5001 
2024-04-28 00:17:19.6776 [INFO ] Server says 'Hello, client'
````