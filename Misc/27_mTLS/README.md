# Mutual TLS

## Abstracts

* How to use Mutual TLS (mTLS) for nginx

## Requirements

### Common

* Powershell 7 or later
* .NET 8.0 or later

## How to run?

### Windows

````bash
$ cd sources\Demo
$ dotnet run -c Release -- https://192.168.11.100 client.pfx password
````

### Linux or MacOS

````bash
$ cd sources/Demo
$ dotnet run -c Release -- https://192.168.11.100 client.crt client.key
````