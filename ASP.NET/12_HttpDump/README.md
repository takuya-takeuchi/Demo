# Dump http request/response

## Abstracts

* How to dump http request and response

## Requirements

* .NET 10.0 SDK

## Dependencies

* [Microsoft.AspNetCore.OpenApi](https://github.com/dotnet/dotnet)
  * MIT license
* [Swashbuckle.AspNetCore](https://github.com/domaindrivendev/Swashbuckle.AspNetCore)
  * MIT license

## How to use?

At first, launch app.

````bash
$ dotnet run -c Release
````

Next, kick some API.

````bash
$ curl -X "GET" "http://localhost:5234/api/Test" -H "accept: application/json"
````

Then, app console outputs http request and response.

````bash
info: Microsoft.Hosting.Lifetime[14]
      Now listening on: http://localhost:5234
info: Microsoft.Hosting.Lifetime[0]
      Application started. Press Ctrl+C to shut down.
info: Microsoft.Hosting.Lifetime[0]
      Hosting environment: Development
info: Microsoft.Hosting.Lifetime[0]
      Content root path: E:\Works\OpenSource\Demo\ASP.NET\11_HttpDump
warn: Microsoft.AspNetCore.HttpsPolicy.HttpsRedirectionMiddleware[3]
      Failed to determine the https port for redirect.
info: Demo.HttpDumpMiddleware[0]
      HTTP DUMP
      ===== REQUEST =====
      GET /api/Test HTTP/1.1
      Accept: application/json
      Host: localhost:5234
      User-Agent: curl/8.13.0
      
      
      ===== RESPONSE =====
      HTTP/1.1 200
      
      
      Duration: 14.0706 ms
      Exception: (none)
````