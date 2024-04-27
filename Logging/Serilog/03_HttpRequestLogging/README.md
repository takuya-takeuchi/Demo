# Output HTTP access log

## Abstracts

* Output HTTP access and application logs separately

## Requirements

* .NET 6.0 SDK

## Dependencies

* [Serilog.AspNetCore](https://github.com/serilog/serilog-aspnetcore)
  * 8.0.1
  * Apache-2.0 license
* [Serilog.Expressions](https://github.com/serilog/serilog-expressions)
  * 4.0.0
  * Apache-2.0 license
* [Serilog.Settings.Configuration](https://github.com/serilog/serilog-settings-configuration)
  * 8.0.0
  * Apache-2.0 license
* [Swashbuckle.AspNetCore](https://github.com/domaindrivendev/Swashbuckle.AspNetCore)
  * 6.5.0
  * MIT License

## How to try?

````bat
$ dotnet run -c Release
ビルドしています...
2024-04-27T18:44:38.3582053+09:00 [INF] [] [] Starting web application
2024-04-27T18:45:50.8438475+09:00 [INF] [0HN36JR9CSOSG:00000001] [Demo.Controllers.GreetingController] Request Get, RequestId (HttpContext.TraceIdentifier): 0HN36JR9CSOSG:00000001
2024-04-27T18:45:50.9158702+09:00 [INF] [0HN36JR9CSOSG:00000001] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP "GET" "/api/greet" responded 200 in 100.7932 ms
````

##### access.log

````txt
2024-04-27T18:45:50.9158702+09:00 [INF] [0HN36JR9CSOSG:00000001] [Serilog.AspNetCore.RequestLoggingMiddleware] HTTP "GET" "/api/greet" responded 200 in 100.7932 ms
````

##### application.log

````txt
2024-04-27T18:44:38.3582053+09:00 [INF] [] [] Starting web application
2024-04-27T18:45:50.8438475+09:00 [INF] [0HN36JR9CSOSG:00000001] [Demo.Controllers.GreetingController] Request Get, RequestId (HttpContext.TraceIdentifier): 0HN36JR9CSOSG:00000001
````