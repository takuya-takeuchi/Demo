# Output RequestID

## Abstracts

* Output `RequestId` assigned string value when receive request
* Get `RequestId` by `Microsoft.AspNetCore.Http.HttpContext.TraceIdentifier`
  * Serilog and controller method can refer same value

## Requirements

* .NET 6.0 SDK

## Dependencies

* [Serilog.AspNetCore](https://github.com/serilog/serilog-aspnetcore)
  * 8.0.1
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
2024-04-19T23:30:51.8357352+09:00 [INF] [] Starting web application
````

And then, you can see `RequestId` when invoke `GET /api/greet`.

````bat
2024-04-19T23:34:12.7308954+09:00 [INF] [] Starting web application
2024-04-19T23:34:18.2897135+09:00 [INF] [0HN30FN5M1CKK:00000001] Request Get, RequestId (HttpContext.TraceIdentifier): 0HN30FN5M1CKK:00000001
2024-04-19T23:34:19.0080005+09:00 [INF] [0HN30FN5M1CKK:00000003] Request Get, RequestId (HttpContext.TraceIdentifier): 0HN30FN5M1CKK:00000003
2024-04-19T23:34:20.8374709+09:00 [INF] [0HN30FN5M1CKK:00000005] Request Get, RequestId (HttpContext.TraceIdentifier): 0HN30FN5M1CKK:00000005
````