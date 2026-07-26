# Thundering Herd

## Abstracts

* Demonstration of 'Thundering Herd' problem in case of JWT token refresh
  * When many clients uses a cerain one user account, then try to refresh an expired or expiring access token at the exact same time.

## Requirements

* .NET 10.0

## Dependencies

* [Microsoft.Extensions.ApiDescription.Client](https://github.com/dotnet/dotnet)
  * 7.0.2
  * MIT License
* [Microsoft.Extensions.Configuration.Binder](https://github.com/dotnet/dotnet)
  * 10.0.10
  * MIT License
* [Microsoft.Extensions.Hosting](https://github.com/dotnet/dotnet)
  * 10.0.10
  * MIT License
* [Microsoft.Extensions.Http](https://github.com/dotnet/dotnet)
  * 10.0.10
  * MIT License
* [Newtonsoft.Json](https://github.com/JamesNK/Newtonsoft.Json)
  * 13.0.1
  * MIT License
* [NSwag.ApiDescription.Client](https://github.com/RicoSuter/NSwag)
  * 13.18.2
  * MIT License
* [StackExchange.Redis](https://github.com/StackExchange/StackExchange.Redis/)
  * 3.0.17
  * MIT License
* [System.CommandLine](https://github.com/dotnet/dotnet)
  * 2.0.10
  * MIT License

## How to run?

This demo provides the following modes.

1. SharedLeader mode
2. Independent mode

And this demo depends on [ASP.NET\13_JWTAuthentication](../../ASP.NET/13_JWTAuthentication).
At first, modify `JWT.AccessTokenLifetimeSeconds` to `10` in [ASP.NET/13_JWTAuthentication/appsettings.json](../../ASP.NET/13_JWTAuthentication/appsettings.json).
Then launch this server process.

#### SharedLeader

All processes share one token via Redis in-memory cache.
Token is refreshed by 'Leader' process.

At first, launch Redis.

````bash
$ docker run --rm --name thundering-herd-redis -p 6379:6379 -d redis:latest
````

And launch 10 client processes.

````bat
$ pwsh run-sharedLeader.ps1
````

You will see that only one process invokes refresh api.

#### Independent

Each process performs its own token refresh.
Launch 10 client processes.

````bat
$ pwsh run-independent.ps1
````

You will see that each processes invoke refresh api.
