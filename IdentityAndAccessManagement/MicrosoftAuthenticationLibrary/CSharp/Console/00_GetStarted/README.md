# Simple Multi-Factor Authentication

## Abstracts

* Use Multi-Factor Authentication by MSAL (Microsoft Authentication Library)

## Requirements

* .NET 8.0 or later

## Dependencies

* [Microsoft.Extensions.Configuration](https://dotnet.microsoft.com)
  * MIT License
  * 9.0.8
* [Microsoft.Extensions.Configuration.Binder](https://dotnet.microsoft.com)
  * MIT License
  * 9.0.8
* [Microsoft.Extensions.Configuration.EnvironmentVariables](https://dotnet.microsoft.com)
  * MIT License
  * 9.0.8
* [Microsoft.Extensions.Configuration.FileExtensions](https://dotnet.microsoft.com)
  * MIT License
  * 9.0.8
* [Microsoft.Extensions.Configuration.Json](https://dotnet.microsoft.com)
  * MIT License
  * 9.0.8
* [Microsoft.Identity.Client](https://github.com/AzureAD/microsoft-authentication-library-for-dotnet)
  * MIT License
  * 4.74.1
* [Microsoft.Identity.Client.Extensions.Msal](https://github.com/AzureAD/microsoft-authentication-library-for-dotnet)
  * MIT License
  * 4.74.1
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
  * 6.0.3

## Preparations

Register a new application in the Microsoft Azure portal. Once the application is registered, note the Application (client) ID and the Directory (tenant) ID from the Overview menu.
Please refer [Register an application in Microsoft Entra ID](https://learn.microsoft.com/en-us/entra/identity-platform/quickstart-register-app)

If you see the following error message after authentication, you need to check whether redirect uri is valid.

> AADSTS500113: No reply address is registered for the application.

And redirect uri SHALL be `http://localhost`.

## How to run?

````bat
$ cd sources\Demo
$ dotnet run -c Release
````

If valid authentication token is remaining, you can skip authentication process.

````bat
$ cd sources\Demo
$ dotnet run -c Release
2025-08-12 09:32:48.3694 [INFO ] AccessToken or RefreshToken is valid
2025-08-12 09:32:48.4010 [INFO ] Authentication is complete
2025-08-12 09:32:48.4010 [INFO ] Token valid for 59.52663904166667 more minutes.
````

If you want to clear token cache, remove `msal.cache` in build directory.