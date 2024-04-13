# Generate C++ client code by OpenAPI

## Abstracts

* Generate C++ client code by using OpenAPI

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++14

### Windows

* Visual Studio
* Java
* vcpkg
* cpprestsdk
  * `vcpkg --triplet x64-windows-static install cpprestsdk`

### Ubuntu

* g++
* Java
* libcpprest-dev
  * `apt install libcpprest-dev`

### OSX

* Xcode
* Java
* brew
* cpprestsdk
  * `brew install cpprestsdk`
* openssl
  * `brew install openssl`

## Dependencies

* [cpprestsdk](https://github.com/microsoft/cpprestsdk)
  * 2.10.18
  * MIT License

## How to build?

At first, you must launch web api server.
This repositry uses [ASP.NET\01_Swagger](../../../ASP.NET\01_Swagger).

````bat
$ dotnet run -c Release --urls http://localhost:9000
Building...
info: Microsoft.Hosting.Lifetime[14]
      Now listening on: http://localhost:9000
info: Microsoft.Hosting.Lifetime[0]
      Application started. Press Ctrl+C to shut down.
info: Microsoft.Hosting.Lifetime[0]
      Hosting environment: Development
info: Microsoft.Hosting.Lifetime[0]
      Content root path: E:\Works\OpenSource\Demo2\ASP.NET\01_Swagger\
warn: Microsoft.AspNetCore.HttpsPolicy.HttpsRedirectionMiddleware[3]
      Failed to determine the https port for redirect.
````

Then, you can build.

````bat
$ pwsh generate.ps1 http://localhost:9000
$ pwsh build-openapi.ps1 Release
$ pwsh build.ps1 Release
````

## How to usage?

#### Windows

````bat
$ install\win\bin\Test.exe "http://localhost:9000"
[Info] message: Hello!!
[Info]    date: 2024-04-13T12:34:38.2261984Z
````

#### OSX

````bat
$ ./install/osx/bin/Test "http://localhost:9000"
[Info] message: Hello!!
[Info]    date: 2024-04-13T13:02:18.355794Z
````

#### Linux

````bat
$ ./install/linux/bin/Test "http://localhost:9000"
[Info] message: Hello!!
[Info]    date: 2024-04-13T13:52:37.9457844Z
````