# Xamarin.Native.Nuget

* Use native library as nuget

## Requirements

* Visual Studio 2019 16.5.4 or later
* CMake 3.15 or later
* PowerShell Core 6.2 or later

## Dependencies

* [ios-cmake](https://github.com/leetal/ios-cmake) 4.3.0
  * BSD-3-Clause License

## How to build

````shell
$ git submodule update --init --recursive
$ pwsh BuildIOS.ps1
````

[Install NuGet client tools](https://docs.microsoft.com/ja-jp/nuget/install-nuget-client-tools#macoslinux)

````shell
$ sudo curl -o /usr/local/bin/nuget.exe https://dist.nuget.org/win-x86-commandline/latest/nuget.exe
$ alias nuget="mono /usr/local/bin/nuget.exe"
````