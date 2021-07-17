# Xamarin.Native.Nuget2

* Use native library as nuget
* Link libc++ instead of libstdc++

## Requirements

* Visual Studio 2019 16.5.4 or later
* CMake 3.15 or later
* PowerShell Core 6.2 or later
* Xcode 11.6 or higher

## Dependencies

* [ios-cmake](https://github.com/leetal/ios-cmake) 4.3.0
  * BSD-3-Clause License

## How to build

````shell
$ git submodule update --init --recursive
$ pwsh BuildIOS.ps1
````

## How to create nuget package and publish to server

````shell
$ pwsh CreatePackage.ps1 https://localserver:5000 apikey
````

## How to provisioning to device/simulator

Open [Xcode.Xamarin.Native.Nuget/Native/Native.xcodeproj](Xcode.Xamarin.Native.Nuget/Native/Native.xcodeproj) in OSX.  
Then build and deploy it to device.

## How to use nuget package

Add **Native.Xamarin** or **Native.Xamarin.Static** to [Xamarin.Native.Nuget/Xamarin.Native.Nuget.iOS](Xamarin.Native.Nuget/Xamarin.Native.Nuget.iOS)