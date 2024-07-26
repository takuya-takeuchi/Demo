# Use Native C++ xcframework library (dynamic link)

## Abstracts

* How to use C++ native library in xcframework from nuget
  * Pack xcframework into nuget package
  * iOS 8 or later, Apple allow using dynamic framework

## Requirements

### Common

* .NET 8.0
* Powershell 7 or later

### OSX

* Ninja
* Mono
* CMake
* Andorid SDK

## Dependencies

* [Prism.Maui](https://github.com/PrismLibrary/Prism.Maui)
  * 9.0.401-pre
  * MIT License
* [Prism.DryIoc.Maui](https://github.com/PrismLibrary/Prism.Maui)
  * 9.0.401-pre
  * MIT License
* [Prism.Maui.Rx](https://github.com/PrismLibrary/Prism.Maui)
  * 9.0.401-pre
  * MIT License

### Caution

All Prism.Maui libraries are pre-release so we should not adopt them on production!!

### How to build?

````shel
$ export ANDROID_HOME=/Users/user/Library/Android/sdk
$ export ANDROID_NDK_ROOT=/Users/user/Library/Android/sdk/ndk/26.1.10909125
$ pwsh Build.ps1
$ pwsh CreatePackage.ps1
````

Then, you must source directory to manager built nuget package from nuget command.

````shell
$ dotnet nuget add source $PWD --name private-source
Package source with Name: private-source added successfully.
$ dotnet nuget list source                          
Registered Sources:
  1.  nuget.org [Enabled]
      https://api.nuget.org/v3/index.json
  2.  private-source [Enabled]
      /Users/t-takeuchi/Work/OpenSource/Demo/MAUI/04_NativeLibrary
$ cd sources/Demo
$ dotnet add package Native.Maui
````

### How to run?

##### Android

````shell
$ dotnet build -t:Run -f net8.0-android sources/Demo/Demo.csproj 
````

##### iOS

````shell
$ dotnet build -t:Run -f net8.0-ios sources/Demo/Demo.csproj 
````

You can specify simulator or device.

````shell
$ /Applications/Xcode.app/Contents/Developer/usr/bin/simctl list
== Device Types ==
iPhone 6s (com.apple.CoreSimulator.SimDeviceType.iPhone-6s)
iPhone 6s Plus (com.apple.CoreSimulator.SimDeviceType.iPhone-6s-Plus)
iPhone SE (1st generation) (com.apple.CoreSimulator.SimDeviceType.iPhone-SE)
iPhone 7 (com.apple.CoreSimulator.SimDeviceType.iPhone-7)
iPhone 7 Plus (com.apple.CoreSimulator.SimDeviceType.iPhone-7-Plus)
iPhone 8 (com.apple.CoreSimulator.SimDeviceType.iPhone-8)
iPhone 8 Plus (com.apple.CoreSimulator.SimDeviceType.iPhone-8-Plus)
...
Apple Watch Series 9 (41mm) (com.apple.CoreSimulator.SimDeviceType.Apple-Watch-Series-9-41mm)
Apple Watch Series 9 (45mm) (com.apple.CoreSimulator.SimDeviceType.Apple-Watch-Series-9-45mm)
Apple Watch Ultra 2 (49mm) (com.apple.CoreSimulator.SimDeviceType.Apple-Watch-Ultra-2-49mm)
== Runtimes ==
iOS 17.0 (17.0.1 - 21A342) - com.apple.CoreSimulator.SimRuntime.iOS-17-0
iOS 17.2 (17.2 - 21C62) - com.apple.CoreSimulator.SimRuntime.iOS-17-2
== Devices ==
-- iOS 17.0 --
    iPhone SE (3rd generation) (8AE1A80C-C0AE-46AE-AAAE-43D2DB9FF13D) (Shutdown) 
    iPhone 15 (8F07870E-9B2D-4B40-9308-F455883A5326) (Shutdown) 
    iPhone 15 Plus (88BF7E0D-2EB8-49FB-979D-AAB05C6D68C1) (Shutdown) 
    iPhone 15 Pro (00790C07-4ECA-48BF-AAB0-8780AB7C39DE) (Shutdown) 
    iPhone 15 Pro Max (EA2DAA2F-48D3-406F-9BEE-76390AA49469) (Shutdown) 
    iPad Air (5th generation) (8E989554-2B9D-4CB8-A84A-AE35A3C07372) (Shutdown) 
    iPad (10th generation) (6AA5D262-0AF0-41D7-8CC2-07216F3C3289) (Shutdown) 
    iPad mini (6th generation) (7449811B-5EC9-4965-BAFF-0A48AA63818C) (Shutdown) 
    iPad Pro (11-inch) (4th generation) (7287A048-6230-4863-9A0B-E4314FCA88BD) (Shutdown) 
    iPad Pro (12.9-inch) (6th generation) (18442026-5B4B-4C06-858F-7EA249A9D90D) (Shutdown) 
-- iOS 17.2 --
    iPhone SE (3rd generation) (A7586751-0EF8-4418-9B97-CFE1ABC0EC3E) (Shutdown) 
    iPhone 15 (CD81E069-5FD4-479D-9718-702E64DE065E) (Shutdown) 
    iPhone 15 Plus (FF56F9BB-8099-4D26-8766-31CCA13634BF) (Shutdown) 
    iPhone 15 Pro (44BCCE5A-8084-45B9-9A0F-9D0C2217A87F) (Shutdown) 
    iPhone 15 Pro Max (527A48CE-B94E-4E4C-B114-9226899FB4F6) (Shutdown) 
    iPad Air (5th generation) (9B810695-DDED-4D3A-A775-983404EE840B) (Shutdown) 
    iPad (10th generation) (C41317A9-02C7-495C-AC31-7682EDAB903B) (Booted) 
    iPad mini (6th generation) (2A5A60FA-7736-497B-A54B-D766D0081B9C) (Shutdown) 
    iPad Pro (11-inch) (4th generation) (06A95827-12E6-4677-9BC0-15C3258D0D2A) (Shutdown) 
    iPad Pro (12.9-inch) (6th generation) (C1578380-8A04-4C8F-BA60-FBC76D81E9D4) (Shutdown) 
    iPad Pro (12.9-inch) (6th generation) (16GB) - iOS 17.2 (829E0BF4-4B3B-4246-8C54-9885231F2457) (Booted) 
== Device Pairs ==

$ dotnet build -t:Run -f net8.0-ios sources/Demo/Demo.csproj -p:_DeviceName=:v2:udid=C41317A9-02C7-495C-AC31-7682EDAB903B
````

## Result

|iOS|Android|
|---|---|
|<img src="images/ios.gif?raw=true" title="ios" width="480" />|<img src="images/android.gif?raw=true" title="android" width="480" />|
