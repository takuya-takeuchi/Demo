# Check Credit Card Number

## Abstracts

* Check Credit Card Number by Luhn Algorithm
  * Luhn Algorithm is implemented as native library
* Link native library into MAUI
* Test UI by Maestro

:warning: Maestro can test Android Emulator, real device and iOS Simulator. For now, Maestro does not support iOS real device

## Requirements

* Powershell 7 or later
* CMake 3.5.0 or later
* Android Stduio or Android SDK/NDK

## Windows

* Visual Studio 2022

## Linux

* GCC

## OSX

* XCode

## Dependencies

* [Luhn Algorithm](https://github.com/karancodes/credit-card-validator)
  * a1e9921e26a6ecf725c600a2eccd13ff78f6f6e9
  * MIT License
* [Maestro](https://github.com/mobile-dev-inc/maestro)
  * 1.31.0
  * Apache-2.0 License

## How to usage?

````sh
$ ./All.sh
````

Then, launch Demo.sln and start program as iOS or Android.
Next,

````sh
$ maestro test <flow-android.yaml/flow-ios.yaml>
````

https://github.com/takuya-takeuchi/Demo/assets/6241854/df1011f2-f3a5-4d3d-838a-5997d4d1aef3
