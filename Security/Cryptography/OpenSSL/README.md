# OpenSSL

## Abstracts

* Build OpenSSL
* Sample and experimental codes

## Requirements

### Common

* Powershell 7 or later
* C++ Compiler

### Windows

* Visual Studio 2022
* [Strawberry Perl](https://strawberryperl.com/)
  * GNU General Public License or the Artistic License
* [NASM](https://www.nasm.us/)
  * BSD-2-Clause License

### Linux

* g++

### OSX

* Xcode

### Android

* Android SDK and NDK

### iOS

* Xcode

## Dependencies

* [OpenSSL](https://www.openssl.org/)
  * 3.6.2
    * 3.0 or later: Apache License 2.0
    * 1.x and earlier: OpenSSL License

## How to use?

At first, you have to download openssl source code.

#### Windows

````shell
$ set PERLPATH=D:\Works\Lib\Strawberry Perl\5.26.2.1\perl\bin
$ set NASMPATH=D:\Works\Lib\NASM\3.01\x64
$ pwsh build.ps1 <Debug/Release> <x86_64/x86/arm64>
````

#### Linux

````shell
$ pwsh build.ps1 <Debug/Release> <x86_64/x86/arm64>
````

#### OSX

````shell
$ pwsh build.ps1 <Debug/Release> <x86_64/arm64>
````

#### Android

````bash
$ export ANDROID_HOME=~/Library/Android/sdk
$ export ANDROID_NDK_HOME=${ANDROID_HOME}/ndk/28.2.13676358
$ pwsh build.ps1 <Debug/Release> <x86/x86_64/arm32/arm64> android
````

#### iOS

````bash
$ pwsh build.ps1 <Debug/Release> <x86_64/arm64> <ios/ios-simulator>
````
