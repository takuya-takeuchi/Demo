# Benchmark of JPEG XL

## Abstracts

* Benchmark `cv::imencode` and `cv::imdecode`

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [libyuv](https://chromium.googlesource.com/libyuv/libyuv)
  * main (deeb764bb92b6305ee0cb5dae7a5940fdb457fa9, Wed Dec 10 03:46:41 2025 +0000)
  * BSD 3-Clause License

## How to build?

### libyuv

Go to [libyuv](..).

````shell
$ pwsh build.ps1 <Debug/Release>
````

Once time you built `libyuv`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ ./install/linux/win/Demo
libyuv cpu: AVX2=8192 FMA3=65536
````