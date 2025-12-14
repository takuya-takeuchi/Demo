# Get Started

## Abstracts

* Check cpuid by libyuv

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

* [opencv](https://github.com/opencv/opencv)
  * 4.11.0
  * Apache-2.0 License

## How to build?

### OpenCV 4, libjxl and benchmark

Go to [OpenCV](..).

````shell
$ pwsh build-libjxl.ps1 <Debug/Release>
$ pwsh build-benchmark.ps1 <Debug/Release>
$ pwsh build-enable-jpegxl.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ ./install/linux/win/Demo
libyuv cpu: AVX2=8192 FMA3=65536
````