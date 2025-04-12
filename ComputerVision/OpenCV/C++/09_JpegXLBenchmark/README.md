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

````bat
$ ./install/linux/osx/Demo
Unable to determine clock rate from sysctl: hw.cpufrequency: No such file or directory
This does not affect benchmark measurements, only the metadata output.
***WARNING*** Failed to set thread affinity. Estimated CPU frequency may be incorrect.
2025-04-13T00:21:42+09:00
Running ./install/osx/bin/Demo
Run on (8 X 24 MHz CPU s)
CPU Caches:
  L1 Data 64 KiB
  L1 Instruction 128 KiB
  L2 Unified 4096 KiB (x8)
Load Average: 3.39, 3.17, 2.86
***WARNING*** Library was built as DEBUG. Timings may be affected.
----------------------------------------------------------
Benchmark                Time             CPU   Iterations
----------------------------------------------------------
BM_EncodeToJxl        35.7 ms         9.99 ms           71
BM_EncodeToJpeg      0.646 ms        0.646 ms         1072
BM_DecodeJxl          5.67 ms        0.973 ms          725
BM_DecodeJpeg         2.38 ms         2.38 ms          294
````