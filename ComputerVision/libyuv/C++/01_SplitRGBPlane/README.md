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

* [benchmark](https://github.com/google/benchmark)
  * v1.9.4
  * Apache-2.0 License
* [libyuv](https://chromium.googlesource.com/libyuv/libyuv)
  * main (deeb764bb92b6305ee0cb5dae7a5940fdb457fa9, Wed Dec 10 03:46:41 2025 +0000)
  * BSD 3-Clause License

## How to build?

### libyuv

### libyuv and benchmark

Go to [libyuv](..).

````shell
$ pwsh build.ps1 <Debug/Release>
$ pwsh build-benchmark.ps1 <Debug/Release>
````

Once time you built `opencv4`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ ./install/linux/win/Demo
2025-12-14T10:48:50+09:00
Running install\win\bin\Demo.exe
Run on (12 X 3192 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x6)
  L1 Instruction 32 KiB (x6)
  L2 Unified 256 KiB (x6)
  L3 Unified 12288 KiB (x1)
-------------------------------------------------------------------------------------------------------------
Benchmark                                                                   Time             CPU   Iterations
-------------------------------------------------------------------------------------------------------------
BM_SplitBGR24Planar/min_time:2.000/repeats:5_mean                        2.02 ms         2.02 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_median                      2.01 ms         2.01 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_stddev                     0.023 ms        0.016 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_cv                          1.13 %          0.79 %             5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_mean         2.02 ms         2.01 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_median       2.00 ms         2.01 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_stddev      0.026 ms        0.028 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_cv           1.29 %          1.41 %             5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_mean          2.01 ms         2.00 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_median        2.00 ms         2.01 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_stddev       0.016 ms        0.017 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_cv            0.82 %          0.84 %             5
````

#### OSX

````bash
$ ./install/osx/bin/Demo
Unable to determine clock rate from sysctl: hw.cpufrequency: No such file or directory
This does not affect benchmark measurements, only the metadata output.
***WARNING*** Failed to set thread affinity. Estimated CPU frequency may be incorrect.
2025-12-14T11:05:31+09:00
Running ./install/osx/bin/Demo
Run on (8 X 24 MHz CPU s)
CPU Caches:
  L1 Data 64 KiB
  L1 Instruction 128 KiB
  L2 Unified 4096 KiB (x8)
Load Average: 2.50, 2.56, 2.53
***WARNING*** Library was built as DEBUG. Timings may be affected.
-------------------------------------------------------------------------------------------------------------
Benchmark                                                                   Time             CPU   Iterations
-------------------------------------------------------------------------------------------------------------
BM_SplitBGR24Planar/min_time:2.000/repeats:5_mean                       0.154 ms        0.154 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_median                     0.154 ms        0.154 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_stddev                     0.004 ms        0.004 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_cv                          2.74 %          2.57 %             5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_mean        0.249 ms        0.249 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_median      0.249 ms        0.249 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_stddev      0.004 ms        0.004 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_cv           1.73 %          1.50 %             5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_mean         0.247 ms        0.247 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_median       0.246 ms        0.246 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_stddev       0.001 ms        0.001 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_cv            0.24 %          0.25 %             5
````