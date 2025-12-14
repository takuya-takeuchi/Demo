# Normalize (optimized for AVX2)

## Abstracts

* Normalize RGB data by optimized code for AVX2

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

## How to build?

### libyuv

### benchmark

Go to [benchmark](..).

````shell
$ pwsh build-benchmark.ps1 <Debug/Release>
````

Once time you built `benchmark`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ ./install/linux/win/Demo
2025-12-14T13:47:17+09:00
Running .\install\win\bin\Demo.exe
Run on (12 X 3192 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x6)
  L1 Instruction 32 KiB (x6)
  L2 Unified 256 KiB (x6)
  L3 Unified 12288 KiB (x1)
------------------------------------------------------------------------------------
Benchmark                                          Time             CPU   Iterations
------------------------------------------------------------------------------------
BM_Scalar/min_time:2.000/repeats:5_mean         3.38 ms         3.39 ms            5
BM_Scalar/min_time:2.000/repeats:5_median       3.35 ms         3.35 ms            5
BM_Scalar/min_time:2.000/repeats:5_stddev      0.055 ms        0.060 ms            5
BM_Scalar/min_time:2.000/repeats:5_cv           1.64 %          1.77 %             5
BM_AVX2/min_time:2.000/repeats:5_mean           1.92 ms         1.92 ms            5
BM_AVX2/min_time:2.000/repeats:5_median         1.91 ms         1.90 ms            5
BM_AVX2/min_time:2.000/repeats:5_stddev        0.053 ms        0.048 ms            5
BM_AVX2/min_time:2.000/repeats:5_cv             2.78 %          2.51 %             5
````

#### Linux

````bash
$ ./install/osx/bin/Demo
2025-12-14T11:30:21+09:00
Running ./install/linux/bin/Demo
Run on (4 X 3092.84 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 10240 KiB (x4)
Load Average: 0.72, 0.64, 0.38
-------------------------------------------------------------------------------------------------------------
Benchmark                                                                   Time             CPU   Iterations
-------------------------------------------------------------------------------------------------------------
BM_SplitBGR24Planar/min_time:2.000/repeats:5_mean                        2.70 ms         2.70 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_median                      2.69 ms         2.69 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_stddev                     0.042 ms        0.042 ms            5
BM_SplitBGR24Planar/min_time:2.000/repeats:5_cv                          1.55 %          1.55 %             5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_mean        0.980 ms        0.980 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_median      0.981 ms        0.981 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_stddev      0.007 ms        0.007 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2OFF/min_time:2.000/repeats:5_cv           0.72 %          0.71 %             5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_mean         0.932 ms        0.932 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_median       0.922 ms        0.922 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_stddev       0.045 ms        0.045 ms            5
BM_SplitBGR24PlanarByLibYuvAVX2ON/min_time:2.000/repeats:5_cv            4.82 %          4.82 %             5
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