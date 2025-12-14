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

## Dependencies

* [benchmark](https://github.com/google/benchmark)
  * v1.9.4
  * Apache-2.0 License

## How to build?

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
$ ./install/linux/bin/Demo
2025-12-14T14:06:09+09:00
Running ./install/linux/bin/Demo
Run on (4 X 3092.84 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 10240 KiB (x4)
Load Average: 0.21, 0.17, 0.14
------------------------------------------------------------------------------------
Benchmark                                          Time             CPU   Iterations
------------------------------------------------------------------------------------
BM_Scalar/min_time:2.000/repeats:5_mean         3.29 ms         3.29 ms            5
BM_Scalar/min_time:2.000/repeats:5_median       3.29 ms         3.28 ms            5
BM_Scalar/min_time:2.000/repeats:5_stddev      0.021 ms        0.021 ms            5
BM_Scalar/min_time:2.000/repeats:5_cv           0.63 %          0.64 %             5
BM_AVX2/min_time:2.000/repeats:5_mean           4.07 ms         4.06 ms            5
BM_AVX2/min_time:2.000/repeats:5_median         4.06 ms         4.05 ms            5
BM_AVX2/min_time:2.000/repeats:5_stddev        0.039 ms        0.040 ms            5
BM_AVX2/min_time:2.000/repeats:5_cv             0.95 %          0.99 %             5
````