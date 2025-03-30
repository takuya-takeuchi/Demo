# Benchmark of delete performance

## Abstracts

* Run a benchmark to compare 4 delete methods for PostgreSQL database via Entrity Framework
  * Observe that the performance varies depending on the use of `System.Data.Entity.DbContext.SaveChanges`, `System.Data.Entity.DbSet<TEntity>.Remove` and `System.Data.Entity.DbSet<TEntity>.RemoveRange`

## Requirements

* .NET 8.0

## Dependencies

* [BenchmarkDotNet](https://github.com/dotnet/BenchmarkDotNet)
  * MIT license
  * 0.14.0
* [NLog](https://github.com/NLog/NLog)
  * BSD-3-Clause License
  * 5.4.0
* [Npgsql Entity Framework Core provider for PostgreSQL](https://github.com/npgsql/efcore.pg)
  * PostgreSQL license
  * 9.0.4
* [R](https://cran.r-project.org)
  * GNU General Public License (GPL) version 2.0 or GNU General Public License (GPL) version 3.0
  * 4.4.3

## Preparation

1. Install R package from [CRAN](https://cran.r-project.org).
2. Open R console
3. Install `ggplot2` by `install.packages("ggplot2")` command
4. Set `C:\Program Files\R\R-4.4.3\bin\x64` to `PATH` environmental value.

## How to run?

````bash
$ dotnet run -c Release
// Validating benchmarks:
// ***** BenchmarkRunner: Start   *****
// ***** Found 12 benchmark(s) in total *****
// ***** Building 1 exe(s) in Parallel: Start   *****
...
// ***** BenchmarkRunner: End *****
Run time: 00:33:24 (2004.4 sec), executed benchmarks: 12

Global total time: 00:34:06 (2046.43 sec), executed benchmarks: 12
// * Artifacts cleanup *
Artifacts cleanup is finished
````

##### Results

```
BenchmarkDotNet v0.14.0, Windows 10 (10.0.19045.5608/22H2/2022Update)
Intel Core i7-8700 CPU 3.20GHz (Coffee Lake), 1 CPU, 12 logical and 6 physical cores
.NET SDK 8.0.303
  [Host]     : .NET 8.0.11 (8.0.1124.51707), X64 RyuJIT AVX2
  Job-CCFRSP : .NET 8.0.11 (8.0.1124.51707), X64 RyuJIT AVX2

InvocationCount=1  UnrollFactor=1  
```

| Method                        | NumberOfItems | Mean          | Error       | StdDev      | Median        |
|------------------------------ |-------------- |--------------:|------------:|------------:|--------------:|
| **RemoveWithSaveChangesEachTime** | **10**            |     **57.919 ms** |   **2.1851 ms** |   **6.3393 ms** |     **57.236 ms** |
| RemoveWithSaveChangeAtEnd     | 10            |     17.971 ms |   1.0233 ms |   3.0173 ms |     18.282 ms |
| RemoveRangeWithSaveChange     | 10            |      7.754 ms |   0.3145 ms |   0.9075 ms |      7.753 ms |
| **RemoveWithSaveChangesEachTime** | **100**           |    **511.667 ms** |   **9.8652 ms** |   **8.2379 ms** |    **512.901 ms** |
| RemoveWithSaveChangeAtEnd     | 100           |     78.173 ms |   5.3786 ms |  15.7744 ms |     70.655 ms |
| RemoveRangeWithSaveChange     | 100           |     13.388 ms |   0.4311 ms |   1.2508 ms |     13.389 ms |
| **RemoveWithSaveChangesEachTime** | **1000**          |  **5,417.481 ms** |  **47.6362 ms** |  **44.5589 ms** |  **5,420.285 ms** |
| RemoveWithSaveChangeAtEnd     | 1000          |    608.372 ms |  18.7037 ms |  54.5595 ms |    587.165 ms |
| RemoveRangeWithSaveChange     | 1000          |     46.591 ms |   0.9308 ms |   1.9430 ms |     45.888 ms |
| **RemoveWithSaveChangesEachTime** | **10000**         | **64,118.755 ms** | **568.1950 ms** | **531.4899 ms** | **63,954.268 ms** |
| RemoveWithSaveChangeAtEnd     | 10000         |  5,831.041 ms | 115.3955 ms | 118.5027 ms |  5,805.222 ms |
| RemoveRangeWithSaveChange     | 10000         |    434.579 ms |   8.5353 ms |   9.4870 ms |    436.958 ms |