# Multiprocessing

## Abstracts

* Clustering sample point cloud data by multiprocessing
  * On POSIX, folk context is default but `folk` occurs deadlock with some function of Open3D. To resolve this issue, change context to `spawn` or `folkserver`.
    * See [Deadlock with multiprocessing (using fork) and OpenMP #1552](https://github.com/isl-org/Open3D/issues/1552).

## Requirements

### Common

* Python 3.7 or later

## Dependencies

* [open3d](https://github.com/isl-org/Open3D)
  * v0.18.0
  * MIT license

## How to build?

### Virtual Environmental

Go to [Open3D](..).

## How to test?

#### Windows

````bat
$ python main.py
Arguments
               eps: 0.05
        min_points: 10
Worker Task-2 started, Process ID: 15372
Worker Task-1 started, Process ID: 12816
Worker Task-4 started, Process ID: 3704
Worker Task-0 started, Process ID: 23764
Worker Task-3 started, Process ID: 26596
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3====================>] 97%
Clustering[========================================] 100%
Number of clusters detected: 3
Number of clusters detected: 3
Number of clusters detected: 3
Worker Task-2 finished
Worker Task-4 finished
Worker Task-1 finished
Worker Task-3 finished
Worker Task-0 finished
All processes completed.
````

#### Linux

````shell
$ python main.py
Arguments
               eps: 0.05
        min_points: 10
Worker Task-2 started, Process ID: 15372
Worker Task-1 started, Process ID: 12816
Worker Task-4 started, Process ID: 3704
Worker Task-0 started, Process ID: 23764
Worker Task-3 started, Process ID: 26596
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3====================>] 97%
Clustering[========================================] 100%
Number of clusters detected: 3
Number of clusters detected: 3
Number of clusters detected: 3
Worker Task-2 finished
Worker Task-4 finished
Worker Task-1 finished
Worker Task-3 finished
Worker Task-0 finished
All processes completed.
````

#### OSX

````shell
$ python main.py
Arguments
               eps: 0.05
        min_points: 10
Worker Task-2 started, Process ID: 15372
Worker Task-1 started, Process ID: 12816
Worker Task-4 started, Process ID: 3704
Worker Task-0 started, Process ID: 23764
Worker Task-3 started, Process ID: 26596
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3
Clustering[========================================] 100%
Clustering[========================================] 100%
Number of clusters detected: 3====================>] 97%
Clustering[========================================] 100%
Number of clusters detected: 3
Number of clusters detected: 3
Number of clusters detected: 3
Worker Task-2 finished
Worker Task-4 finished
Worker Task-1 finished
Worker Task-3 finished
Worker Task-0 finished
All processes completed.
````