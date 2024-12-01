# Multiprocessing

## Abstracts

* Clustering sample point cloud data by multiprocessing
  * On POSIX, folk context is default but `folk` occurs deadlock with some function of Open3D. To resolve this issue, change context to `spawn` or `folkserver`.
    * See [Deadlock with multiprocessing (using fork) and OpenMP #1552](https://github.com/isl-org/Open3D/issues/1552).
  * This sample program will face to deadlock. Because c++ code can not write simple multi-processing code like python. Boost.Python can it but worker code shall be script code.

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++17

### Linux

* Ubuntu 20.04 or later
* glibc 2.31 or later
* OpenGL
  * `sudo apt-get install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev libc++-dev`

## How to build?

Go to [C++](..).

At first, you must download prebuilt binaries.

````shell
$ pwsh download.ps1
````

After that, script generates artifacts so you can run the following command.

````shell
$ pwsh build.ps1 <Debug/Release> <Open3D version>
````

## How to use?

#### Linux

````bat
$ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:install/linux/bin
$ ./install/linux/bin/Demo
[Info] Worker 0 started, Process ID: 2841768
[Info] Worker 1 started, Process ID: 2841769
[Info] Worker 2 started, Process ID: 2841770
[Info] Worker 4 started, Process ID: 2841772
[Info] Worker 3 started, Process ID: 2841771
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
Precompute neighbors.[========================================] 100%
[Info] Number of clusters detected: 10===========> ] 95%
[Info] Worker 2 finished
[Info] Number of clusters detected: 10             ] 50%
[Info] Worker 0 finished
[Info] Number of clusters detected: 10===>         ] 75%
[Info] Worker 1 finished
[Info] Number of clusters detected: 10=====>       ] 80%
[Info] Worker 4 finished
[Info] Number of clusters detected: 10============>] 97%
[Info] Worker 3 finished
````