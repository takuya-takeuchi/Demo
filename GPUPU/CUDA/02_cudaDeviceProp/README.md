# cudaDeviceProp

## Abstracts

* Check nvidia gpu device capabilities

## Requirements

### Common

* Powershell 7 or later
* CMake 3.18 or later
* CUDA

### Windows

* Visual Stduio
  * 2022 if CUDA is 12.4
  * 2019 if CUDA is 12.3 or older

## Dependencies

* N/A

## How to build?

At first, you must check your gpu's **Compute Capability**.
Then, modify `set_target_properties(${PROJ_NAME} PROPERTIES CUDA_ARCHITECTURES "75")` in [CMakeLists.txt](./CMakeLists.txt).
GPU code does not work if you specify larger value for `CUDA_ARCHITECTURES` than your gpu's **Compute Capability**.

````shell
$ pwsh build.ps1 <Release/Debug>
````

If you want to specify CUDA, update `CUDA_PATH`.

## How to test?

````cmd
$ .\install\win\bin\Demo.exe
Device 0: NVIDIA GeForce GTX 1080
  Compute capability: 6.1
  Total global memory: 8191 MB
  Multiprocessors: 20
  Clock rate: 1733 MHz
  Max threads per block: 1024
  Max threads dim: [1024, 1024, 64]
  Max grid size: [2147483647, 65535, 65535]
````