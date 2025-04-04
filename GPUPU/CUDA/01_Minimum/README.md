# Minimum

## Abstracts

* Infer by TensorRT
* Use `cudaMemcpyAsync` to transfer input data from host to device

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

````shell
$ pwsh build.ps1 <Release/Debug>
````

If you want to specify CUDA, update `CUDA_PATH`.

## How to test?

````cmd
$ .\install\win\bin\Demo.exe
Start!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
Hello from CUDA kernel!
End!
````