# Minimum

## Abstracts

* Miminum cuda program

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