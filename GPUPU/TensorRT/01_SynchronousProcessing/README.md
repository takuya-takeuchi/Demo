# Infer TensorRT model (Synchronous)

## Abstracts

* Infer by TensorRT
* Use `cudaMemcpy` to transfer input data from host to device

## Requirements

### Common

* Powershell 7 or later
* CMake 3 or later
* CUDA 11.0 or later
* TensorRT 8 or later

## Dependencies

* [ResNet101-DUC-12.onnx](https://github.com/onnx/models/blob/main/validated/vision/object_detection_segmentation/duc/model/ResNet101-DUC-12.onnx)
  * Apache-2.0 License

## How to build?

#### Windows 

````shell
$ set CUDA_PATH=%CUDA_PATH_V11_8%
$ set TENSORRT_PATH=<path/to/tensorrt>
$ pwsh build.ps1 <Release/Debug>
````

#### Linux using systemd

````shell
$ set CUDA_PATH=%CUDA_PATH_V11_8%
$ set TENSORRT_PATH=<path/to/tensorrt>
$ pwsh build.ps1 <Release/Debug>
````

## How to test?

#### Windows 

````cmd
$ Demo.exe ResNet101-DUC-12.trt
CUDA Runtime Version: 11.8
 CUDA Driver Version: 12.0
    TensorRT Version: 8.0.3
Using cublas a tactic source
TensorRT was linked against cuBLAS/cuBLAS LT 11.5.1 but loaded cuBLAS/cuBLAS LT 111.1.3
Using cuDNN as a tactic source
Deserialization required 2061235 microseconds.
Using cublas a tactic source
TensorRT was linked against cuBLAS/cuBLAS LT 11.5.1 but loaded cuBLAS/cuBLAS LT 111.1.3
Using cuDNN as a tactic source
Total per-runner device memory is 240175104
Total per-runner host memory is 238304
Allocated activation device memory of size 194560000
Finish.
````

#### Linux using systemd