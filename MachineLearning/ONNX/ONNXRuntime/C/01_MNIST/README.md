# MNIST

## Abstracts

* Input simple onnx file and mnist image file
  * You can specify CPU or GPU

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher
* CUDA 11.8

### Windows

* Visual Studio 2022

### Ubuntu

* g++
* libiconv

## Dependencies

* [ONNX Runtime](https://onnxruntime.ai/)
  * 1.16.3
  * MIT license
* [opencv](https://github.com/opencv/opencv)
  * 4.7.0
  * Apache-2.0 License

## How to build?

### ONNX Runtime

Go to [ONNXRuntime/C](..).

Once time you built `ONNX Runtime`, you need not to do again.

````shell
$ pwsh build-cuda.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe mnist.onnx mnist-7.bmp cuda
0:0.000000
1:0.000000
2:0.000000
3:0.000000
4:0.000000
5:0.000000
6:0.000000
7:1.000000
8:0.000000
9:0.000000
````

#### Linux

````bat
$ ./install/osx/bin/Demo mnist.onnx mnist-7.bmp cpu
0:0.000000
1:0.000000
2:0.000000
3:0.000000
4:0.000000
5:0.000000
6:0.000000
7:1.000000
8:0.000000
9:0.000000
````