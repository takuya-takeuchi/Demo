# Get started

## Abstracts

* Input simple onnx file and inference sample tensor

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.5 or higher

### Windows

* Visual Studio 2022

### Ubuntu

* g++
* libiconv

### OSX

* Xcode
* libiconv

## Dependencies

* [ONNX Runtime](https://onnxruntime.ai/)
  * 1.16.3
  * MIT license

## How to build?

### ONNX Runtime

Go to [ONNXRuntime/C](..).

Once time you built `ONNX Runtime`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

#### Windows

````bat
$ .\install\win\bin\Demo.exe testdata\mnist.onnx Input3 1 28 28 Plus214_Output_0
[Info] output tensor: {1, 10}
````

#### Linux

````bat
$ ./install/linux/bin/Demo testdata/mnist.onnx Input3 1 28 28 Plus214_Output_0
[Info] output tensor: {1, 10}
````

#### OSX

````shell
$ ./install/osx/bin/Demo testdata/mnist.onnx Input3 1 28 28 Plus214_Output_0 
[Info] output tensor: {1, 10}
````