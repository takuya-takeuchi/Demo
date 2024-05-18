# Get started Tensorflow Lite C

## Abstracts

* Test tensorflow lite file

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher

### Windows

* Visual Studio 2019 or later

## Dependencies

* [TensorFlow](https://github.com/tensorflow/tensorflow)
  * v2.5.3, v2.10.1, v2.11.1, v2.12.1
  * Apache-2.0 license
* [mobilenetv2-12.onnx](https://github.com/onnx/models)
  * [MobileNet v2-1.0-fp32](https://github.com/onnx/models/blob/main/validated/vision/classification/mobilenet/model/mobilenetv2-12.onnx)
  * Apache-2.0 license

### Note

* Static library of `tensorflowlite_c.lib` is unavailable even build is succeed because it does not export APIs. So linker can not refer to them.

## How to build?

Go to [TensorFlowLite/C](..).

Once time you built `Tensorflow Lite`, you need not to do again.

Then,

````shell
$ pwsh build.ps1 <Debug/Release> <build system> <tensorflow version>
````

For example,

````shell
$ pwsh build.ps1 Release bazel v2.5.3
````

Each arguments should be same value as values that built TensorFlow Lite.

# How to use?

````shell
$ cd install\win\bazel\bin
$ Demo.exe mobilenetv2-12_integer_quant.tflite 1 float
[Info] tensorflow lite:2.12.1
[Info] TfLiteTensorByteSize: 602112
[Info]  TfLiteTensorNumDims: 4
        0:1
        1:224
        2:224
        3:3
[Info]   Average Inference time: 1086.00000000 ms
[Info] TfLiteTensorByteSize: 4000
[Info]  TfLiteTensorNumDims: 2
        0:1
        1:1000
[Info] Finish
````