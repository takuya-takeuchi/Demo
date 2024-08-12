# ONNX Runtime (Python API)

## Abstracts

* Build ONNX Runtime

## Requirements

### Common

* Powershell 7 or later
* Python 3.9 or later
  * Requires `ninja` and `flatbuffers`
* CMake 3.26 or later
* Node.js (16.0+, 18.0+ is recommended)

## Dependencies

* [ONNX Runtime](https://onnxruntime.ai/)
  * 1.18.1
  * MIT license

## How to build?

For now, we tested on only Windows.

````shell
$ pwsh build.ps1 <Release/Debug>
````