# ONNX Runtime (C API)

## Abstracts

* Build ONNX Runtime

## Requirements

### Common

* Powershell 7 or later
* CMake
  * 3.26 or higher
* python packages `onnx`, `onnxruntime` and `flatbuffers` for minimal build

### Windows

* Visual Studio 2022

### Ubuntu

* g++

### OSX

* Xcode
* libiconv

## Dependencies

* [ONNX Runtime](https://onnxruntime.ai/)
  * 1.24.3
  * MIT license

## How to use?

````shell
$ pwsh build.ps1  <Debug/Release>
````

## Misc

Generate ORT format by

````bash
$ python3 onnxruntime/tools/python/create_reduced_build_config.py 00_GetStarted/testdata
````