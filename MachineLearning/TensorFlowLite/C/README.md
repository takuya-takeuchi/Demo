# TensorFlow Lite C

## Abstracts

* Build Tensorflow Lite C
* Use embed Python to avoid getting dirty system's Python

## Requirements

### Common

* Powershell 7 or later
* CMake (3.26 or higher) or Bazel

### Windows

* Visual Studio 2019
* MSYS2

## Dependencies

* [TensorFlow](https://github.com/tensorflow/tensorflow)
  * 2.5.3, v2.10.1
  * Apache-2.0 license
* [CPython](https://github.com/python/cpython)
  * Python License

### Note

* Static library of `tensorflowlite_c.lib` is unavailable even build is succeed because it does not export APIs. So linker can not refer to them.

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release>
````