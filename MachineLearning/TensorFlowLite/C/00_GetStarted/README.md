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
  * Apache-2.0 license

### Note

* Static library of `tensorflowlite_c.lib` is unavailable even build is succeed because it does not export APIs. So linker can not refer to them.

## How to build?

Go to [TensorFlowLite/C](..).

Once time you built `Tensorflow Lite`, you need not to do again.

````shell
$ pwsh build.ps1 <Debug/Release> <build system> <tensorflow version>
````

## How to use?

````shell
$ pwsh build.ps1 <Debug/Release> <build system> <tensorflow version>
````

For example,

````shell
$ pwsh build.ps1 Release bazel v2.5.3
````

Each arguments should be same value as values that built TensorFlow Lite.