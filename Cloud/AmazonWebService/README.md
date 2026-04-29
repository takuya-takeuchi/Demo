# Amazon Web Service

## Abstracts

* Build aws-sdk-cpp
  * This artifact are used in subdirectory

## Requirements

### Common

* Powershell
* CMake 3.15.0 or later
* C++ Compiler

### Windows

* Visual Studio

### Ubuntu

* g++
* `libcurl4-openssl-dev` and `libz3-dev`
  * via `apt` command

### OSX

* Xcode

## Dependencies

* [AWS SDK for C++](https://github.com/aws/aws-sdk-cpp)
  * 1.11.798
  * Apache 2.0 License

## How to use?

At first, you must build [AWS SDK for C++](https://github.com/aws/aws-sdk-cpp).

````shell
$ pwsh build.ps1 <Debug/Release>
````

You can change modules you want to use by edit [build-config.json](./build-config.json).
For example,  change `{ "option": "ec2",  "flag": false },` to `{ "option": "ec2", "flag": true },`, `aws-cpp-sdk-ec2` module will be generated.
