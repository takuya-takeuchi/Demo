# cppkafka

## Abstracts

* Build cppkafka
* Minimal cppkafka sample codes

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++14

### Windows

* Visual Studio
* librdkafka
  * via `vcpkg` command
    * `vcpkg install librdkafka --triplet x64-windows-static`

### Ubuntu

* g++
* libcpprest-dev
  * via `apt` command
    * `apt install librdkafka-dev`

### OSX

* Xcode
* libcpprest
  * via `brew` command
    * `brew install librdkafka`

## Dependencies

* [librdkafka](https://github.com/confluentinc/librdkafka)
  * 1.9.2
  * BSD-2-Clause license
* [cppkafka](https://github.com/mfontanini/cppkafka)
  * v0.3.1
  * BSD-3-Clause license

## How to use?

At first, you must build [cppkafka](https://github.com/aws/aws-sdk-cpp).

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````

Then you can try samples. For example [01_Producer](./01_Producer).