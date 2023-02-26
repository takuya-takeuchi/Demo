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
* librdkafka, gtest, boost-optional, boost-algorithm, boost-program-options, rapidjson and lz4
  * via `vcpkg` command
    * `vcpkg install librdkafka gtest boost-optional boost-algorithm boost-program-options rapidjson lz4 --triplet x64-windows`

### Ubuntu

* g++
* libcpprest-dev
  * via `apt` command
    * `apt install librdkafka-dev`

### OSX

* Xcode
* librdkafka, googletest, boost, rapidjson and lz4
  * via `brew` command
    * `brew install librdkafka googletest boost rapidjson lz4`

## Dependencies

* [boost](https://www.boost.org/)
  * 1.80.0
  * Boost Software License
* [googletest](https://github.com/google/googletest)
  * 1.12.1
  * BSD-3-Clause license
* [librdkafka](https://github.com/confluentinc/librdkafka)
  * 1.9.2
  * BSD-2-Clause license
* [modern-cpp-kafka](https://github.com/morganstanley/modern-cpp-kafka)
  * v2020.01.05
  * Apache-2.0 license
* [LZ4](https://github.com/lz4/lz4)
  * 1.9.3
  * BSD-2-Clause license
* [rapidjson](https://github.com/Tencent/rapidjson)
  * 2022-06-28
  * MIT license

## How to use?

At first, you must build [cppkafka](https://github.com/aws/aws-sdk-cpp).

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````

Then you can try samples. For example [01_Producer](./01_Producer).