# libiconv

## Abstracts

* Use libiconv and convert shit-jis text to utf-8 text

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
* libcpprest-dev and libboost-program-options-dev
  * via `apt` command
    * `apt install librdkafka-dev libboost-program-options-dev`

### OSX

* Xcode
* librdkafka, googletest, boost, rapidjson and lz4
  * via `brew` command
    * `brew install librdkafka googletest boost rapidjson lz4`

## Dependencies

* [libiconv](https://www.gnu.org/software/libiconv/)
  * GNU Lesser General Public License

## How to use?

At first, you must build [cppkafka](https://github.com/aws/aws-sdk-cpp).

````shell
$ git submodule update --init --recursive .
$ pwsh build.ps1  <Debug/Release>
````

Then you can try samples. For example [01_Producer](./01_Producer).