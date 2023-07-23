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
* libiconv
  * via `vcpkg` command
    * `vcpkg install libiconv --triplet x64-windows`

### Ubuntu

* g++
* libcpprest-dev and libboost-program-options-dev
  * via `apt` command
    * `apt install librdkafka-dev libboost-program-options-dev`

### OSX

* Xcode
* libiconv
  * via `brew` command
    * `brew install libiconv`

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