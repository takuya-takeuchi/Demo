# Producer

## Abstracts

* Create topic and publish

## Requirements

### Common

* Powershell 7 or later
* CMake 3.0.0 or later
* C++ Compiler supports C++14

### Windows

* Visual Studio
* cppkafka
  * via `vcpkg` command
    * `vcpkg install librdkafka --triplet x64-windows-static`

#### NOTE

For now, program can not link `cppkafka` due to C2491 error.

* [Error building cppkafka on Windows](https://github.com/mfontanini/cppkafka/issues/10)
* [Error building on Windows (again)](https://github.com/mfontanini/cppkafka/issues/255)

### Ubuntu

* g++
* libcpprest-dev
  * via `apt` command
    * `apt install librdkafka-dev`

### OSX

* Xcode
* cpprestsdk
  * via `brew` command

## Dependencies

* [librdkafka](https://github.com/confluentinc/librdkafka)
  * 1.9.2
  * BSD-2-Clause license
* [cppkafka](https://github.com/mfontanini/cppkafka)
  * v0.3.1
  * BSD-3-Clause license

## How to build?

### cppkafka

Go to [cppkafka](../cppkafka).

````shell
$ pwsh build.ps1 <Debug/Release>
````

Once time you built `cppkafka`, you need not to do again.



````shell
$ pwsh build.ps1 <Debug/Release>
````

## How to test?

````bat
$ install\win\bin\Test "https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T134137Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=4b5d4337e3ca087ec5526335d8081e2620c2de4466aac56b6361e63a4939ca0e" install\win\bin\lenna.jpg  
````