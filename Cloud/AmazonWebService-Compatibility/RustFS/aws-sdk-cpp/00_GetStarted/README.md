# Generate Presigned Url

## Abstracts

* Generate presigned url

## Requirements

### Common

* Powershell
* CMake 3.15.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [AWS SDK for C++](https://github.com/aws/aws-sdk-cpp)
  * 1.11.798
  * Apache 2.0 License

## How to build?

### Build AWS SDK for C++

Go to [AmazonWebService-Compatibility](../../..).

````shell
$ pwsh build.ps1 <Debug/Release>
````

Once time you built AWS SDK for C++, you need not to do again.

### Build

````shell
$ pwsh build.ps1 <Debug/Release>
````

Then, program will be present in `install/<your os name>/bin`.

## How to use?

### Windows

 <endpoint> <bucket_name> <object_name> <region> <filepath>
````bat
$ .\install\win\Release\bin\Demo.exe http://192.168.11.45:9000 data-bucket test-image ap-northeast-1 lenna.jpg
````