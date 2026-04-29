# ListBuckets

## Abstracts

* Enumerate backets

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

<img src="./images/image.png" />

### Windows

````bat
$ set AWS_ACCESS_KEY_ID=rustfsadmin
$ set AWS_SECRET_ACCESS_KEY=rustfsadmin
$ .\install\win\Release\bin\Demo.exe http://192.168.11.45:9000 ap-northeast-1
[Info]    endpoint: http://192.168.11.45:9000
[Info]      region: ap-northeast-1
[Info] Aws::InitAPI
[Info] Use DefaultAWSCredentialsProviderChain like IAM Role etc
[Error] while ListBuckets InvalidAccessKeyId The Access Key Id you provided does not exist in our records.
[Info] Aws::ShutdownAPI
````

### Linux

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ ./install/linux/Release/bin/Demo http://192.168.11.45:9000 ap-northeast-1
[Info]    endpoint: http://192.168.11.45:9000
[Info]      region: ap-northeast-1
[Info] Aws::InitAPI
[Info] Use access key and secret key
data-bucket     2026-04-29T19:11:45Z
log-bucket      2026-04-29T19:11:46Z
[Info] Aws::ShutdownAPI
````

### OSX

````bash
$ ./install/osx/Release/bin/Demo http://192.168.11.45:9000 ap-northeast-1
[Info]    endpoint: http://192.168.11.45:9000
[Info]      region: ap-northeast-1
[Info] Aws::InitAPI
[Info] Use access key and secret key
data-bucket     2026-04-29T19:11:45Z
log-bucket      2026-04-29T19:11:46Z
[Info] Aws::ShutdownAPI
````