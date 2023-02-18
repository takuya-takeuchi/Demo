# Generate Presigned Url

## Abstracts

* Generate presigned url

## Requirements

### Common

* Powershell
* CMake 3.0.0 or later
* C++ Compiler supports C++11

### Windows

* Visual Studio

### Ubuntu

* g++

### OSX

* Xcode

## Dependencies

* [AWS SDK for C++](https://github.com/aws/aws-sdk-cpp)
  * 1.11.4
  * Apache 2.0 License

## How to build?

### Build AWS SDK for C++

Go to [aws-sdk-cpp](../aws-sdk-cpp).

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

### for Upload

````shell
$ Test <your-bucket-name> lenna.jpg ap-northeast-1 PUT 600
        bucket_name: <your-bucket-name>
        object_name: lenna.jpg
             region: ap-northeast-1
             method: PUT
expirationInSeconds: 600
[Info] Aws::InitAPI
[Info] Generate Presigned Url
https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T125934Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=2a46e84e9215eb0968865824a1dfca7f7935287193992fa3652846ce34962ad6
[Info] Aws::ShutdownAPI
````

You can use this url to upload file.
For example,

````sh
$ curl -X PUT "https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T125934Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=2a46e84e9215eb0968865824a1dfca7f7935287193992fa3652846ce34962ad6" -F file=@lenna.jpg 
````

### for Download

````shell
$ Test <your-bucket-name> lenna.jpg ap-northeast-1 PUT 600
        bucket_name: <your-bucket-name>
        object_name: lenna.jpg
             region: ap-northeast-1
             method: GET
expirationInSeconds: 600
[Info] Aws::InitAPI
[Info] Generate Presigned Url
https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T131715Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=66df959ceeb2a7b7794e0b2181a93f4db4ec88d22c02aaa356cb8bc35dcd91ca
[Info] Aws::ShutdownAPI
````

You can use this url to download file which be present in S3.
Therefore, you can not use this url for uploading.
For example,

````sh
$ curl "https://<your-bucket-name>.s3.ap-northeast-1.amazonaws.com/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=XXXXXXXXXXXXXXXXXXX%2F20230218%2Fap-northeast-1%2Fs3%2Faws4_request&X-Amz-Date=20230218T125934Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=2a46e84e9215eb0968865824a1dfca7f7935287193992fa3652846ce34962ad6" -o lenna.jpg
````