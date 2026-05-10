# Get File from RustFS

## Abstracts

* Download file from RustFS

## Requirements

### Common

* .NET 10

## Dependencies

* [AWSSDK.Core](https://github.com/aws/aws-sdk-net/)
  * 4.0.6.1
  * Apache 2.0 License
* [AWSSDK.S3](https://github.com/aws/aws-sdk-net/)
  * 4.0.23
  * Apache 2.0 License
* [NLog](https://github.com/NLog/NLog)
  * 6.1.3
  * BSD-3-Clause License

## How to use?

### Windows

````bat
$ set AWS_ACCESS_KEY_ID=rustfsadmin
$ set AWS_SECRET_ACCESS_KEY=rustfsadmin
$ dotnet run -c <Debug/Release> -- http://192.168.11.45:9000 data-bucket /tmp/test-image.jpg ap-northeast-1
2026-05-10 19:05:13.9550 [INFO ] endpoint: http://192.168.11.45:9000 
2026-05-10 19:05:13.9972 [INFO ] bucket_name: data-bucket        
2026-05-10 19:05:13.9972 [INFO ] object_name: tmp/test-image.jpg 
2026-05-10 19:05:13.9972 [INFO ] region: ap-northeast-1 
2026-05-10 19:05:13.9972 [INFO ] GetObject
2026-05-10 19:05:15.1620 [INFO ] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
````

### Linux

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ dotnet run -c <Debug/Release> -- http://192.168.11.45:9000 data-bucket tmp/test-image.jpg ap-northeast-1
2026-05-10 19:00:49.3168 [INFO ] endpoint: http://192.168.11.45:9000 
2026-05-10 19:00:49.4059 [INFO ] bucket_name: data-bucket 
2026-05-10 19:00:49.4059 [INFO ] object_name: tmp/test-image.jpg 
2026-05-10 19:00:49.4062 [INFO ] region: ap-northeast-1 
2026-05-10 19:00:49.4062 [INFO ] GetObject 
2026-05-10 19:00:50.2078 [INFO ] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'. 
````

### OSX

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ dotnet run -c <Debug/Release> -- http://192.168.11.45:9000 data-bucket tmp/test-image.jpg ap-northeast-1
2026-05-10 19:15:11.5467 [INFO ] endpoint: http://192.168.11.45:9000 
2026-05-10 19:15:11.5586 [INFO ] bucket_name: data-bucket 
2026-05-10 19:15:11.5586 [INFO ] object_name: tmp/test-image.jpg 
2026-05-10 19:15:11.5586 [INFO ] region: ap-northeast-1 
2026-05-10 19:15:11.5586 [INFO ] GetObject 
2026-05-10 19:15:11.8883 [INFO ] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
````
