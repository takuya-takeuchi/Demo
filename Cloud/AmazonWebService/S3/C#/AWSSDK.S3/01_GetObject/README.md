# Get File from S3

## Abstracts

* Download file from S3

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
$ set AWS_ACCESS_KEY_ID=xxxxxxxx
$ set AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ dotnet run -c <Debug/Release> -- ZZZZZZZZZZZZZZ lenna.jpg ap-northeast-1
2026-05-10 18:32:16.3326 [INFO ] bucket_name: ZZZZZZZZZZZZZZ 
2026-05-10 18:32:16.3630 [INFO ] object_name: lenna.jpg 
2026-05-10 18:32:16.3630 [INFO ] region: ap-northeast-1
2026-05-10 18:32:16.3630 [INFO ] GetObject
2026-05-10 18:32:17.7929 [INFO ] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'.
````

### Linux

````bash
$ export AWS_ACCESS_KEY_ID=xxxxxxxx
$ export AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ dotnet run -c <Debug/Release> -- ZZZZZZZZZZZZZZ lenna.jpg ap-northeast-1
2026-05-10 17:53:35.5135 [INFO ] bucket_name: ZZZZZZZZZZZZZZ 
2026-05-10 17:53:35.5694 [INFO ] object_name: lenna.jpg 
2026-05-10 17:53:35.5694 [INFO ] region: ap-northeast-1 
2026-05-10 17:53:35.5694 [INFO ] [Info] GetObject 
2026-05-10 17:53:37.2876 [INFO ] [Info] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'.
````

### OSX

````bash
$ export AWS_ACCESS_KEY_ID=xxxxxxxx
$ export AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ dotnet run -c <Debug/Release> -- ZZZZZZZZZZZZZZ lenna.jpg ap-northeast-1
2026-05-10 18:10:23.2225 [INFO ] bucket_name: ZZZZZZZZZZZZZZ 
2026-05-10 18:10:23.2338 [INFO ] object_name: lenna.jpg 
2026-05-10 18:10:23.2339 [INFO ] region: ap-northeast-1 
2026-05-10 18:10:23.2339 [INFO ] GetObject 
2026-05-10 18:10:23.8840 [INFO ] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'. 
````

## Why does program not work?

### Error: GetObject: Access Denied

You must check the following things

#### Block public access (bucket setting)

This program does not take care of credentials. In other words, we have to disable block public access in AWS console.

<img src="images/block.png" />

#### Block policy

Bcket policy must allow write actions.
For examples, you can write json like 

````json
{
    "Version": "2012-10-17",
    "Id": "Policy9999999999999",
    "Statement": [
        {
            "Sid": "Stmt9999999999999",
            "Effect": "Allow",
            "Principal": "*",
            "Action": [
                "s3:DeleteObject",
                "s3:GetObject",
                "s3:PutObject"
            ],
            "Resource": "arn:aws:s3:::<your-bucket-name>/*"
        }
    ]
}
````
