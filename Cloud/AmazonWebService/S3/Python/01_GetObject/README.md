# Get File from S3

## Abstracts

* Download file from S3

## Requirements

### Common

* Python3 or later

## Dependencies

* [boto3](https://github.com/boto/boto3)
  * 1.43.6
  * Apache 2.0 License

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
$ export AWS_ACCESS_KEY_ID=xxxxxxxx
$ export AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ python main.py -b ve8rjmzuy84wq7f2iyhp -k lenna.jpg -r ap-northeast-1
Arguments
  bucket_name: ve8rjmzuy84wq7f2iyhp
          key: lenna.jpg
       region: ap-northeast-1
[Info] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
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
