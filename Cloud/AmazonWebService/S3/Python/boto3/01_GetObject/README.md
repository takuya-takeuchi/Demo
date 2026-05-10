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

Go to [boto3](..) and read [Setup](../README.md#Setup).
Once time you cratead virtual python environmental, you need not to do again.

### Windows

````bat
$ set AWS_ACCESS_KEY_ID=xxxxxxxx
$ set AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ python main.py -b ZZZZZZZZZZZZZZ -k lenna.jpg -r ap-northeast-1
Arguments
  bucket_name: ZZZZZZZZZZZZZZ
          key: lenna.jpg
       region: ap-northeast-1
[Info] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'.
````

### Linux

````bash
$ export AWS_ACCESS_KEY_ID=xxxxxxxx
$ export AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ python main.py -b ZZZZZZZZZZZZZZ -k lenna.jpg -r ap-northeast-1
Arguments
  bucket_name: ZZZZZZZZZZZZZZ
          key: lenna.jpg
       region: ap-northeast-1
[Info] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'.
````

### OSX

````bash
$ export AWS_ACCESS_KEY_ID=xxxxxxxx
$ export AWS_SECRET_ACCESS_KEY=yyyyyyyy
$ python main.py -b ZZZZZZZZZZZZZZ -k lenna.jpg -r ap-northeast-1
Arguments
  bucket_name: ZZZZZZZZZZZZZZ
          key: lenna.jpg
       region: ap-northeast-1
[Info] Retrieved object 'lenna.jpg' from bucket 'ZZZZZZZZZZZZZZ'.
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
