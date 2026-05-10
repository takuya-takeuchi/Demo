# Get File from RustFS

## Abstracts

* Download file from RustFS

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
$ set AWS_ACCESS_KEY_ID=rustfsadmin
$ set AWS_SECRET_ACCESS_KEY=rustfsadmin
$ python main.py -e http://192.168.11.45:9000 -b data-bucket -k tmp/test-image.jpg -r ap-northeast-1
Arguments
     endpoint: http://192.168.11.45:9000
  bucket_name: data-bucket
          key: tmp/test-image.jpg
       region: ap-northeast-1
[Info] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
````

### Linux

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ python main.py -e http://192.168.11.45:9000 -b data-bucket -k tmp/test-image.jpg -r ap-northeast-1
Arguments
     endpoint: http://192.168.11.45:9000
  bucket_name: data-bucket
          key: tmp/test-image.jpg
       region: ap-northeast-1
[Info] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
````

### OSX

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ python main.py -e http://192.168.11.45:9000 -b data-bucket -k tmp/test-image.jpg -r ap-northeast-1
Arguments
     endpoint: http://192.168.11.45:9000
  bucket_name: data-bucket
          key: tmp/test-image.jpg
       region: ap-northeast-1
[Info] Retrieved object 'tmp/test-image.jpg' from bucket 'data-bucket'.
````