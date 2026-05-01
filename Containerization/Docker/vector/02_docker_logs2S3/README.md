# from docker_logs to AWS S3

## Abstracts

* use `docker_logs` input to collect app's stdin logs via docker log driver
* use `remap` transform to remove unnecessary elements and screen out input containers
* use `aws_s3` sink to publish logs to S3, this demo configures for S3 Compatibility distributed object storage system (e.g. RustFS)

## Dependencies

* [vector](https://github.com/vectordotdev/vector)
  * v0.55.0
  * Mozilla Public License 2.0

## How to use?

At first, build app image.

````bash
$ pwsh build.ps1
````

Then, lanuch container and show logs.

````bash
$ export AWS_ACCESS_KEY_ID=rustfsadmin
$ export AWS_SECRET_ACCESS_KEY=rustfsadmin
$ export AWS_REGION=ap-northeast-1
$ export S3_ENDPOINT=http://192.168.11.45:9000
$ export S3_BUCKET_NAME=log-bucket
$ docker compose up -d
````

<img src="./images/rustfs_1.png" />

<img src="./images/rustfs_2.png" />