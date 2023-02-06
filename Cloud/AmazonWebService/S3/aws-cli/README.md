# Cheat Sheet for S3

## Create Bucket

````shell
$ aws s3 mb s3://<bucker_name>
make_bucket: <bucker_name>
````

#### Check Points

* Block public access is disable if you want to expose bucket?
* Bucket policy allow us get, put and delete actions?

## Create Pre-Signed URL

````shell
$ aws s3 presign s3://<bucker_name>/<upload-file-path> --expires-in <sec> --region <resion>
https://<bucker_name>.s3.amazonaws.com/<bucker_name>/<upload-file-path>?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIA5LJ4NHJDMVUECNMK%2F20230204%2F<resion>%2Fs3%2Faws4_request&X-Amz-Date=20230204T141335Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=352abdc27172572c9d6813c9bdcbc361277787a8f07936faa99fb216024277a0
````

You can upload file via this pre-signed url by using `curl` like http client.

````shell
$ curl -X PUT --upload-file lenna.jpg "https://<bucker_name>.s3.amazonaws.com/<bucker_name>/lenna.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIA5LJ4NHJDMVUECNMK%2F20230204%2F<resion>%2Fs3%2Faws4_request&X-Amz-Date=20230204T141335Z&X-Amz-Expires=600&X-Amz-SignedHeaders=host&X-Amz-Signature=352abdc27172572c9d6813c9bdcbc361277787a8f07936faa99fb216024277a0"
````

#### Check Points

* IAM user must have privilege to access bucket if bucket owner is other user
  * Url is published w/o error even if IAM user has no privilege to access bucket

## Enumerate objects

````shell
$ aws s3 ls s3://<bucker_name>
2023-02-04 23:40:55     227148 fileA.jpg
````

#### Check Points

* `s3:ListBucket` action is required
* IAM user must have privilege to access bucket if bucket owner is other user 