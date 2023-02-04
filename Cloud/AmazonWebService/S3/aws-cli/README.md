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
