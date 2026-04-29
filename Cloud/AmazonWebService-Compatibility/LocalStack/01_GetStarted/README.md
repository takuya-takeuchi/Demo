# Get started LocalStack

## Abstracts

* Launch LocalStack on docker

## Requirements

* docker
  * We recommend using latest version

## Dependencies

* [localstack/localstack:1.3.1](https://hub.docker.com/r/localstack/localstack)
  * 1.3.1

## How to use?

````shell
$ docker compose up -d
````

After this, you can confirm it works fine.

````shell
$ curl -s -X GET http://<host machine address>:4566/health  | jq .
{
  "features": {
    "initScripts": "initialized"
  },
  "services": {
    "acm": "available",
    "apigateway": "available",
    "cloudformation": "available",
    "cloudwatch": "available",
    "config": "available",
    "dynamodb": "available",
    "dynamodbstreams": "available",
    "ec2": "available",
    "es": "available",
    "events": "available",
    "firehose": "available",
    "iam": "available",
    "kinesis": "available",
    "kms": "available",
    "lambda": "available",
    "logs": "available",
    "opensearch": "available",
    "redshift": "available",
    "resource-groups": "available",
    "resourcegroupstaggingapi": "available",
    "route53": "available",
    "route53resolver": "available",
    "s3": "running",
    "s3control": "available",
    "secretsmanager": "available",
    "ses": "available",
    "sns": "available",
    "sqs": "available",
    "ssm": "available",
    "stepfunctions": "available",
    "sts": "available",
    "support": "available",
    "swf": "available",
    "transcribe": "available"
  },
  "version": "1.3.1"
}
````

Or you can install `localstack` via python and use localstack command.

````shell
$ localstack status services
┏━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━┓
┃ Service                  ┃ Status      ┃
┡━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━┩
│ acm                      │ ✔ available │
│ apigateway               │ ✔ available │
│ cloudformation           │ ✔ available │
│ cloudwatch               │ ✔ available │
│ config                   │ ✔ available │
│ dynamodb                 │ ✔ available │
│ dynamodbstreams          │ ✔ available │
│ ec2                      │ ✔ available │
│ es                       │ ✔ available │
│ events                   │ ✔ available │
│ firehose                 │ ✔ available │
│ iam                      │ ✔ available │
│ kinesis                  │ ✔ available │
│ kms                      │ ✔ available │
│ lambda                   │ ✔ available │
│ logs                     │ ✔ available │
│ opensearch               │ ✔ available │
│ redshift                 │ ✔ available │
│ resource-groups          │ ✔ available │
│ resourcegroupstaggingapi │ ✔ available │
│ route53                  │ ✔ available │
│ route53resolver          │ ✔ available │
│ s3                       │ ✔ running   │
│ s3control                │ ✔ available │
│ secretsmanager           │ ✔ available │
│ ses                      │ ✔ available │
│ sns                      │ ✔ available │
│ sqs                      │ ✔ available │
│ ssm                      │ ✔ available │
│ stepfunctions            │ ✔ available │
│ sts                      │ ✔ available │
│ support                  │ ✔ available │
│ swf                      │ ✔ available │
│ transcribe               │ ✔ available │
└──────────────────────────┴─────────────┘
````

## How about persistance?

Persistance feature is not available for community.
For detail information is [Persistence Mechanism](https://docs.localstack.cloud/references/persistence-mechanism/)