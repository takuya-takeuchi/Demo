# Dockerfiles examples

## Abstracts

* How to build dockerfile contains components you want

## Requirements

* Docker

## List of dockerfiles

### protobuf

|Target OS|Version|Location|Build command example|
|---|---|---|---|
|ubuntu|18.04|[./protobuf/ubuntu/18.04/Dockerfile](./protobuf/ubuntu/18.04/Dockerfile)|`docker build -t ubuntu-18.04-protobuf protobuf/ubuntu/18.04 --build-arg COMPONENT_VERSION=v3.20.3`|
