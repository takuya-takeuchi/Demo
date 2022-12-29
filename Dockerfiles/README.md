# Dockerfiles examples

## Abstracts

* How to build dockerfile contains components you want

## Requirements

* Docker

## List of dockerfiles

### CMake

|Target OS|Version|Location|Build command example|Run command example|
|---|---|---|---|---|
|ubuntu|18.04|[./protobuf/ubuntu/18.04/Dockerfile](./protobuf/ubuntu/18.04/Dockerfile)|`docker build -t ubuntu-18.04-cmake cmake/ubuntu/18.04 --build-arg COMPONENT_VERSION=3.25.1`|`docker run --rm -it ubuntu-18.04-cmake cmake --version`|

### protobuf

|Target OS|Version|Location|Build command example|Run command example|
|---|---|---|---|---|
|ubuntu|18.04|[./protobuf/ubuntu/18.04/Dockerfile](./protobuf/ubuntu/18.04/Dockerfile)|`docker build -t ubuntu-18.04-protobuf protobuf/ubuntu/18.04 --build-arg COMPONENT_VERSION=v3.20.3`|-|
