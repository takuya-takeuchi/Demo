#!/bin/bash +x

docker build -t grpc-build-ubuntu-18.04 .
docker run --rm -v ${PWD}:/opt/data -w /opt/data -it grpc-build-ubuntu-18.04 /bin/bash