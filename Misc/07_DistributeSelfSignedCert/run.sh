#!/bin/bash +x
docker build -t nginx-cert .
docker run --name nginx-cert \
           -d -v $(pwd)/src:/usr/share/nginx/html \
           -p 80:80 nginx-cert:latest