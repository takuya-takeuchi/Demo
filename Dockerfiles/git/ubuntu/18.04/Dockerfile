FROM ubuntu:18.04
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

ENV GIT_VERSION 2.17.1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    fakeroot \
    dpkg-dev \
    git
RUN sed -i '/deb-src/s/^# //' /etc/apt/sources.list && apt update
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get build-dep -y  \
    git
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcurl4-openssl-dev 
WORKDIR /opt/git/source-git
RUN apt-get update && apt-get source -y \
    git \
 && apt-get clean && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/git/source-git/git-${GIT_VERSION}/
RUN sed -i -- 's/libcurl4-gnutls-dev/libcurl4-openssl-dev/' ./debian/control \
 && sed -i -- '/TEST\s*=\s*test/d' ./debian/rules \
 && dpkg-buildpackage -rfakeroot -b -uc -us \
 && dpkg -i ../git_*ubuntu*.deb