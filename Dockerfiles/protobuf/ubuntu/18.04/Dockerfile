FROM ubuntu:18.04
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

ARG COMPONENT_VERSION

# build protobuf
RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf \
    automake \
    libtool \
    git \
    wget \
    curl \
    make \
    g++ \
    unzip \
    ca-certificates \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN update-ca-certificates

# install latest cmake
ENV CMAKE_VERSION 3.25.0
ENV CMAKE_FILE cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz
ENV CMAKE_URL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_FILE}
ENV CMAKE_URL_FILE ${CMAKE_FILE}
RUN wget ${CMAKE_URL} && tar -xzf ${CMAKE_FILE} \
 && rm ${CMAKE_FILE} \
 && mv ${CMAKE_FILE%.tar.gz} /opt/cmake \
 && ln -s /opt/cmake/bin/* /usr/bin

# build and install protobuf
WORKDIR /opt/protobuf
RUN git clone -b ${COMPONENT_VERSION} https://github.com/protocolbuffers/protobuf.git . \
 && git submodule update --init --recursive
RUN ./autogen.sh && ./configure
RUN make && make install && ldconfig