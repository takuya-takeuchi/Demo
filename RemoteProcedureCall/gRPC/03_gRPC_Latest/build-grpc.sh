#!/bin/bash +x

VERSION=v1.46.3

GRPC_SOURCE_DIR=${PWD}/grpc-${VERSION}
CMAKE_DIR=${GRPC_SOURCE_DIR}/cmake
GRPC_BUILD_DIR=${CMAKE_DIR}/build

INSTALL_DIR=${GRPC_SOURCE_DIR}/install
GRPC_DIR=${INSTALL_DIR}/grpc

BUILD_TYPE=Release
BUILD_PARALLEL=16

# clone
mkdir -p ${GRPC_SOURCE_DIR}
pushd ${GRPC_SOURCE_DIR}
git clone -b ${VERSION} https://github.com/grpc/grpc .
git submodule update --init --recursive
popd

# grpc
mkdir -p ${GRPC_BUILD_DIR}
pushd ${GRPC_BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
      -D gRPC_INSTALL=ON \
      -D gRPC_BUILD_TESTS=OFF \
      -D gRPC_PROTOBUF_PROVIDER=package \
      -D gRPC_ZLIB_PROVIDER=package \
      -D gRPC_CARES_PROVIDER=package \
      ${GRPC_SOURCE_DIR}
make -j${BUILD_PARALLEL}
make install
popd