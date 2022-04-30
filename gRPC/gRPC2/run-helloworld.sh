#!/bin/bash +x

VERSION=v1.26.0

GRPC_SOURCE_DIR=${PWD}/grpc-${VERSION}
CMAKE_DIR=${GRPC_SOURCE_DIR}/cmake
CMAKE_BUILD_DIR=${CMAKE_DIR}/build

INSTALL_DIR=${CMAKE_BUILD_DIR}/install
GRPC_DIR=${INSTALL_DIR}/grpc
BORINGSSL_DIR=${INSTALL_DIR}/boringssl
ZLIB_DIR=${INSTALL_DIR}/zlib
CARES_DIR=${INSTALL_DIR}/cares
PROTOBUF_DIR=${INSTALL_DIR}/protobuf

BUILD_TYPE=Release
BUILD_PARALLEL=16

HELLOWORLD_DIR=${GRPC_SOURCE_DIR}/examples/cpp/helloworld
BUILD_DIR=${HELLOWORLD_DIR}/build
mkdir -p ${BUILD_DIR}
pushd ${BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D OPENSSL_ROOT_DIR="${BORINGSSL_DIR}" \
      -D CMAKE_PREFIX_PATH="${GRPC_DIR};${CARES_DIR};${PROTOBUF_DIR};${ZLIB_DIR}" \
      ..
cmake --build .

# run server 
./greeter_server &
# run client
./greeter_client

popd