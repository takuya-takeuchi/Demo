#!/bin/bash +x

VERSION=v1.46.3

GRPC_SOURCE_DIR=${PWD}/grpc-${VERSION}
CMAKE_DIR=${GRPC_SOURCE_DIR}/cmake
CMAKE_BUILD_DIR=${CMAKE_DIR}/build

INSTALL_DIR=${CMAKE_BUILD_DIR}/install
GRPC_DIR=${INSTALL_DIR}/grpc

BUILD_TYPE=Release
BUILD_PARALLEL=16

HELLOWORLD_DIR=${GRPC_SOURCE_DIR}/examples/cpp/helloworld
BUILD_DIR=${HELLOWORLD_DIR}/build
mkdir -p ${BUILD_DIR}
pushd ${BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_PREFIX_PATH="${GRPC_DIR}" \
      ${HELLOWORLD_DIR}
cmake --build .

# run server 
./greeter_server &
# run client
./greeter_client

popd