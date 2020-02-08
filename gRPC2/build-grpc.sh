#!/bin/bash +x

VERSION=v1.26.0

GRPC_SOURCE_DIR=${PWD}/grpc-${VERSION}
THIRD_PARTY_SOURCE_DIR=${GRPC_SOURCE_DIR}/third_party
CMAKE_DIR=${GRPC_SOURCE_DIR}/cmake
CMAKE_BUILD_DIR=${CMAKE_DIR}/build
GRPC_BUILD_DIR=${CMAKE_BUILD_DIR}/grpc
BORINGSSL_BUILD_DIR=${CMAKE_BUILD_DIR}/boringssl
CARES_BUILD_DIR=${CMAKE_BUILD_DIR}/cares
PROTOBUF_BUILD_DIR=${CMAKE_BUILD_DIR}/protobuf

INSTALL_DIR=${CMAKE_BUILD_DIR}/install
GRPC_DIR=${INSTALL_DIR}/grpc
BORINGSSL_DIR=${INSTALL_DIR}/boringssl
ZLIB_DIR=${INSTALL_DIR}/zlib
CARES_DIR=${INSTALL_DIR}/cares
PROTOBUF_DIR=${INSTALL_DIR}/protobuf

BUILD_TYPE=Release
BUILD_PARALLEL=16

# clone
mkdir -p ${GRPC_SOURCE_DIR}
pushd ${GRPC_SOURCE_DIR}
git clone -b ${VERSION} https://github.com/grpc/grpc .
git submodule update --init --recursive
popd

# boringssl
mkdir -p ${BORINGSSL_BUILD_DIR}
pushd ${BORINGSSL_BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_INSTALL_PREFIX=${BORINGSSL_DIR} \
      ${THIRD_PARTY_SOURCE_DIR}/boringssl
make -j${BUILD_PARALLEL}
mkdir -p ${BORINGSSL_DIR}/lib
cp ssl/libssl.a crypto/libcrypto.a ${BORINGSSL_DIR}/lib
mkdir -p ${BORINGSSL_DIR}/include
rm -rf ${BORINGSSL_DIR}/include/openssl
cp -r ${GRPC_SOURCE_DIR}/third_party/boringssl/include/openssl ${BORINGSSL_DIR}/include/openssl
popd

# zlib
pushd ${THIRD_PARTY_SOURCE_DIR}/zlib
./configure --prefix=${ZLIB_DIR} --static
make -j${BUILD_PARALLEL}
make install
make clean
popd

# cares
mkdir -p ${CARES_BUILD_DIR}
pushd ${CARES_BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_INSTALL_PREFIX=${CARES_DIR} \
      -D CARES_STATIC=ON \
      -D CARES_SHARED=OFF \
      ${THIRD_PARTY_SOURCE_DIR}/cares/cares
make -j${BUILD_PARALLEL}
make install
popd

# protobuf
mkdir -p ${PROTOBUF_BUILD_DIR}
pushd ${PROTOBUF_BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_INSTALL_PREFIX=${PROTOBUF_DIR} \
      -D CMAKE_PREFIX_PATH="${ZLIB_DIR}" \
      -D protobuf_BUILD_TESTS=OFF \
      ${THIRD_PARTY_SOURCE_DIR}/protobuf/cmake
make -j${BUILD_PARALLEL}
make install
popd

# grpc
mkdir -p ${GRPC_BUILD_DIR}
pushd ${GRPC_BUILD_DIR}
cmake -D CMAKE_BUILD_TYPE=${BUILD_TYPE} \
      -D CMAKE_INSTALL_PREFIX=${GRPC_DIR} \
      -D gRPC_ZLIB_PROVIDER=package \
      -D gRPC_CARES_PROVIDER=package \
      -D gRPC_PROTOBUF_PROVIDER=package \
      -D gRPC_SSL_PROVIDER=package \
      -D gRPC_BUILD_CSHARP_EXT=OFF \
      -D OPENSSL_ROOT_DIR=${BORINGSSL_DIR} \
      -D CMAKE_PREFIX_PATH="${CARES_DIR};${PROTOBUF_DIR};${ZLIB_DIR}" \
      ${GRPC_SOURCE_DIR}
make -j${BUILD_PARALLEL}
make install
popd