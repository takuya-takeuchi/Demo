FROM ubuntu:24.04 AS builder
LABEL maintainer="Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    wget \
    ca-certificates \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN update-ca-certificates

# install latest cmake
ENV CMAKE_VERSION=3.31.12
RUN CMAKE_FILE=cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz \
 && CMAKE_URL=https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_FILE} \
 && wget ${CMAKE_URL} && tar -xzf ${CMAKE_FILE} \
 && rm ${CMAKE_FILE} \
 && mv ${CMAKE_FILE%.tar.gz} /opt/cmake \
 && ln -s /opt/cmake/bin/* /usr/bin

 # app
WORKDIR /opt/sources/app
ADD CMakeLists.txt .
ADD main.cpp .
RUN mkdir -p /opt/sources/app/build \
 && mkdir -p /opt/packages/app \
 && cmake -D CMAKE_INSTALL_PREFIX=/opt/packages/app \
          -D CMAKE_BUILD_TYPE=Release \
          -B /opt/sources/app/build \
          -S /opt/sources/app \
 && cmake --build /opt/sources/app/build --config Release --parallel $nproc --target install

FROM ubuntu:24.04 AS runtime
LABEL maintainer="Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

WORKDIR /app
COPY --from=builder /opt/packages/app/* /app