FROM ubuntu:18.04
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

ARG COMPONENT_VERSION

# install package to build
RUN apt-get update && apt-get install -y \
    wget \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# install latest cmake
ENV CMAKE_VERSION ${COMPONENT_VERSION}
ENV CMAKE_FILE cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz
ENV CMAKE_URL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_FILE}
ENV CMAKE_URL_FILE ${CMAKE_FILE}
RUN wget ${CMAKE_URL} && tar -xzf ${CMAKE_FILE} \
 && rm ${CMAKE_FILE} \
 && mv ${CMAKE_FILE%.tar.gz} /opt \
 && ln -s /opt/${CMAKE_FILE%.tar.gz}/bin/* /usr/bin