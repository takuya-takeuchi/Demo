FROM ubuntu:18.04
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    wget \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# cmake
ENV CMAKE_VERSION 3.21.3
ENV ARCH linux-x86_64
RUN wget -q https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-${ARCH}.sh && chmod +x cmake-${CMAKE_VERSION}-${ARCH}.sh
RUN echo y | ./cmake-${CMAKE_VERSION}-${ARCH}.sh
RUN mv cmake-${CMAKE_VERSION}-${ARCH} /opt && echo y | ln -s /opt/cmake-${CMAKE_VERSION}-${ARCH}/bin/* /usr/bin