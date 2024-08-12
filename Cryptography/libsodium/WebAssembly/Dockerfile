ARG EMSDK_VERSION
FROM emscripten/emsdk:${EMSDK_VERSION}
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

# install package to build
RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    libtool \
 && apt-get clean && rm -rf /var/lib/apt/lists/*