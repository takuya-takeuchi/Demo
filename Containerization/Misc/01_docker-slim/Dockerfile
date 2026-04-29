FROM ubuntu:24.04 AS builder
LABEL maintainer="Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    libz3-dev \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# install latest cmake
ENV CMAKE_VERSION=3.31.12
RUN CMAKE_FILE=cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz \
 && CMAKE_URL=https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_FILE} \
 && wget ${CMAKE_URL} && tar -xzf ${CMAKE_FILE} \
 && rm ${CMAKE_FILE} \
 && mv ${CMAKE_FILE%.tar.gz} /opt/cmake \
 && ln -s /opt/cmake/bin/* /usr/bin

# build and install opencv
WORKDIR /opt/sources/opencv
ENV OPENCV_VERSION=4.13.0
RUN SRC_DIR=/opt/sources/opencv BUILD_DIR=/opt/sources/opencv/build INSTALL_DIR=/opt/packages/opencv \
    git clone -b ${OPENCV_VERSION} https://github.com/opencv/opencv . \
 && cmake -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
          -D CMAKE_BUILD_TYPE=Release \
          -D BUILD_SHARED_LIBS=ON \
          -D BUILD_LIST=core,gapi,videoio \
          -D BUILD_opencv_apps=OFF \
          -D BUILD_opencv_world=OFF \
          -D BUILD_opencv_java=OFF \
          -D BUILD_opencv_python=OFF \
          -D BUILD_opencv_python2=OFF \
          -D BUILD_opencv_python3=OFF \
          -D BUILD_DOCS=OFF \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_TIFF=ON \
          -D WITH_JASPER=OFF \
          -D WITH_IMGCODEC_HDR=OFF \
          -D WITH_IMGCODEC_PFM=OFF \
          -D WITH_IMGCODEC_PXM=OFF \
          -D WITH_IMGCODEC_SUNRASTER=OFF \
          -D WITH_OPENEXR=OFF \
          -D WITH_OPENJPEG=OFF \
          -D WITH_TIFF=ON \
          -D WITH_WEBP=OFF \
          -D WITH_DSHOW=OFF \
          -D WITH_FFMPEG=OFF \
          -D WITH_GSTREAMER=OFF \
          -D WITH_MSMF=OFF \
          -D WITH_OBSENSOR=OFF \
          -D WITH_VA=OFF \
          -D WITH_ITT=OFF \
          -D WITH_EIGEN=OFF \
          -D WITH_IPP=OFF \
          -D WITH_PROTOBUF=OFF \
          -D WITH_OPENCL=OFF \
          -D WITH_FLATBUFFERS=OFF \
          -B ${BUILD_DIR} \
          -D ${SRC_DIR} \
 && cmake --build ${BUILD_DIR} --config Release --parallel $nproc --target install

 # app
WORKDIR /opt/sources/app
ADD CMakeLists.txt .
ADD main.cpp .
RUN SRC_DIR=/opt/sources/app BUILD_DIR=/opt/sources/app/build INSTALL_DIR=/opt/packages/app \
    cmake -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
          -D CMAKE_BUILD_TYPE=Release \
          -B ${BUILD_DIR} \
          -D ${SRC_DIR} \
 && cmake --build ${BUILD_DIR} --config Release --parallel $nproc --target install

FROM ubuntu:24.04 AS runtime
 
RUN apt-get update && apt-get install -y --no-install-recommends \
    libz3-4 \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app
ADD *.tiff /app
COPY --from=builder /opt/packages/app/* ./opt/app
COPY --from=builder /opt/packages/opencv/lib/* /opt/opencv