FROM ubuntu:24.04 AS builder
LABEL maintainer="Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    wget \
    libz3-dev \
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

# build and install opencv
WORKDIR /opt/sources/opencv
ENV OPENCV_VERSION=4.13.0
RUN git clone -b ${OPENCV_VERSION} https://github.com/opencv/opencv . \
 && mkdir -p /opt/sources/opencv/build \
 && mkdir -p /opt/packages/opencv \
 && cmake -D CMAKE_INSTALL_PREFIX=/opt/packages/opencv \
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
          -B /opt/sources/opencv/build \
          -S /opt/sources/opencv \
 && cmake --build /opt/sources/opencv/build --config Release --parallel $nproc --target install

 # app
WORKDIR /opt/sources/app
ADD CMakeLists.txt .
ADD main.cpp .
RUN mkdir -p /opt/sources/app/build \
 && mkdir -p /opt/packages/app \
 && cmake -D CMAKE_INSTALL_PREFIX=/opt/packages/app \
          -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_PREFIX_PATH=/opt/packages/opencv \
          -D OpenCV_DIR=/opt/packages/opencv/lib/cmake/opencv4 \
          -B /opt/sources/app/build \
          -S /opt/sources/app \
 && cmake --build /opt/sources/app/build --config Release --parallel $nproc --target install

FROM ubuntu:24.04 AS runtime
LABEL maintainer="Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"
 
RUN apt-get update && apt-get install -y --no-install-recommends \
    libz3-4 \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app
ADD *.tiff /app
COPY --from=builder /opt/packages/app/* /app
COPY --from=builder /opt/packages/opencv/lib/* /opt/opencv
ENV LD_LIBRARY_PATH=/opt/opencv