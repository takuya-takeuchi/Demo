#!/bin/bash +x

### path setup
SCRIPT=$(readlink -f $0)
WD=`dirname $SCRIPT`
OPENCV_ROOT=${WD}/opencv

### ABI setup (for ndk 16)
#ANDROID_ABI=${ANDROID_ABI:-"armeabi-v7a with NEON"}
ANDROID_ABI=${ANDROID_ABI:-"arm64-v8a"}
#ANDROID_ABI=${ANDROID_ABI:-"x86"}
# ANDROID_ABI=${ANDROID_ABI:-"x86_64"}

BUILD_DIR=$OPENCV_ROOT/platforms/build_android
INSTALL_DIR=${WD}/android_opencv
N_JOBS=${N_JOBS:-4}

if [ "${ANDROID_ABI}" = "armeabi" ]; then
    API_LEVEL=19
else
    API_LEVEL=21
fi

# rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
      -DCMAKE_TOOLCHAIN_FILE="${ANDROID_NDK}/build/cmake/android.toolchain.cmake" \
      -DANDROID_NDK="${ANDROID_NDK}" \
      -DANDROID_NATIVE_API_LEVEL=${API_LEVEL} \
      -DANDROID_ABI="${ANDROID_ABI}" \
      -D WITH_CUDA=OFF \
      -D WITH_MATLAB=OFF \
      -D BUILD_ANDROID_EXAMPLES=OFF \
      -D BUILD_DOCS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PYTHON=OFF \
      -D BUILD_PYTHON3=OFF \
      -D BUILD_ANDROID_PROJECTS=OFF \
      -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}/opencv" \
      ../..

make -j${N_JOBS}

# rm and install install folder, and remove build folder
rm -rf "${INSTALL_DIR}/opencv"
make install/strip

cd "${WD}"
