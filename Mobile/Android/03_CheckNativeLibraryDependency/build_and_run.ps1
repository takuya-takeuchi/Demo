$CURRENT_DIR = $PSScriptRoot

$CMAKE_VERSION="3.22.1"
$NDK_VERSION="26.1.10909125"
$BUILD_ABI="arm64-v8a"
$BUILD_API_LEVEL="21"
$BUILD_CONFIGURATION="Release"
$BINARY_NAME="hello"

if (!($env:ANDROID_SDK_ROOT))
{
    Write-Host "The environment variable 'ANDROID_SDK_ROOT' is not defined" -ForegroundColor Red
    return
}

if (!((Test-Path($env:ANDROID_SDK_ROOT))))
{
    Write-Host "The environment variable ANDROID_SDK_ROOT: '${env:ANDROID_SDK_ROOT}' is not exist" -ForegroundColor Red
    return
}

if (!($env:ANDROID_NDK_ROOT))
{
    Write-Host "The environment variable 'ANDROID_NDK_ROOT' is not defined" -ForegroundColor Red
    return
}

if (!((Test-Path($env:ANDROID_NDK_ROOT))))
{
    Write-Host "The environment variable ANDROID_NDK_ROOT: '${env:ANDROID_NDK_ROOT}' is not exist" -ForegroundColor Red
    return
}

$ANDROID_CMAKE_DIR = Join-Path ${env:ANDROID_SDK_ROOT} cmake | `
                     Join-Path -ChildPath $CMAKE_VERSION | `
                     Join-Path -ChildPath bin
if (!((Test-Path($ANDROID_CMAKE_DIR))))
{
    Write-Host "The variable ANDROID_CMAKE_DIR: '${ANDROID_CMAKE_DIR}' is not exist" -ForegroundColor Red
    return
}

if ($global:IsWindows)
{
    $CMAKE_BINARY = "cmake.exe"
    $NINJA_BINARY = "ninja.exe"
    $ADB_BINARY = "adb.exe"
}
else
{
    $CMAKE_BINARY = "cmake"
    $NINJA_BINARY = "ninja"
    $ADB_BINARY = "adb"
}

$ANDROID_CMAKE_PATH = Join-Path ${ANDROID_CMAKE_DIR} $CMAKE_BINARY
if (!((Test-Path($ANDROID_CMAKE_PATH))))
{
    Write-Host "The variable ANDROID_CMAKE_PATH: '${ANDROID_CMAKE_PATH}' is not exist" -ForegroundColor Red
    return
}

$CMAKE_MAKE_PROGRAM = Join-Path ${ANDROID_CMAKE_DIR} $NINJA_BINARY
if (!((Test-Path($CMAKE_MAKE_PROGRAM))))
{
    Write-Host "The variable CMAKE_MAKE_PROGRAM: '${CMAKE_MAKE_PROGRAM}' is not exist" -ForegroundColor Red
    return
}

$CMAKE_TOOLCHAIN_FILE = Join-Path ${env:ANDROID_NDK_ROOT} build | `
                        Join-Path -ChildPath cmake | `
                        Join-Path -ChildPath android.toolchain.cmake
if (!((Test-Path($CMAKE_TOOLCHAIN_FILE))))
{
    Write-Host "The variable CMAKE_TOOLCHAIN_FILE: '${CMAKE_TOOLCHAIN_FILE}' is not exist" -ForegroundColor Red
    return
}

$ADB = Join-Path ${env:ANDROID_SDK_ROOT} platform-tools | `
       Join-Path -ChildPath $ADB_BINARY
if (!((Test-Path($ADB))))
{
    Write-Host "The variable ADB: '${ADB}' is not exist" -ForegroundColor Red
    return
}

Write-Host "    ANDROID_SDK_ROOT: ${env:ANDROID_SDK_ROOT}" -ForegroundColor Green
Write-Host "    ANDROID_NDK_ROOT: ${env:ANDROID_NDK_ROOT}" -ForegroundColor Green
Write-Host "  ANDROID_CMAKE_PATH: ${ANDROID_CMAKE_PATH}" -ForegroundColor Green
Write-Host "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}" -ForegroundColor Green
Write-Host "                 ADB: ${ADB}" -ForegroundColor Green

Write-Host "Build OpenCV..." -ForegroundColor Green

$SOURCE_DIR = Join-Path $CURRENT_DIR opencv 
$BUILD_DIR = Join-Path $CURRENT_DIR build | `
             Join-Path -ChildPath opencv | `
             Join-Path -ChildPath $BUILD_ABI | `
             Join-Path -ChildPath $BUILD_API_LEVEL | `
             Join-Path -ChildPath $BUILD_CONFIGURATION
$INSTALL_OPENCV_DIR = Join-Path $CURRENT_DIR install | `
                      Join-Path -ChildPath opencv | `
                      Join-Path -ChildPath $BUILD_ABI | `
                      Join-Path -ChildPath $BUILD_API_LEVEL | `
                      Join-Path -ChildPath $BUILD_CONFIGURATION
New-Item -Type Directory $BUILD_DIR -Force | Out-Null
New-Item -Type Directory $INSTALL_OPENCV_DIR -Force | Out-Null

Push-Location $BUILD_DIR | Out-Null
    & "${ANDROID_CMAKE_PATH}" -G Ninja `
                              -D BUILD_SHARED_LIBS=OFF `
                              -D BUILD_FAT_JAVA_LIB=OFF `
                              -D CMAKE_INSTALL_PREFIX="${INSTALL_OPENCV_DIR}" `
                              -D CMAKE_SYSTEM_NAME=Android `
                              -D CMAKE_EXPORT_COMPILE_COMMANDS=ON `
                              -D CMAKE_SYSTEM_VERSION=$BUILD_API_LEVEL `
                              -D ANDROID_PLATFORM="android-${BUILD_API_LEVEL}" `
                              -D ANDROID_ABI=$BUILD_ABI `
                              -D CMAKE_ANDROID_ARCH_ABI=$BUILD_ABI `
                              -D ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" `
                              -D CMAKE_MAKE_PROGRAM="${CMAKE_MAKE_PROGRAM}" `
                              -D CMAKE_CXX_FLAGS=-std=c++14 `
                              -D CMAKE_LIBRARY_OUTPUT_DIRECTORY=lib `
                              -D CMAKE_RUNTIME_OUTPUT_DIRECTORY=bin `
                              -D CMAKE_BUILD_TYPE=$BUILD_CONFIGURATION `
                              -D ANDROID_STL=c++_static `
                              -D BUILD_opencv_world=OFF `
                              -D BUILD_PERF_TESTS=OFF `
                              -D BUILD_TESTS=OFF `
                              -D BUILD_DOCS=OFF `
                              -D BUILD_opencv_androidcamera=OFF `
                              -D BUILD_opencv_calib3d=OFF `
                              -D BUILD_opencv_contrib=OFF `
                              -D BUILD_opencv_core=ON `
                              -D BUILD_opencv_dnn=OFF `
                              -D BUILD_opencv_dynamicuda=OFF `
                              -D BUILD_opencv_features2d=OFF `
                              -D BUILD_opencv_flann=OFF `
                              -D BUILD_opencv_gapi=OFF `
                              -D BUILD_opencv_gpu=OFF `
                              -D BUILD_opencv_highgui=OFF `
                              -D BUILD_opencv_imgcodecs=ON `
                              -D BUILD_opencv_imgproc=ON `
                              -D BUILD_opencv_java_bindings_generator=OFF `
                              -D BUILD_opencv_java=OFF `
                              -D BUILD_opencv_js=OFF `
                              -D BUILD_opencv_lagacy=OFF `
                              -D BUILD_opencv_ml=OFF `
                              -D BUILD_opencv_nonfree=OFF `
                              -D BUILD_opencv_objdetect=OFF `
                              -D BUILD_opencv_ocl=OFF `
                              -D BUILD_opencv_photo=OFF `
                              -D BUILD_opencv_python_bindings_generator=OFF `
                              -D BUILD_opencv_python=OFF `
                              -D BUILD_opencv_python2=OFF `
                              -D BUILD_opencv_python3=OFF `
                              -D BUILD_opencv_shape=OFF `
                              -D BUILD_opencv_stitching=OFF `
                              -D BUILD_opencv_superres=OFF `
                              -D BUILD_opencv_ts=OFF `
                              -D BUILD_opencv_video=OFF `
                              -D BUILD_opencv_videoio=OFF `
                              -D BUILD_opencv_videostab=OFF `
                              -D BUILD_opencv_viz=OFF `
                              -D BUILD_opencv_apps=OFF `
                              -D BUILD_opencv_alphamat=OFF `
                              -D BUILD_opencv_aruco=OFF `
                              -D BUILD_opencv_barcode=OFF `
                              -D BUILD_opencv_bgsegm=OFF `
                              -D BUILD_opencv_bioinspired=OFF `
                              -D BUILD_opencv_ccalib=OFF `
                              -D BUILD_opencv_cnn_3dobj=OFF `
                              -D BUILD_opencv_cudaarithm=OFF `
                              -D BUILD_opencv_cudabgsegm=OFF `
                              -D BUILD_opencv_cudacodec=OFF `
                              -D BUILD_opencv_cudafeatures2d=OFF `
                              -D BUILD_opencv_cudafilters=OFF `
                              -D BUILD_opencv_cudaimgproc=OFF `
                              -D BUILD_opencv_cudalegacy=OFF `
                              -D BUILD_opencv_cudaobjdetect=OFF `
                              -D BUILD_opencv_cudaoptflow=OFF `
                              -D BUILD_opencv_cudastereo=OFF `
                              -D BUILD_opencv_cudawarping=OFF `
                              -D BUILD_opencv_cudev=OFF `
                              -D BUILD_opencv_cvv=OFF `
                              -D BUILD_opencv_datasets=OFF `
                              -D BUILD_opencv_dnn_objdetect=OFF `
                              -D BUILD_opencv_dnn_superres=OFF `
                              -D BUILD_opencv_dnns_easily_fooled=OFF `
                              -D BUILD_opencv_dpm=OFF `
                              -D BUILD_opencv_face=OFF `
                              -D BUILD_opencv_freetype=ON `
                              -D BUILD_opencv_fuzzy=OFF `
                              -D BUILD_opencv_hdf=OFF `
                              -D BUILD_opencv_hfs=OFF `
                              -D BUILD_opencv_img_hash=OFF `
                              -D BUILD_opencv_intensity_transform=OFF `
                              -D BUILD_opencv_julia=OFF `
                              -D BUILD_opencv_line_descriptor=OFF `
                              -D BUILD_opencv_matlab=OFF `
                              -D BUILD_opencv_mcc=OFF `
                              -D BUILD_opencv_optflow=OFF `
                              -D BUILD_opencv_ovis=OFF `
                              -D BUILD_opencv_phase_unwrapping=OFF `
                              -D BUILD_opencv_plot=OFF `
                              -D BUILD_opencv_quality=OFF `
                              -D BUILD_opencv_rapid=OFF `
                              -D BUILD_opencv_reg=OFF `
                              -D BUILD_opencv_rgbd=OFF `
                              -D BUILD_opencv_saliency=OFF `
                              -D BUILD_opencv_sfm=OFF `
                              -D BUILD_opencv_shape=OFF `
                              -D BUILD_opencv_stereo=OFF `
                              -D BUILD_opencv_structured_light=OFF `
                              -D BUILD_opencv_superres=OFF `
                              -D BUILD_opencv_surface_matching=OFF `
                              -D BUILD_opencv_text=OFF `
                              -D BUILD_opencv_tracking=OFF `
                              -D BUILD_opencv_videostab=OFF `
                              -D BUILD_opencv_viz=OFF `
                              -D BUILD_opencv_wechat_qrcode=OFF `
                              -D BUILD_opencv_xfeatures2d=OFF `
                              -D BUILD_opencv_ximgproc=OFF `
                              -D BUILD_opencv_xobjdetect=OFF `
                              -D BUILD_opencv_xphoto=OFF `
                              -D BUILD_JASPER=OFF `
                              -D BUILD_OPENJPEG=OFF `
                              -D BUILD_PROTOBUF=OFF `
                              -D BUILD_TBB=ON `
                              -D BUILD_TIFF=OFF `
                              -D BUILD_WEBP=OFF `
                              -D ENABLE_FAST_MATH=ON `
                              -D WITH_ADE=OFF `
                              -D WITH_CUDA=OFF `
                              -D WITH_EIGEN=OFF `
                              -D WITH_FFMPEG=OFF `
                              -D WITH_IPP=OFF `
                              -D WITH_JASPER=OFF `
                              -D WITH_OPENJPEG=OFF `
                              -D WITH_OPENMP=OFF `
                              -D WITH_PROTOBUF=OFF `
                              -D WITH_TBB=ON `
                              -D WITH_TIFF=OFF `
                              -D WITH_WEBP=OFF `
                              -D BUILD_ANDROID_EXAMPLES=OFF `
                              "${SOURCE_DIR}"
    & "${ANDROID_CMAKE_PATH}" --build "${BUILD_DIR}" --config $BUILD_CONFIGURATION --target install
Pop-Location

Write-Host "Build Wrapper..." -ForegroundColor Green

$SOURCE_DIR = Join-Path $CURRENT_DIR wrapper 
$BUILD_DIR = Join-Path $CURRENT_DIR build | `
             Join-Path -ChildPath wrapper | `
             Join-Path -ChildPath $BUILD_ABI | `
             Join-Path -ChildPath $BUILD_API_LEVEL | `
             Join-Path -ChildPath $BUILD_CONFIGURATION
$INSTALL_NATIVE_DIR = Join-Path $CURRENT_DIR install | `
                      Join-Path -ChildPath wrapper | `
                      Join-Path -ChildPath $BUILD_ABI | `
                      Join-Path -ChildPath $BUILD_API_LEVEL | `
                      Join-Path -ChildPath $BUILD_CONFIGURATION
New-Item -Type Directory $BUILD_DIR -Force | Out-Null
New-Item -Type Directory $INSTALL_NATIVE_DIR -Force | Out-Null

Push-Location $BUILD_DIR | Out-Null
    & "${ANDROID_CMAKE_PATH}" -G Ninja `
                              -D CMAKE_INSTALL_PREFIX="${INSTALL_NATIVE_DIR}" `
                              -D CMAKE_SYSTEM_NAME=Android `
                              -D CMAKE_EXPORT_COMPILE_COMMANDS=ON `
                              -D CMAKE_SYSTEM_VERSION=$BUILD_API_LEVEL `
                              -D ANDROID_PLATFORM="android-${BUILD_API_LEVEL}" `
                              -D ANDROID_ABI=$BUILD_ABI `
                              -D CMAKE_ANDROID_ARCH_ABI=$BUILD_ABI `
                              -D ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" `
                              -D CMAKE_MAKE_PROGRAM="${CMAKE_MAKE_PROGRAM}" `
                              -D CMAKE_CXX_FLAGS=-std=c++14 `
                              -D CMAKE_LIBRARY_OUTPUT_DIRECTORY=lib `
                              -D CMAKE_RUNTIME_OUTPUT_DIRECTORY=bin `
                              -D CMAKE_BUILD_TYPE=$BUILD_CONFIGURATION `
                              -D ANDROID_STL=c++_static `
                              -D OpenCV_DIR="${INSTALL_OPENCV_DIR}/sdk/native/jni/abi-arm64-v8a/" `
                              "${SOURCE_DIR}"
    & "${ANDROID_CMAKE_PATH}" --build "${BUILD_DIR}" --config $BUILD_CONFIGURATION --target install
Pop-Location

Write-Host "Build Main..." -ForegroundColor Green

$SOURCE_DIR = $CURRENT_DIR
$BUILD_DIR = Join-Path $CURRENT_DIR build | `
             Join-Path -ChildPath main | `
             Join-Path -ChildPath $BUILD_ABI | `
             Join-Path -ChildPath $BUILD_API_LEVEL | `
             Join-Path -ChildPath $BUILD_CONFIGURATION
New-Item -Type Directory $BUILD_DIR -Force | Out-Null

Push-Location $BUILD_DIR | Out-Null
    & "${ANDROID_CMAKE_PATH}" -G Ninja `
                              -D CMAKE_SYSTEM_NAME=Android `
                              -D CMAKE_EXPORT_COMPILE_COMMANDS=ON `
                              -D CMAKE_SYSTEM_VERSION=$BUILD_API_LEVEL `
                              -D ANDROID_PLATFORM="android-${BUILD_API_LEVEL}" `
                              -D ANDROID_ABI=$BUILD_ABI `
                              -D CMAKE_ANDROID_ARCH_ABI=$BUILD_ABI `
                              -D ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                              -D CMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" `
                              -D CMAKE_MAKE_PROGRAM="${CMAKE_MAKE_PROGRAM}" `
                              -D CMAKE_CXX_FLAGS=-std=c++14 `
                              -D CMAKE_LIBRARY_OUTPUT_DIRECTORY=lib `
                              -D CMAKE_RUNTIME_OUTPUT_DIRECTORY=bin `
                              -D CMAKE_BUILD_TYPE=$BUILD_CONFIGURATION `
                              -D ANDROID_STL=c++_static `
                              -D Native_DIR="${INSTALL_NATIVE_DIR}" `
                              "${SOURCE_DIR}"
    & "${ANDROID_CMAKE_PATH}" --build "${BUILD_DIR}" --config $BUILD_CONFIGURATION --target install
Pop-Location

$PROGRAM = Join-Path ${BUILD_DIR} bin | `
           Join-Path -ChildPath $BINARY_NAME
& "${ADB}" push "${PROGRAM}" /data/local/tmp/
& "${ADB}" push "${INSTALL_NATIVE_DIR}/lib/libwrapper.so" /data/local/tmp/
& "${ADB}" shell chmod 775 /data/local/tmp/hello
& "${ADB}" shell LD_LIBRARY_PATH=/data/local/tmp LD_DEBUG=3 /data/local/tmp/hello