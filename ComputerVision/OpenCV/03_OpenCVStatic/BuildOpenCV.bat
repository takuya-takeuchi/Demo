echo off

@set CONFIG=%1
@set CURRENT=%cd%
@set OPENCV=%2
@set BUILDDIR=build-opencv

@call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"

@mkdir %BUILDDIR%
@cd %BUILDDIR%
@cmake -G "NMake Makefiles" -D CMAKE_BUILD_TYPE=%CONFIG% ^
                            -D BUILD_SHARED_LIBS=OFF ^
                            -D BUILD_WITH_STATIC_CRT=OFF ^
                            -D CMAKE_INSTALL_PREFIX="%cd%/install" ^
                            -D BUILD_SHARED_LIBS=OFF ^
                            -D BUILD_opencv_world=ON ^
                            -D BUILD_opencv_java=OFF ^
                            -D BUILD_opencv_python=OFF ^
                            -D BUILD_opencv_python2=OFF ^
                            -D BUILD_opencv_python3=OFF ^
                            -D BUILD_PERF_TESTS=OFF ^
                            -D BUILD_TESTS=OFF ^
                            -D BUILD_DOCS=OFF ^
                            -D WITH_CUDA=OFF ^
                            -D BUILD_PROTOBUF=OFF ^
                            -D WITH_PROTOBUF=OFF ^
                            -D WITH_IPP=OFF ^
                            -D WITH_FFMPEG=OFF ^
                            "%OPENCV%"
@nmake
@nmake install