echo off

@set CONFIG=%1
@set CURRENT=%cd%
@set BUILDDIR=build
@set OpenCV_DIR=%cd%\build-opencv

@call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"

@set BUILDDIR=build
@mkdir %BUILDDIR%
@cd %BUILDDIR%
@cmake -G "Visual Studio 15 2017" -A x64 ^
                                  -T host=x64 ^
                                  -D BUILD_SHARED_LIBS=ON ^
                                  -D USE_NCNN_VULKAN=ON ^
                                  -D USE_NCNN_OPENCV=OFF ^
                                  -D OPENCV_ROOT_DIR=D:\Works\Lib\OpenCV\3.4.1\sources ^
                                  ..
@cmake --build . --config %CONFIG%