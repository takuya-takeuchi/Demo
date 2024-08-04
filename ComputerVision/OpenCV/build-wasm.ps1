#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration
)

$current = $PSScriptRoot

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "opencv4"
$shared = "static"
$sharedFlag = "OFF"
# --memory-init-file is not supported 3.1.55 or later
# This issue is fixed by https://github.com/opencv/opencv/pull/25629
# $emsdkVersion = "3.1.64"
$emsdkVersion = "3.1.51"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build-wasm | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install-wasm | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$emsdkDir = Join-Path $current emsdk
$buildPythonScript = Join-Path $target platform | `
                     Join-Path -ChildPath js | `
                     Join-Path -ChildPath build_js.py

git submodule update --init --recursive .

if (!(Test-Path($emsdkDir)))
{
    Write-Host "[Error] ${emsdkDir} is missing" -ForegroundColor Red
    exit
}

Push-Location $buildDir
if ($global:IsWindows)
{
    # On Windows, we can not avoid `Compiler doesn't support baseline optimization flags`
    Write-Host "[Error] For now, we can not support building binary on windows" -ForegroundColor Red
    exit

    # use submodule instead of system installed emscripten!!
    Push-Location $emsdkDir | Out-Null
    $emsdk = Join-Path $emsdkDir emsdk.bat
    git pull
    & "${emsdk}" install $emsdkVersion
    $emsdk = Join-Path $emsdkDir emsdk.ps1
    & "${emsdk}" activate $emsdkVersion
    Pop-Location

    $env:EMSCRIPTEN = Join-Path $current emsdk | `
                      Join-Path -ChildPath upstream | `
                      Join-Path -ChildPath emscripten
    $toolchainFile = Join-Path $env:EMSCRIPTEN cmake | `
                     Join-Path -ChildPath Modules | `
                     Join-Path -ChildPath Platform | `
                     Join-Path -ChildPath Emscripten.cmake
    if (!(Test-Path($toolchainFile)))
    {
        Write-Host "[Error] ${toolchainFile} is missing" -ForegroundColor Red
        exit
    }

    $pythonPath = (Get-Command python).Path
    
    # * build_js.py can not override install prefix. So we specify argument manually...
    # * https://github.com/opencv/opencv/issues/18657
    #   Occur `Compiler doesn't support baseline optimization flags`
    # On Windows, CV_DISABLE_OPTIMIZATION shall be specified..
    $buildFlag = "-s WASM=1 -s USE_PTHREADS=0 "
    $CV_DISABLE_OPTIMIZATION="ON"
}
elseif ($global:IsMacOS)
{
    # use submodule instead of system installed emscripten!!
    Push-Location $emsdkDir | Out-Null
    $emsdk = Join-Path $emsdkDir emsdk
    git pull
    & "${emsdk}" install $emsdkVersion
    & "${emsdk}" activate $emsdkVersion
    Pop-Location

    $env:EMSCRIPTEN = Join-Path $current emsdk | `
                      Join-Path -ChildPath upstream | `
                      Join-Path -ChildPath emscripten
    $toolchainFile = Join-Path $env:EMSCRIPTEN cmake | `
                     Join-Path -ChildPath Modules | `
                     Join-Path -ChildPath Platform | `
                     Join-Path -ChildPath Emscripten.cmake
    if (!(Test-Path($toolchainFile)))
    {
        Write-Host "[Error] ${toolchainFile} is missing" -ForegroundColor Red
        exit
    }
    # # Supress 'The term 'python' is not recognized as a name of a cmdlet, function, script file, or executable program. Check the spelling of the name, or if a path was included, verify that the path is correct and try again.'
    alias python=python3
    $pythonPath = (Get-Command python3).Path

    # * build_js.py can not override install prefix. So we specify argument manually...
    # * https://github.com/opencv/opencv/issues/18657
    #   Occur `Compiler doesn't support baseline optimization flags`
    #   It supress by adding `-D CV_DISABLE_OPTIMIZATION=ON` or removing `-msimd128 -s USE_WEBNN=1 `
    # $buildFlag = "-s WASM=1 -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=4 -s DISABLE_EXCEPTION_CATCHING=0 "
    $buildFlag = "-s WASM=1 -s USE_PTHREADS=1 -s PTHREAD_POOL_SIZE=4 -s DISABLE_EXCEPTION_CATCHING=0 -s MODULARIZE=1 -s EXPORT_NAME='cv' "
    $CV_DISABLE_OPTIMIZATION="OFF"
}
elseif ($global:IsLinux)
{
}

$env:EMMAKEN_JUST_CONFIGURE = Join-Path $current $target | `
                              Join-Path -ChildPath platforms | `
                              Join-Path -ChildPath js | `
                              Join-Path -ChildPath opencv_js.config.py
# avoid `em++: error: --preload-file and --embed-file cannot be used with NODERAWFS which disables virtual filesystem`
$env:EMMAKEN_JUST_CONFIGURE=""
cmake -D PYTHON_DEFAULT_EXECUTABLE="${pythonPath}" `
      -D PYTHON_EXECUTABLE="${pythonPath}" `
      -D ENABLE_PIC=FALSE `
      -D CMAKE_BUILD_TYPE=$Configuration `
      -D CMAKE_TOOLCHAIN_FILE="${toolchainFile}" `
      -D CV_DISABLE_OPTIMIZATION="${CV_DISABLE_OPTIMIZATION}" `
      -D CPU_BASELINE="" `
      -D CMAKE_INSTALL_PREFIX="${installDir}" `
      -D CPU_DISPATCH='' `
      -D CV_TRACE=OFF `
      -D BUILD_SHARED_LIBS=OFF `
      -D WITH_1394=OFF `
      -D WITH_ADE=OFF `
      -D WITH_VTK=OFF `
      -D WITH_EIGEN=OFF `
      -D WITH_FFMPEG=OFF `
      -D WITH_GSTREAMER=OFF `
      -D WITH_GTK=OFF `
      -D WITH_GTK_2_X=OFF `
      -D WITH_IPP=OFF `
      -D WITH_JASPER=OFF `
      -D WITH_JPEG=OFF `
      -D WITH_WEBP=OFF `
      -D WITH_OPENEXR=OFF `
      -D WITH_OPENGL=OFF `
      -D WITH_OPENVX=OFF `
      -D WITH_OPENNI=OFF `
      -D WITH_OPENNI2=OFF `
      -D WITH_PNG=OFF `
      -D WITH_TBB=OFF `
      -D WITH_TIFF=OFF `
      -D WITH_V4L=OFF `
      -D WITH_OPENCL=OFF `
      -D WITH_OPENCL_SVM=OFF `
      -D WITH_OPENCLAMDFFT=OFF `
      -D WITH_OPENCLAMDBLAS=OFF `
      -D WITH_GPHOTO2=OFF `
      -D WITH_LAPACK=OFF `
      -D WITH_ITT=OFF `
      -D WITH_QUIRC=ON `
      -D BUILD_ZLIB=ON `
      -D BUILD_opencv_apps=OFF `
      -D BUILD_opencv_calib3d=ON `
      -D BUILD_opencv_dnn=ON `
      -D BUILD_opencv_features2d=ON `
      -D BUILD_opencv_flann=ON `
      -D BUILD_opencv_gapi=OFF `
      -D BUILD_opencv_ml=OFF `
      -D BUILD_opencv_photo=ON `
      -D BUILD_opencv_imgcodecs=OFF `
      -D BUILD_opencv_shape=OFF `
      -D BUILD_opencv_videoio=OFF `
      -D BUILD_opencv_videostab=OFF `
      -D BUILD_opencv_highgui=OFF `
      -D BUILD_opencv_superres=OFF `
      -D BUILD_opencv_stitching=OFF `
      -D BUILD_opencv_java=OFF `
      -D BUILD_opencv_js=ON `
      -D BUILD_opencv_python2=OFF `
      -D BUILD_opencv_python3=OFF `
      -D BUILD_EXAMPLES=ON `
      -D BUILD_PACKAGE=OFF `
      -D BUILD_TESTS=ON `
      -D BUILD_PERF_TESTS=ON `
      -D BUILD_DOCS=OFF `
      -D WITH_PTHREADS_PF=OFF `
      -D CV_ENABLE_INTRINSICS=OFF `
      -D BUILD_WASM_INTRIN_TESTS=OFF `
      -D WITH_WEBNN=OFF `
      -D CMAKE_C_FLAGS="${buildFlag}" `
      -D CMAKE_CXX_FLAGS="${buildFlag}" `
      "${sourceDir}"
cmake --build . --config ${Configuration} --target install
Pop-Location

# copy opencv.js
if ($global:IsWindows)
{
}
elseif ($global:IsMacOS)
{
    Copy-Item (Join-Path $buildDir bin | Join-Path -ChildPath opencv.js) (Join-Path $installDir bin | Join-Path -ChildPath opencv.js) -Force | Out-Null
}
elseif ($global:IsLinux)
{
}