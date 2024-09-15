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
elseif ($global:IsLinux)
{
    $os = "linux"
}
else
{
    Write-Host "[Error] This platform does not support CUDA" -ForegroundColor Red
    exit
}

# check CUDA
if (!($env:CUDA_PATH))
{
    Write-Host "CUDA_PATH environmental variable is missing" -ForegroundColor Red
    return
}
if (!(Test-Path($env:CUDA_PATH)))
{
    Write-Host "${env:CUDA_PATH} is missing" -ForegroundColor Red
    return
}

$target = "opencv4"
$shared = "static"
$sharedFlag = "OFF"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build-with-cudacodec | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install-with-cudacodec | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake
$contribDir = Join-Path $current opencv_contrib | `
              Join-Path -ChildPath modules

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D WITH_CUDA=OFF `
          -D BUILD_PROTOBUF=ON `
          -D BUILD_JPEG=ON `
          -D BUILD_PACKAGE=ON `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=ON `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_PNG=ON `
          -D WITH_TIFF=ON `
          -D WITH_OPENEXR=OFF `
          -D WITH_JASPER=OFF `
          -D WITH_FFMPEG=ON `
          -D WITH_MSMF=ON `
          -D WITH_MSMF_DXVA=ON `
          -D WITH_DSHOW=ON `
          -D OPENCV_EXTRA_MODULES_PATH="${contribDir}" `
          -D BUILD_opencv_cudacodec=ON `
          -D BUILD_opencv_cudev=ON `
          -D WITH_CUDA=ON `
          -D WITH_NVCUVID=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D WITH_CUDA=OFF `
          -D BUILD_PROTOBUF=ON `
          -D BUILD_JPEG=ON `
          -D BUILD_PACKAGE=ON `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=ON `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_PNG=ON `
          -D WITH_TIFF=ON `
          -D WITH_OPENEXR=OFF `
          -D WITH_JASPER=OFF `
          -D WITH_FFMPEG=ON `
          -D WITH_MFX=OFF `
          -D WITH_GSTREAMER=ON `
          -D OPENCV_EXTRA_MODULES_PATH="${contribDir}" `
          -D BUILD_opencv_cudacodec=ON `
          -D BUILD_opencv_cudev=ON `
          -D WITH_CUDA=ON `
          -D WITH_NVCUVID=ON `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location