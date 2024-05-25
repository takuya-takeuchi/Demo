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

$target = "opencv"
$shared = "static"
$sharedFlag = "OFF"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    if (!($env:VCPKG_ROOT_DIR))
    {
        Write-Host "VCPKG_ROOT_DIR environmental variable is missing" -ForegroundColor Red
        return
    }

    $toolchain = "${env:VCPKG_ROOT_DIR}\scripts\buildsystems\vcpkg.cmake"
    if (!(Test-Path(${toolchain})))
    {
        Write-Host "${toolchain} is missing" -ForegroundColor Red
        return
    }

    $library_type = "x64-windows"
    $vcpkg_base_directory = "${env:VCPKG_ROOT_DIR}\installed\${library_type}"

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
          -D CMAKE_INSTALL_PREFIX="${installDir}" `
          -D BUILD_WITH_STATIC_CRT=OFF `
          -D BUILD_opencv_world=OFF `
          -D BUILD_opencv_java=OFF `
          -D BUILD_opencv_python=OFF `
          -D BUILD_opencv_python2=OFF `
          -D BUILD_opencv_python3=OFF `
          -D BUILD_PERF_TESTS=OFF `
          -D BUILD_TESTS=OFF `
          -D BUILD_DOCS=OFF `
          -D WITH_CUDA=OFF `
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_JPEG=ON `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PACKAGE=ON `
          -D BUILD_JASPER=OFF `
          -D BUILD_WEBP=OFF `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=OFF `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_OPENJPEG=OFF `
          -D WITH_JASPER=OFF `
          -D WITH_PNG=ON `
          -D WITH_WEBP=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_EIGEN=OFF `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_dnn=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_gapi=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
          $sourceDir
}
elseif ($global:IsMacOS)
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
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_JPEG=ON `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PACKAGE=ON `
          -D BUILD_JASPER=OFF `
          -D BUILD_WEBP=OFF `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=OFF `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_OPENJPEG=OFF `
          -D WITH_JASPER=OFF `
          -D WITH_PNG=ON `
          -D WITH_WEBP=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_EIGEN=OFF `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_dnn=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_gapi=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
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
          -D BUILD_PROTOBUF=OFF `
          -D BUILD_JPEG=ON `
          -D BUILD_OPENJPEG=OFF `
          -D BUILD_PACKAGE=ON `
          -D BUILD_JASPER=OFF `
          -D BUILD_WEBP=OFF `
          -D BUILD_PNG=ON `
          -D BUILD_TIFF=OFF `
          -D BUILD_ZLIB=ON `
          -D WITH_JPEG=ON `
          -D WITH_OPENJPEG=OFF `
          -D WITH_JASPER=OFF `
          -D WITH_PNG=ON `
          -D WITH_WEBP=OFF `
          -D WITH_TIFF=OFF `
          -D WITH_QUIRC=OFF `
          -D WITH_PROTOBUF=OFF `
          -D WITH_EIGEN=OFF `
          -D BUILD_opencv_calib3d=OFF `
          -D BUILD_opencv_dnn=OFF `
          -D BUILD_opencv_features2d=OFF `
          -D BUILD_opencv_flann=OFF `
          -D BUILD_opencv_gapi=ON `
          -D BUILD_opencv_highgui=OFF `
          -D BUILD_opencv_ml=OFF `
          -D BUILD_opencv_objdetect=OFF `
          -D BUILD_opencv_photo=OFF `
          -D BUILD_opencv_stitching=OFF `
          -D BUILD_opencv_video=OFF `
          -D BUILD_opencv_videoio=OFF `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location