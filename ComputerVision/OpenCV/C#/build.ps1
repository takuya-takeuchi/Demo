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
$tagPostfix = "with-gstreamer"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared
$targetDir = Join-Path $installDir $target | `
             Join-Path -ChildPath lib | `
             Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
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
          $sourceDir
    cmake --build . --config ${Configuration} --target install
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D BUILD_SHARED_LIBS=${sharedFlag} `
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
          $sourceDir
    cmake --build . --config ${Configuration} --target install
}
elseif ($global:IsLinux)
{
    docker build --force-rm -f docker/ubuntu20-dotnet6-opencv/Dockerfile -t "ubuntu20-dotnet6-opencv-${tagPostfix}"
}
Pop-Location