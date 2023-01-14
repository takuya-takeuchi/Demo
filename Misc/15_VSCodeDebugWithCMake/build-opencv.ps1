$current = $PSScriptRoot

# update submodule
git submodule update --init --recursive .

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

# build opencv
$sourceDir = Join-Path $current opencv
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath opencv
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath opencv

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=$installDir `
      -D CMAKE_BUILD_TYPE=Debug `
      -D BUILD_SHARED_LIBS=OFF `
      -D BUILD_WITH_STATIC_CRT=OFF `
      -D BUILD_opencv_world=OFF `
      -D BUILD_opencv_java=OFF `
      -D BUILD_opencv_python=OFF `
      -D BUILD_opencv_python2=OFF `
      -D BUILD_opencv_python3=OFF `
      -D BUILD_PERF_TESTS=OFF `
      -D BUILD_TESTS=OFF `
      -D BUILD_DOCS=OFF `
      -D BUILD_opencv_core=ON `
      -D BUILD_opencv_highgui=ON `
      -D BUILD_opencv_imgcodecs=ON `
      -D BUILD_opencv_imgproc=ON `
      -D BUILD_opencv_calib3d=OFF `
      -D BUILD_opencv_features2d=OFF `
      -D BUILD_opencv_flann=OFF `
      -D BUILD_opencv_java_bindings_generator=OFF `
      -D BUILD_opencv_ml=OFF `
      -D BUILD_opencv_objdetect=OFF `
      -D BUILD_opencv_photo=OFF `
      -D BUILD_opencv_python_bindings_generator=OFF `
      -D BUILD_opencv_shape=OFF `
      -D BUILD_opencv_stitching=OFF `
      -D BUILD_opencv_superres=OFF `
      -D BUILD_opencv_video=OFF `
      -D BUILD_opencv_videoio=OFF `
      -D BUILD_opencv_videostab=OFF `
      -D WITH_CUDA=OFF `
      -D BUILD_PROTOBUF=OFF `
      -D WITH_PROTOBUF=OFF `
      -D WITH_IPP=OFF `
      -D WITH_FFMPEG=OFF `
      -D WITH_ITT=OFF `
      -D WITH_QUIRC=OFF `
      -D WITH_EIGEN=OFF `
      $sourceDir
cmake --build . --config Debug --target install
Pop-Location
