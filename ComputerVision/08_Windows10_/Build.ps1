$currentDir = $PSScriptRoot
$sourceDir = Join-Path $currentDir sources

$archs = @(
    "x64",
    "Win32"
)

foreach ($arch in $archs)
{
    $spdlogSourceDir = Join-Path $sourceDir spdlog
    $spdlogBuildDir = Join-Path $spdlogSourceDir build | `
                        Join-Path -ChildPath $arch
    $spdlogInstallDir = Join-Path $spdlogSourceDir installs | `
                          Join-Path -ChildPath $arch
    
    New-Item -Type Directory $spdlogBuildDir -Force | Out-Null
    New-Item -Type Directory  $spdlogInstallDir -Force | Out-Null
    
    Push-Location $spdlogBuildDir
    cmake -G "Visual Studio 17 2022" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${spdlogInstallDir}" `
          "${spdlogSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location

    $zxingCppSourceDir = Join-Path $sourceDir zxing-cpp
    $zxingCppBuildDir = Join-Path $zxingCppSourceDir build | `
                        Join-Path -ChildPath $arch
    $zxingCppInstallDir = Join-Path $zxingCppSourceDir install | `
                          Join-Path -ChildPath $arch
    
    New-Item -Type Directory $zxingCppBuildDir -Force | Out-Null
    New-Item -Type Directory  $zxingCppInstallDir -Force | Out-Null
    
    Push-Location $zxingCppBuildDir
    cmake -G "Visual Studio 17 2022" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${zxingCppInstallDir}" `
          -D BUILD_EXAMPLES=OFF `
          "${zxingCppSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location
    
    $opencvSourceDir = Join-Path $sourceDir opencv
    $opencvBuildDir = Join-Path $opencvSourceDir build | `
                      Join-Path -ChildPath $arch
    $opencvInstallDir = Join-Path $opencvSourceDir install | `
                        Join-Path -ChildPath $arch
    
    New-Item -Type Directory $opencvBuildDir -Force | Out-Null
    New-Item -Type Directory  $opencvInstallDir -Force | Out-Null
    
    Push-Location $opencvBuildDir
    cmake -G "Visual Studio 17 2022" -A $arch `
          -D CMAKE_BUILD_TYPE=Release `
          -D CMAKE_INSTALL_PREFIX="${opencvInstallDir}" `
          -D BUILD_opencv_world=ON `
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
          -D WITH_IPP=ON `
          -D WITH_FFMPEG=OFF `
          -D WITH_ADE=OFF `
          "${opencvSourceDir}"
    cmake --build . --config Release --target install
    Pop-Location
}