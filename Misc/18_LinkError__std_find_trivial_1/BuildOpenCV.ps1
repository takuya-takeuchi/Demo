#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#***************************************
Param
(
)

$current = $PSScriptRoot

$configuration = "Release"

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
else
{
    Write-Host "This platfor is not supported"
}

$target = "opencv"
$shared = "static"
$sharedFlag = "OFF"

git submodule update --init --recursive .

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
cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
      -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D BUILD_WITH_STATIC_CRT=ON `
      -D CMAKE_BUILD_TYPE=$configuration `
      -D BUILD_SHARED_LIBS=${sharedFlag} `
      -D CMAKE_INSTALL_PREFIX="${installDir}" `
      -D BUILD_opencv_world=ON `
      -D BUILD_opencv_java=OFF `
      -D BUILD_opencv_python=OFF `
      -D BUILD_opencv_python2=OFF `
      -D BUILD_opencv_python3=OFF `
      -D BUILD_PERF_TESTS=OFF `
      -D BUILD_TESTS=OFF `
      -D BUILD_DOCS=OFF `
      -D WITH_CUDA=OFF `
      -D WITH_JPEG=OFF `
      -D WITH_PNG=OFF `
      -D WITH_TIFF=OFF `
      -D WITH_OPENJPEG=OFF `
      -D WITH_JASPER=OFF `
      -D WITH_OPENEXR=OFF `
      $sourceDir
cmake --build . --config ${configuration} --target install
Pop-Location