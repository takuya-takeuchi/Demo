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

$sourceDir = Join-Path $current $target
$buildOpenCVDir = Join-Path $current build | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $shared
$installOpenCVDir = Join-Path $current install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $shared | `
                    Join-Path -ChildPath x64 | `
                    Join-Path -ChildPath vc17 | `
                    Join-Path -ChildPath staticlib
$targetOpenCVCMake = Join-Path $installOpenCVDir "OpenCVConfig.cmake"

if (!(Test-Path("${targetOpenCVCMake}")))
{
    # build opencv
    git submodule update --init --recursive .

    New-Item -Type Directory $buildOpenCVDir -Force | Out-Null
    New-Item -Type Directory $installOpenCVDir -Force | Out-Null

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
}

$visualStudioVersions = @{
    "Visual Studio 16 2019"="vs2019"
    "Visual Studio 17 2022"="vs2022"
}

foreach ($key in $h.Keys) {
	Write-Host $key
	Write-Host $h[$key]
}
foreach ($key in $visualStudioVersions.Keys)
{
    Write-Host "Build by ${key}" -ForegroundColor Blue
    $visualStudio = $visualStudioVersions[$key]
    
    $sourceDir = $current
    $buildDir = Join-Path $current build | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath program | `
                Join-Path -ChildPath $visualStudio
    $installDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath program | `
                  Join-Path -ChildPath $visualStudio

    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null

    Write-Host "Move to ${buildDir}" -ForegroundColor Blue
    Push-Location $buildDir
    cmake -G "${key}" -A x64 -T host=x64 `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${installOpenCVDir}" `
          $sourceDir
    cmake --build . --config ${configuration} --target install
    Pop-Location
}