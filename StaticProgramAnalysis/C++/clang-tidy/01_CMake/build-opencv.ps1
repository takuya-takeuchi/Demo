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
$copnfigPath = Join-Path $current "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json
$target = "opencv"
$version = $config.opencv.version
if ($config.opencv.shared)
{
    $shared = "dynamic"
    $sharedFlag = "ON"
}
else
{
    $shared = "static"
    $sharedFlag = "OFF"
}

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

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $shared | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $shared | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $current
git submodule update --init --recursive .
Pop-Location

Push-Location $sourceDir
git fetch --all --prune
git checkout $version
git submodule update --init --recursive .
Pop-Location

# apply patch
$patch = Join-Path $current patch |
         Join-Path -ChildPath $target |
         Join-Path -ChildPath $version |
         Join-Path -ChildPath $os
if (Test-Path($patch))
{
    Copy-Item -Recurse $patch/* $sourceDir -Force
}

Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vs = "C:\Program Files\Microsoft Visual Studio\2022"
        $path = "${vs}\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Professional\VC\Auxiliary\Build\vcvars64.bat"
        }
        if (!(Test-Path($path)))
        {
            $path = "${vs}\Community\VC\Auxiliary\Build\vcvars64.bat"
        }

        Write-Host "Use: ${path}" -ForegroundColor Green

        cmd.exe /c "call `"${path}`" && set > %temp%\vcvars.txt"
        Get-Content "${env:temp}\vcvars.txt" | Foreach-Object {
            if ($_ -match "^(.*?)=(.*)$") {
                Set-Content "env:\$($matches[1])" $matches[2]
            }
        }
    }
    CallVisualStudioDeveloperConsole

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
    )
}

# standard
$cmakeArgs += @(
    "-D BUILD_LIST=core,imgproc"
    "-D BUILD_opencv_apps=OFF"
    "-D BUILD_opencv_world=OFF"
    "-D BUILD_opencv_java=OFF"
    "-D BUILD_opencv_python=OFF"
    "-D BUILD_opencv_python2=OFF"
    "-D BUILD_opencv_python3=OFF"
    "-D BUILD_DOCS=OFF"
    "-D BUILD_EXAMPLES=OFF"
    "-D BUILD_PERF_TESTS=OFF"
    "-D BUILD_TESTS=OFF"
)

# image
$cmakeArgs += @(
    "-D WITH_JASPER=OFF"
    "-D WITH_IMGCODEC_HDR=OFF"
    "-D WITH_IMGCODEC_PFM=OFF"
    "-D WITH_IMGCODEC_PXM=OFF"
    "-D WITH_IMGCODEC_SUNRASTER=OFF"
    "-D WITH_OPENEXR=OFF"
    "-D WITH_OPENJPEG=OFF"
    "-D WITH_TIFF=OFF"
    "-D WITH_WEBP=OFF"
)

# video
$cmakeArgs += @(
    "-D WITH_DSHOW=OFF"
    "-D WITH_FFMPEG=OFF"
    "-D WITH_GSTREAMER=OFF"
    "-D WITH_MSMF=OFF"
    "-D WITH_OBSENSOR=OFF"
)

# threads
$cmakeArgs += @(
    "-D WITH_VA=OFF"
)

# trace
$cmakeArgs += @(
    "-D WITH_ITT=OFF"
)

# 3rd party
$cmakeArgs += @(
    "-D WITH_EIGEN=OFF"
    "-D WITH_IPP=OFF"
    "-D WITH_PROTOBUF=OFF"
    "-D WITH_OPENCL=OFF"
    "-D WITH_FLATBUFFERS=OFF"
)

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
cmake --build . --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location