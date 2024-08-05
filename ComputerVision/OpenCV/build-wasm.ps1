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

# cv.imread is not a function
# https://github.com/opencv/opencv/issues/25057
# https://github.com/opencv/opencv/issues/21580
# https://github.com/opencv/opencv/issues/24620
# $emsdkVersion = "3.1.51"

# cv.imread is not a function
# $emsdkVersion = "3.0.1"

# https://github.com/opencv/opencv/pull/25084
# $emsdkVersion = "3.1.54"

# https://docs.opencv.org/4.x/d4/da1/tutorial_js_setup.html
# Mac is not supported
# cv.imread is not a function
# $emsdkVersion = "2.0.10"

# https://stackoverflow.com/questions/67190799/how-to-include-cv-imread-when-building-opencv-js
$emsdkVersion = "1.39.15"

# build
$sourceDir = Join-Path $current $target
$buildDirName = "build-wasm"
$buildDir = Join-Path $current ${buildDirName} | `
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


$dockerBuildDir = Join-Path ${buildDirName} $os | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $shared

if ($global:IsWindows)
{
    # # On Windows, we can not avoid `Compiler doesn't support baseline optimization flags`
    # Write-Host "[Error] For now, we can not support building binary on windows" -ForegroundColor Red
    # exit

    # # use submodule instead of system installed emscripten!!
    # Push-Location $emsdkDir | Out-Null
    # $emsdk = Join-Path $emsdkDir emsdk.bat
    # git pull
    # & "${emsdk}" install $emsdkVersion
    # $emsdk = Join-Path $emsdkDir emsdk.ps1
    # & "${emsdk}" activate $emsdkVersion
    # source ./emsdk_env.sh --build=$Configuration
    # Pop-Location

    # $env:EMSCRIPTEN = Join-Path $current emsdk | `
    #                   Join-Path -ChildPath upstream | `
    #                   Join-Path -ChildPath emscripten
    # $toolchainFile = Join-Path $env:EMSCRIPTEN cmake | `
    #                  Join-Path -ChildPath Modules | `
    #                  Join-Path -ChildPath Platform | `
    #                  Join-Path -ChildPath Emscripten.cmake
    # if (!(Test-Path($toolchainFile)))
    # {
    #     Write-Host "[Error] ${toolchainFile} is missing" -ForegroundColor Red
    #     exit
    # }

    # $pythonPath = (Get-Command python).Path
    
    # # * build_js.py can not override install prefix. So we specify argument manually...
    # # * https://github.com/opencv/opencv/issues/18657
    # #   Occur `Compiler doesn't support baseline optimization flags`
    # # On Windows, CV_DISABLE_OPTIMIZATION shall be specified..
    # $buildFlag = "-s WASM=1 -s USE_PTHREADS=0 "
    # $CV_DISABLE_OPTIMIZATION="ON"

    docker run --rm --workdir /project/${target} -v "${current}:/project" "emscripten/emsdk:${emsdkVersion}" emcmake python3 ./platforms/js/build_js.py "/project/${dockerBuildDir}"
}
elseif ($global:IsMacOS)
{
    # docker run --rm --workdir /project/${target} -v "${current}:/project" "emscripten/emsdk:${emsdkVersion}" emcmake python3 ./platforms/js/build_js.py "/project/${dockerBuildDir}"
}
elseif ($global:IsLinux)
{
}

# copy opencv.js
$installDir = Join-Path $installDir bin
New-Item -Type Directory $installDir -Force | Out-Null
Copy-Item (Join-Path $buildDir bin | Join-Path -ChildPath "*") $installDir -Force -Recurse | Out-Null