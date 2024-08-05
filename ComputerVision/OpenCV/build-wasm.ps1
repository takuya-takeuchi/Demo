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

Push-Location $buildDir


$dockerBuildDir = Join-Path ${buildDirName} $os | `
                  Join-Path -ChildPath $target | `
                  Join-Path -ChildPath $shared

# build
docker run --rm --workdir /project/${target} -v "${current}:/project" "emscripten/emsdk:${emsdkVersion}" emcmake python3 ./platforms/js/build_js.py "/project/${dockerBuildDir}"

# copy opencv.js
$installDir = Join-Path $installDir bin
New-Item -Type Directory $installDir -Force | Out-Null
Copy-Item (Join-Path $buildDir bin | Join-Path -ChildPath "*") $installDir -Force -Recurse | Out-Null