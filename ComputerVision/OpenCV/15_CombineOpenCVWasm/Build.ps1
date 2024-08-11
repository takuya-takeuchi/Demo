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
# $emSdkVersion = "1.39.15"
$emSdkVersion = "2.0.22"

# Not work!!
# * https://github.com/atteneder/DracoUnity/issues/55
#   error: undefined symbol: _ZNSt3__212basic_stringIcNS_11char_traitsIcEENS_9allocatorIcEEEC1ERKS5_ (referenced by top-level compiled C/C++ code)
# * from 2.0.23, emscripten updated llvm components to llvm-12
#   Sedd https://github.com/emscripten-core/emscripten/blob/main/ChangeLog.md#2023---05262021
# $emSdkVersion = "2.0.23"
# $emSdkVersion = "2.0.24"
# $emSdkVersion = "2.0.25"
# $emSdkVersion = "2.0.26"
# $emSdkVersion = "2.0.27"
# $emSdkVersion = "2.0.28"
# $emSdkVersion = "2.0.29"
# $emSdkVersion = "2.0.30"
# $emSdkVersion = "2.0.31"
# $emSdkVersion = "2.0.32"
# $emSdkVersion = "2.0.33"
# $emSdkVersion = "2.0.34"

# Not work!!
# * wasm-ld: error: /project/install-wasm/1.39.15/win/opencv4/static/lib/libopencv_core.a(system.cpp.o): undefined symbol: std::__2::__basic_string_common<true>::__throw_length_error() const
# $emSdkVersion = "3.1.64"
# $emSdkVersion = "3.1.57"

# $opencvEmSdkVersion = "1.39.15"
$opencvEmSdkVersion = "2.0.15"

# build
$buildDirName = Join-Path "build" $os
$installDirName = Join-Path "install" $os
$buildDir = $buildDirName
$installDir = $installDirName
$rootDir = Split-Path $current -Parent

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

$currentDirName = Split-Path -Path $current -Leaf
$rootDir = Split-Path $current -Parent

$opencvRoot = Join-Path install-wasm $opencvEmSdkVersion | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath opencv4 | `
              Join-Path -ChildPath $shared

if ($global:IsWindows)
{
    $buildDirName = $buildDirName.Replace("`\", "/")
    $installDirName = $installDirName.Replace("`\", "/")
    $opencvRoot = $opencvRoot.Replace("`\", "/")
}

$sourceDir = "/project/${currentDirName}"
$buildCommand = "emcmake cmake -B ${sourceDir}/${buildDirName} -D CMAKE_INSTALL_PREFIX=${sourceDir}/${installDirName} -D CMAKE_BUILD_TYPE=${Configuration}"
$buildCommand += " ${sourceDir}"
$installCommand = "EMCC_DEBUG=0 VERBOSE=0 cmake --build ${sourceDir}/${buildDirName} --config $Configuration --target install"
docker run --rm --workdir "${sourceDir}" `
           -v "${rootDir}:/project" `
           -e OpenCV_DIR=/project/${opencvRoot} `
           "emscripten/emsdk:${emSdkVersion}" sh -c "${buildCommand} && ${installCommand}"