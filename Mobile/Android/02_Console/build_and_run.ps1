$CURRENT_DIR = $PSScriptRoot

$CMAKE_VERSION="3.22.1"
$NDK_VERSION="26.1.10909125"
$BUILD_ABI="arm64-v8a"
$BUILD_API_LEVEL="21"
$BUILD_CONFIGURATION="Release"
$BINARY_NAME="hello"

if (!($env:ANDROID_SDK_ROOT))
{
    Write-Host "The environment variable 'ANDROID_SDK_ROOT' is not defined" -ForegroundColor Red
    return
}

if (!((Test-Path($env:ANDROID_SDK_ROOT))))
{
    Write-Host "The environment variable ANDROID_SDK_ROOT: '${env:ANDROID_SDK_ROOT}' is not exist" -ForegroundColor Red
    return
}

if (!($env:ANDROID_NDK_ROOT))
{
    Write-Host "The environment variable 'ANDROID_NDK_ROOT' is not defined" -ForegroundColor Red
    return
}

if (!((Test-Path($env:ANDROID_NDK_ROOT))))
{
    Write-Host "The environment variable ANDROID_NDK_ROOT: '${env:ANDROID_NDK_ROOT}' is not exist" -ForegroundColor Red
    return
}

$ANDROID_CMAKE_DIR = Join-Path ${env:ANDROID_SDK_ROOT} cmake | `
                     Join-Path -ChildPath $CMAKE_VERSION | `
                     Join-Path -ChildPath bin
if (!((Test-Path($ANDROID_CMAKE_DIR))))
{
    Write-Host "The variable ANDROID_CMAKE_DIR: '${ANDROID_CMAKE_DIR}' is not exist" -ForegroundColor Red
    return
}

if ($global:IsWindows)
{
    $CMAKE_BINARY = "cmake.exe"
    $NINJA_BINARY = "ninja.exe"
    $ADB_BINARY = "adb.exe"
}
else
{
    $CMAKE_BINARY = "cmake"
    $NINJA_BINARY = "ninja"
    $ADB_BINARY = "adb"
}

$ANDROID_CMAKE_PATH = Join-Path ${ANDROID_CMAKE_DIR} $CMAKE_BINARY
if (!((Test-Path($ANDROID_CMAKE_PATH))))
{
    Write-Host "The variable ANDROID_CMAKE_PATH: '${ANDROID_CMAKE_PATH}' is not exist" -ForegroundColor Red
    return
}

$CMAKE_MAKE_PROGRAM = Join-Path ${ANDROID_CMAKE_DIR} $NINJA_BINARY
if (!((Test-Path($CMAKE_MAKE_PROGRAM))))
{
    Write-Host "The variable CMAKE_MAKE_PROGRAM: '${CMAKE_MAKE_PROGRAM}' is not exist" -ForegroundColor Red
    return
}

$CMAKE_TOOLCHAIN_FILE = Join-Path ${env:ANDROID_NDK_ROOT} build | `
                        Join-Path -ChildPath cmake | `
                        Join-Path -ChildPath android.toolchain.cmake
if (!((Test-Path($CMAKE_TOOLCHAIN_FILE))))
{
    Write-Host "The variable CMAKE_TOOLCHAIN_FILE: '${CMAKE_TOOLCHAIN_FILE}' is not exist" -ForegroundColor Red
    return
}

$ADB = Join-Path ${env:ANDROID_SDK_ROOT} platform-tools | `
       Join-Path -ChildPath $ADB_BINARY
if (!((Test-Path($ADB))))
{
    Write-Host "The variable ADB: '${ADB}' is not exist" -ForegroundColor Red
    return
}

Write-Host "    ANDROID_SDK_ROOT: ${env:ANDROID_SDK_ROOT}" -ForegroundColor Green
Write-Host "    ANDROID_NDK_ROOT: ${env:ANDROID_NDK_ROOT}" -ForegroundColor Green
Write-Host "  ANDROID_CMAKE_PATH: ${ANDROID_CMAKE_PATH}" -ForegroundColor Green
Write-Host "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}" -ForegroundColor Green
Write-Host "                 ADB: ${ADB}" -ForegroundColor Green

$BUILD_DIR = Join-Path $CURRENT_DIR build | `
             Join-Path -ChildPath $BUILD_ABI | `
             Join-Path -ChildPath $BUILD_API_LEVEL | `
             Join-Path -ChildPath $BUILD_CONFIGURATION

& "${ANDROID_CMAKE_PATH}" -G Ninja `
                          -D CMAKE_SYSTEM_NAME=Android `
                          -D CMAKE_EXPORT_COMPILE_COMMANDS=ON `
                          -D CMAKE_SYSTEM_VERSION=$BUILD_API_LEVEL `
                          -D ANDROID_PLATFORM="android-${BUILD_API_LEVEL}" `
                          -D ANDROID_ABI=$BUILD_ABI `
                          -D CMAKE_ANDROID_ARCH_ABI=$BUILD_ABI `
                          -D ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                          -D CMAKE_ANDROID_NDK="${env:ANDROID_NDK_ROOT}" `
                          -D CMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" `
                          -D CMAKE_MAKE_PROGRAM="${CMAKE_MAKE_PROGRAM}" `
                          -D CMAKE_CXX_FLAGS=-std=c++14 `
                          -D CMAKE_LIBRARY_OUTPUT_DIRECTORY=lib `
                          -D CMAKE_RUNTIME_OUTPUT_DIRECTORY=bin `
                          -D CMAKE_BUILD_TYPE=$BUILD_CONFIGURATION `
                          -D ANDROID_STL=c++_static `
                          -B "${BUILD_DIR}"
& "${ANDROID_CMAKE_PATH}" --build "${BUILD_DIR}"

$PROGRAM = Join-Path ${BUILD_DIR} bin | `
           Join-Path -ChildPath $BINARY_NAME
& "${ADB}" push "${PROGRAM}" /data/local/tmp/
& "${ADB}" shell chmod 775 /data/local/tmp/hello
& "${ADB}" shell /data/local/tmp/hello
& "${ADB}" shell rm -rf /data/local/tmp/hello