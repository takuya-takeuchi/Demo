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
$target = "protobuf"
$version = $config.protobuf.version
if ($config.protobuf.shared)
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

$abslVersion = $config.abseil_cpp.version
if ($config.abseil_cpp.shared)
{
    $abslShared = "dynamic"
}
else
{
    $abslShared = "static"
}

$zlibVersion = $config.zlib.version
if ($config.zlib.shared)
{
    $zlibShared = "dynamic"
}
else
{
    $zlibShared = "static"
}

$ABSL_INSTALL_DIR = Join-Path $current install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath abseil-cpp | `
                    Join-Path -ChildPath $abslVersion | `
                    Join-Path -ChildPath $abslShared | `
                    Join-Path -ChildPath $Configuration
$ABSL_CMAKE_DIR = Join-Path $ABSL_INSTALL_DIR lib | `
                  Join-Path -ChildPath cmake | `
                  Join-Path -ChildPath absl
$ZLIB_INSTALL_DIR = Join-Path $current install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath zlib | `
                    Join-Path -ChildPath $zlibVersion | `
                    Join-Path -ChildPath $zlibShared | `
                    Join-Path -ChildPath $Configuration
$ZLIB_CMAKE_DIR = Join-Path $ZLIB_INSTALL_DIR lib | `
                  Join-Path -ChildPath cmake | `
                  Join-Path -ChildPath zlib

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $current
git submodule update --init --recursive .
Pop-Location

Push-Location $target
git fetch -ap
git checkout $version
git submodule update --init --recursive .
Pop-Location

Push-Location $buildDir

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

    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${ABSL_INSTALL_DIR}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D CMAKE_CXX_STANDARD=17"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D protobuf_BUILD_TESTS=OFF"
        "-D ZLIB_ROOT=${ZLIB_INSTALL_DIR}"
        "-D ZLIB_LIBRARY_RELEASE=${ZLIB_INSTALL_DIR}/lib/zs.lib"
        "-D ZLIB_LIBRARY_DEBUG=${ZLIB_INSTALL_DIR}/lib/zsd.lib"
        "${sourceDir}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${ABSL_INSTALL_DIR}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_CXX_STANDARD=20"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D protobuf_BUILD_TESTS=OFF"
        "-D ZLIB_ROOT=${ZLIB_INSTALL_DIR}"
        "-D ZLIB_LIBRARY_RELEASE=${ZLIB_INSTALL_DIR}/lib/libz.a"
        "-D ZLIB_LIBRARY_DEBUG=${ZLIB_INSTALL_DIR}/lib/libz.a"
        "${sourceDir}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_PREFIX_PATH=${ABSL_INSTALL_DIR}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_CXX_STANDARD=17"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D protobuf_BUILD_TESTS=OFF"
        "-D ZLIB_ROOT=${ZLIB_INSTALL_DIR}"
        "-D ZLIB_LIBRARY_RELEASE=${ZLIB_INSTALL_DIR}/lib/libz.a"
        "-D ZLIB_LIBRARY_DEBUG=${ZLIB_INSTALL_DIR}/lib/libz.a"
        "${sourceDir}"
    )
}

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
cmake --build . --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location