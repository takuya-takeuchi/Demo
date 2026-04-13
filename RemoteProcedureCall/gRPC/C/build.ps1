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
$target = "grpc"
$version = $config.gRPC.version
if ($config.gRPC.shared)
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

$protobufVersion = $config.protobuf.version
if ($config.protobuf.shared)
{
    $protobufShared = "dynamic"
}
else
{
    $protobufShared = "static"
}

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

$PROTOBUF_INSTALL_DIR = Join-Path $current install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath protobuf | `
                        Join-Path -ChildPath $protobufVersion | `
                        Join-Path -ChildPath $protobufShared | `
                        Join-Path -ChildPath $Configuration
$PROTOBUF_CMAKE_DIR = Join-Path $PROTOBUF_INSTALL_DIR lib | `
                      Join-Path -ChildPath cmake | `
                      Join-Path -ChildPath protobuf
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

$paths = @(
    "${PROTOBUF_CMAKE_DIR}"
    "${ABSL_CMAKE_DIR}"
    "${ZLIB_CMAKE_DIR}"
)
foreach ($path in $paths)
{
    if (!(Test-Path($path)))
    {
        Write-Host "${path} is missing" -ForegroundColor Red
        exit
    }        
}

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

# apply patch
$patch = Join-Path $current patch |
         Join-Path -ChildPath grpc |
         Join-Path -ChildPath $version |
         Join-Path -ChildPath $os
if (Test-Path($patch))
{
    Copy-Item -Recurse $patch/* $sourceDir -Force
}

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
        $gRPC_MSVC_STATIC_RUNTIME = "ON"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
        $gRPC_MSVC_STATIC_RUNTIME = "OFF"
    }

    $cmakeArgs = @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR};${ABSL_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_CXX_STANDARD=17"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D gRPC_MSVC_STATIC_RUNTIME=${gRPC_MSVC_STATIC_RUNTIME}"
        "-D gRPC_BUILD_GRPC_CPP_PLUGIN=ON"
        "-D gRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF"
        "-D gRPC_BUILD_GRPC_NODE_PLUGIN=OFF"
        "-D gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF"
        "-D gRPC_BUILD_GRPC_PHP_PLUGIN=OFF"
        "-D gRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF"
        "-D gRPC_BUILD_GRPC_RUBY_PLUGIN=OFF"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_ABSL_PROVIDER=package"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "-D gRPC_ZLIB_PROVIDER=package"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
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
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR};${ABSL_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_CXX_STANDARD=20"
        "-D ZLIB_ROOT=${ZLIB_INSTALL_DIR}"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_ABSL_PROVIDER=package"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "-D gRPC_ZLIB_PROVIDER=package"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
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
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR};${ABSL_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_CXX_STANDARD=17"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_ABSL_PROVIDER=package"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "-D gRPC_ZLIB_PROVIDER=package"
        "-D absl_DIR=${ABSL_CMAKE_DIR}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
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