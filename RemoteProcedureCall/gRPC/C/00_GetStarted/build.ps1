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
$rootDir = Split-Path $current -Parent
$copnfigPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json

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

$gRPCVersion = $config.gRPC.version
if ($config.gRPC.shared)
{
    $gRPCShared = "dynamic"
}
else
{
    $gRPCShared = "static"
}
$protobufVersion = $config.protobuf.version
if ($config.protobuf.shared)
{
    $protobufShared = "dynamic"
}
else
{
    $protobufShared = "static"
}

$GRPC_INSTALL_DIR = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath grpc | `
                    Join-Path -ChildPath $gRPCVersion | `
                    Join-Path -ChildPath $gRPCShared | `
                    Join-Path -ChildPath $Configuration
$GRPC_CMAKE_DIR = Join-Path $GRPC_INSTALL_DIR lib | `
                  Join-Path -ChildPath cmake | `
                  Join-Path -ChildPath grpc
$PROTOBUF_INSTALL_DIR = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath protobuf | `
                        Join-Path -ChildPath $protobufVersion | `
                        Join-Path -ChildPath $protobufShared | `
                        Join-Path -ChildPath $Configuration
$PROTOBUF_CMAKE_DIR = Join-Path $PROTOBUF_INSTALL_DIR lib | `
                      Join-Path -ChildPath cmake | `
                      Join-Path -ChildPath protobuf

if (!(Test-Path(${GRPC_CMAKE_DIR})))
{
    Write-Host "${GRPC_CMAKE_DIR} is missing" -ForegroundColor Red
    return
}
if (!(Test-Path(${PROTOBUF_CMAKE_DIR})))
{
    Write-Host "${PROTOBUF_CMAKE_DIR} is missing" -ForegroundColor Red
    return
}

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $PROTOC_BIN = Join-Path $PROTOBUF_INSTALL_DIR bin | `
                  Join-Path -ChildPath proto.exe
    $GRPC_C_PLUGIN_BIN = Join-Path $GRPC_INSTALL_DIR bin | `
                         Join-Path -ChildPath grpc_cpp_plugin.exe

    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${GRPC_INSTALL_DIR};${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_CXX_STANDARD=17"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_DIR=${GRPC_CMAKE_DIR}"
        "${sourceDir}"
    )
}
elseif ($global:IsMacOS)
{
    $PROTOC_BIN = Join-Path $PROTOBUF_INSTALL_DIR bin | `
                  Join-Path -ChildPath proto
    $GRPC_C_PLUGIN_BIN = Join-Path $GRPC_INSTALL_DIR bin | `
                         Join-Path -ChildPath grpc_cpp_plugin

    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${GRPC_INSTALL_DIR};${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_DIR=${GRPC_CMAKE_DIR}"
        "${sourceDir}"
    )
}
elseif ($global:IsLinux)
{
    $PROTOC_BIN = Join-Path $PROTOBUF_INSTALL_DIR bin | `
                  Join-Path -ChildPath proto
    $GRPC_C_PLUGIN_BIN = Join-Path $GRPC_INSTALL_DIR bin | `
                         Join-Path -ChildPath grpc_cpp_plugin

    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${GRPC_INSTALL_DIR};${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_DIR=${GRPC_CMAKE_DIR}"
        "${sourceDir}"
    )
}

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
cmake --build . --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location