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
$PROTOBUF_INSTALL_DIR = Join-Path $current install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath protobuf | `
                        Join-Path -ChildPath $protobufVersion | `
                        Join-Path -ChildPath $protobufShared | `
                        Join-Path -ChildPath $Configuration
$PROTOBUF_CMAKE_DIR = Join-Path $PROTOBUF_INSTALL_DIR lib | `
                      Join-Path -ChildPath cmake | `
                      Join-Path -ChildPath protobuf

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

    if ($config.gRPC.shared)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreadedDLL"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded"
    }
    if ($Configuration -eq "Debug")
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY += "Debug"
    }

    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "${sourceDir}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "${sourceDir}"
    )
}
elseif ($global:IsLinux)
{
    $cmakeArgs = @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D Protobuf_DIR=${PROTOBUF_CMAKE_DIR}"
        "-D gRPC_INSTALL=ON"
        "-D gRPC_BUILD_TESTS=OFF"
        "-D gRPC_PROTOBUF_PROVIDER=package"
        "${sourceDir}"
    )
}

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
cmake --build . --config ${Configuration} --target install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location