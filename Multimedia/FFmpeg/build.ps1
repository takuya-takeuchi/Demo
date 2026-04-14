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
$target = "ffmpeg"
$version = $config.ffmpeg.version
if ($config.ffmpeg.shared)
{
    $shared = "dynamic"
    $sharedFlag = $True
}
else
{
    $shared = "static"
    $sharedFlag = $False

    Write-Host "WARNING!!!!" -ForegroundColor Yellow
    Write-Host "When distributing an application using static linking, you must distribute either the application's object code or source code." -ForegroundColor Yellow
    Write-Host "" -ForegroundColor Yellow
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

Push-Location $target
git fetch -ap
git checkout $version
git submodule update --init --recursive .
Pop-Location

# apply patch
$patch = Join-Path $current patch |
         Join-Path -ChildPath ffmpeg |
         Join-Path -ChildPath $version |
         Join-Path -ChildPath $os
if (Test-Path($patch))
{
    Copy-Item -Recurse $patch/* $sourceDir -Force
}

Push-Location $buildDir

$configure = Join-Path $sourceDir "configure"
if (!(Test-Path($configure)))
{
    Write-Host "${pconfigureath} is missing" -ForegroundColor Red
    exit
} 

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

    $configureArgs = @(
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
    $configureArgs = @(
        "--prefix=${installDir}"
        "--disable-logging"
        "--disable-doc"
        "--disable-htmlpages"
        "--disable-manpages"
        "--disable-podpages"
        "--disable-txtpages"
        "--disable-avdevice"
    )

    if ($sharedFlag)
    {
        $configureArgs += @(
            "--enable-shared"
        )
    }

    if (!($config.ffmpeg.enableGpl))
    {
        $configureArgs += @(
            "--disable-gpl"
        )
    }

    if (!($config.ffmpeg.enableNonFree))
    {
        $configureArgs += @(
            "--enable-nonfree"
        )
    }        

    if ($Configure -ne "Debug")
    {
        $configureArgs += @(
            "--enable-optimizations"
            "--disable-debug"
        )
    }

    if (!($config.ffmpeg.linkStaticRuntime))
    {
        $configureArgs += @(
            "--extra-ldflags=-static-libgcc -static-libstdc++"
        )
    }
}
elseif ($global:IsLinux)
{
    $configureArgs = @(
        "--prefix=${installDir}"
        "--disable-logging"
        "--disable-doc"
        "--disable-htmlpages"
        "--disable-manpages"
        "--disable-podpages"
        "--disable-txtpages"
        "--disable-avdevice"
    )

    if ($sharedFlag)
    {
        $configureArgs += @(
            "--enable-shared"
        )
    }

    if (!($config.ffmpeg.enableGpl))
    {
        $configureArgs += @(
            "--disable-gpl"
        )
    }

    if (!($config.ffmpeg.enableNonFree))
    {
        $configureArgs += @(
            "--enable-nonfree"
        )
    }        

    if ($Configure -ne "Debug")
    {
        $configureArgs += @(
            "--enable-optimizations"
            "--disable-debug"
        )
    }

    if (!($config.ffmpeg.linkStaticRuntime))
    {
        $configureArgs += @(
            "--extra-ldflags=-static-libgcc -static-libstdc++"
        )
    }
}

$configLogFile = Join-Path $buildDir make-config.log
$buildLogFile = Join-Path $buildDir make-build.log

& "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
make -j $nproc
make install 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location