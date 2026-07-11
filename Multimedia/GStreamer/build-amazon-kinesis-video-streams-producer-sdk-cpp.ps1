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
$rootDir = $PSScriptRoot
$configPath = Join-Path $current "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

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

$target = "amazon-kinesis-video-streams-producer-sdk-cpp"
$version = $config."${target}".version

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $version | `
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

Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    function CallVisualStudioDeveloperConsole()
    {
        $vsVersion = $config.windows.visualStudioVersion
        $vs = "C:\Program Files\Microsoft Visual Studio\${vsVersion}"
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
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $vsVersion = $config.windows.visualStudioVersion
    $vsInternalVersion = $config.windows.visualStudioInternalVersion

    $cmakeArgs += @(
        "-G", "Visual Studio ${vsInternalVersion} ${vsVersion}", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}
elseif ($global:IsLinux)
{
    $hasApt = Get-Command apt -ErrorAction SilentlyContinue
    $hasDnf = Get-Command dnf -ErrorAction SilentlyContinue
    $hasYum = Get-Command yum -ErrorAction SilentlyContinue

    if ($hasApt)
    {
        $libpqInstallDir = Join-Path $current install | `
                           Join-Path -ChildPath $os | `
                           Join-Path -ChildPath libpq | `
                           Join-Path -ChildPath usr
        if (!(Test-Path(${libpqInstallDir})))
        {
            Write-Host "[Error] ${libpqInstallDir} is missing" -ForegroundColor Red
            return
        }
    }
    elseif ($hasDnf -or $hasYum)
    {
    }

    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}

# standard
PKG_CONFIG_PATH = "${gstreamerInstallDir}/lib/pkgconfig"
$cmakeArgs += @(
    "-D BUILD_GSTREAMER_PLUGIN=ON"
    "-D BUILD_DEPENDENCIES=ON"
    "-D BUILD_TEST=OFF"
    "-D OPENSSL_ROOT_DIR="
    "-D CMAKE_PREFIX_PATH=<gstreamer-install-dir>"
    "-D CMAKE_C_FLAGS=-D_GNU_SOURCE"
    "-D CMAKE_CXX_FLAGS=-D_GNU_SOURCE"
)

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

# $env:PKG_CONFIG_PATH = "${pkgConfigPath}:/usr/local/lib/pkgconfig"
cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location