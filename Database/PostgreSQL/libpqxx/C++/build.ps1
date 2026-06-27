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
$configPath = Join-Path $current "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json
$target = "libpqxx"
$version = $config.libpqxx.version
if ($config.libpqxx.shared)
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
    $libpqInstallDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath libpq | `
                       Join-Path -ChildPath pgsql
    if (!(Test-Path(${libpqInstallDir})))
    {
        Write-Host "[Error] ${libpqInstallDir} is missing" -ForegroundColor Red
        return
    }

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
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
        "-D PostgreSQL_ROOT=$libpqInstallDir"
    )
}
elseif ($global:IsMacOS)
{
    $libpqInstallDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath libpq | `
                       Join-Path -ChildPath pgsql
    if (!(Test-Path(${libpqInstallDir})))
    {
        Write-Host "[Error] ${libpqInstallDir} is missing" -ForegroundColor Red
        return
    }

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
$cmakeArgs += @(
    "-D SKIP_BUILD_TEST=ON"
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