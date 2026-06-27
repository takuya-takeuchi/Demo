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
$configPath = Join-Path $rootDir "build-config.json"
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

$target = "libpqxx"
$version = $config.libpqxx.version
if ($config.libpqxx.shared)
{
    $shared = "dynamic"
}
else
{
    $shared = "static"
}

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$installBinaryDir = Join-Path $installDir bin
$targetInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath $shared | `
                    Join-Path -ChildPath $Configuration
if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "[Error] ${targetInstallDir} is missing" -ForegroundColor Red
    return
}

$libpqInstallRootDir = Join-Path $rootDir install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath libpq
if (!(Test-Path(${libpqInstallRootDir})))
{
    Write-Host "[Error] ${libpqInstallRootDir} is missing" -ForegroundColor Red
    return
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null


Push-Location $buildDir

$cmakeArgs = @()
if ($global:IsWindows)
{
    $libpqInstallDir = Join-Path $libpqInstallRootDir pgsql
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
        "-D CMAKE_PREFIX_PATH=${targetInstallDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D BUILD_SHARED_LIBS=$sharedFlag"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    $libpqInstallDir = Join-Path $libpqInstallRootDir pgsql
    if (!(Test-Path(${libpqInstallDir})))
    {
        Write-Host "[Error] ${libpqInstallDir} is missing" -ForegroundColor Red
        return
    }

    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${targetInstallDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D PostgreSQL_ROOT=${libpqInstallDir}"
    )
}
elseif ($global:IsLinux)
{
    $pkgconfig = "${libpqInstallRootDir}/usr/lib/x86_64-linux-gnu/pkgconfig"
    if (!(Test-Path(${pkgconfig})))
    {
        Write-Host "[Error] ${pkgconfig} is missing" -ForegroundColor Red
        return
    }

    $version = $config.libpq.linux.version
    $majorVersion = $version.Split(".")[0]

    $libdir = "${libpqInstallRootDir}/usr/lib/postgresql/${majorVersion}/lib"
    if (!(Test-Path(${libdir})))
    {
        Write-Host "[Error] ${libdir} is missing" -ForegroundColor Red
        return
    }
    
    $env:PKG_CONFIG_PATH="${pkgconfig}"
    $env:PKG_CONFIG_SYSROOT_DIR="${libpqInstallRootDir}"
    $cmakeArgs += @(
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${targetInstallDir}"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D PostgreSQL_ROOT=${libpqInstallRootDir}/usr"
        "-D PostgreSQL_SERVER_DEV_DIR=${libdir}"
        "-D libpqxx_LIBRARY_DIR=${targetInstallDir}/lib"
    )
}

$cmakeArgs += @(
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location