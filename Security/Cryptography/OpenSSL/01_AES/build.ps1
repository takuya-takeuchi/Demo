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
   $Configuration,
   
   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Architecture
)

$ConfigurationArray =
@(
   "Debug",
   "Release",
   "RelWithDebInfo",
   "MinSizeRel"
)

$ArchitectureArray =
@(
   "arm64",
   "x86_64",
   "x86"
)

if ($ConfigurationArray.Contains($Configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Specify build configuration [${candidate}]" -ForegroundColor Red
   exit -1
}

if ($ArchitectureArray.Contains($Architecture) -eq $False)
{
   $candidate = $ArchitectureArray -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json
$version = $config.openssl.version
if ($config.openssl.shared)
{
    $shared = "dynamic"
}
else
{
    $shared = "static"
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
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $Architecture | `
            Join-Path -ChildPath $shared | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install| `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $Architecture | `
              Join-Path -ChildPath $shared | `
              Join-Path -ChildPath $Configuration

$OPENSSL_INSTALL_DIR = Join-Path $rootDir install | `
                       Join-Path -ChildPath openssl | `
                       Join-Path -ChildPath $version | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath $Architecture | `
                       Join-Path -ChildPath $shared | `
                       Join-Path -ChildPath $Configuration
$OPENSSL_CMAKECONFIG_FILE = (Get-ChildItem -Path $OPENSSL_INSTALL_DIR -Recurse -File | Where-Object { $_.Name -eq "OpenSSLConfig.cmake" } | Select-Object -First 1).FullName
$OPENSSL_CMAKE_DIR = Split-Path -Parent $OPENSSL_CMAKECONFIG_FILE

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir

$cmakeArgs = @()
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
    chcp 65001

    if ($config.windows.msvcStaticRuntime)
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    }
    else
    {
        $CMAKE_MSVC_RUNTIME_LIBRARY = "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
    }

    $env:OPENSSL_ROOT_DIR = "${OPENSSL_INSTALL_DIR}"
    $cmakeArgs += @(
        "-G", "Visual Studio 17 2022", "-A", "x64", "-T", "host=x64"
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${OPENSSL_CMAKE_DIR}"
        "-D CMAKE_MSVC_RUNTIME_LIBRARY=${CMAKE_MSVC_RUNTIME_LIBRARY}"
    )
}
elseif ($global:IsMacOS)
{
    $cmakeArgs += @(
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${OPENSSL_CMAKE_DIR}"
    )
}
elseif ($global:IsLinux)
{    
    $env:OPENSSL_ROOT_DIR = "${OPENSSL_INSTALL_DIR}"
    $cmakeArgs += @(
        "-D CMAKE_BUILD_TYPE=${Configuration}"
        "-D CMAKE_INSTALL_PREFIX=${installDir}"
        "-D CMAKE_PREFIX_PATH=${OPENSSL_CMAKE_DIR}"
    )
}

$cmakeArgs += @(
    "-D CMAKE_POLICY_VERSION_MINIMUM=3.5"
    "${sourceDir}"
)

$configLogFile = Join-Path $buildDir cmake-config.log
$buildLogFile = Join-Path $buildDir cmake-build.log

cmake @cmakeArgs 2>&1 | Tee-Object -FilePath $configLogFile
$nproc = [Environment]::ProcessorCount
cmake --build . --config ${Configuration} --target install --parallel $nproc 2>&1 | Tee-Object -FilePath $buildLogFile

Pop-Location