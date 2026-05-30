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

if ($ArchitectureArray.Contains($architecture) -eq $False)
{
   $candidate = $ArchitectureArray -join "/"
   Write-Host "Error: Specify Architecture [${candidate}]" -ForegroundColor Red
   exit -1
}

$current = $PSScriptRoot
$configPath = Join-Path $current "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json
$target = "openssl"
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
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $version | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $Architecture | `
            Join-Path -ChildPath $shared | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $version | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $Architecture | `
              Join-Path -ChildPath $shared | `
              Join-Path -ChildPath $Configuration

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $current
# it may be deleted
git checkout $target
Pop-Location

Push-Location $sourceDir
git clean -fxd .
git fetch -ap
git checkout $version
# it takes so long time....
git submodule update --init --recursive .
Pop-Location

# apply patch
$patch = Join-Path $current patch |
         Join-Path -ChildPath cxxopts |
         Join-Path -ChildPath $version |
         Join-Path -ChildPath $os
if (Test-Path($patch))
{
    Copy-Item -Recurse $patch/* $sourceDir -Force
}

Push-Location $buildDir

$configure = Join-Path $sourceDir "Configure"
if (!(Test-Path($configure)))
{
    Write-Host "${configure} is missing" -ForegroundColor Red
    exit
}

$configLogFile = Join-Path $buildDir make-config.log
$buildLogFile = Join-Path $buildDir make-build.log

$configureArgs = @()
if ($global:IsWindows)
{
    $perlDir = $env:PERLPATH
    if (!$perlDir)
    {
        Write-Host "[Error] Environmental Variable: PERLPATH is missing" -ForegroundColor Red
        exit
    }

    if (!(Test-Path("${perlDir}")))
    {
        Write-Host "[Error] '${perlDir}' is missing" -ForegroundColor Red
        exit
    }

    $nasmDir = $env:NASMPATH
    if (!$nasmDir)
    {
        Write-Host "[Error] Environmental Variable: NASMPATH is missing" -ForegroundColor Red
        exit
    }

    if (!(Test-Path("${nasmDir}")))
    {
        Write-Host "[Error] '${nasmDir}' is missing" -ForegroundColor Red
        exit
    }

    $env:PATH="${perlDir};${nasmDir};${env:PATH}"

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

    $configureArgs += @(
        $targetAbi
    )
}
elseif ($global:IsMacOS)
{
    $targetAbi = "darwin64-${Architecture}-cc"

    $env:SDK = "macosx"
    $env:MACOSX_MIN_VERSION = $config.osx.macosxMinVersion

    $SDKROOT = (& xcrun --sdk $env:SDK --show-sdk-path).Trim()
    $CLANG = (& xcrun --sdk $env:SDK -find clang).Trim()
    $AR = (& xcrun --sdk $env:SDK -find ar).Trim()
    $RANLIB = (& xcrun --sdk $env:SDK -find ranlib).Trim()

    $env:SDKROOT = $SDKROOT
    $env:CC = $CLANG
    $env:AR = $AR
    $env:RANLIB = $RANLIB
    $env:CFLAGS = "-arch ${Architecture} -isysroot `"$SDKROOT`" -mmacosx-version-min=$env:MACOSX_MIN_VERSION"

    $configureArgs += @(
        $targetAbi
    )
}
elseif ($global:IsLinux)
{
    if ($Architecture -eq "arm64")
    {
        $Architecture = "aarch64"
    }

    $configureArgs += @(
        "linux-${Architecture}"
    )
}

$configureArgs += @(
    "no-docs",
    "no-tests",
    "no-apps",
    "no-module",
    "no-dso",
    "no-engine",
    "no-legacy",
    "no-ssl",
    "no-zlib"
)

if (!($config.openssl.shared))
{
    $configureArgs += @(
        "no-shared"
    )
}

$configureArgs += @(
    "--prefix=${installDir}"
)

if ($Configuration -eq "Debug")
{
    $configureArgs += @(
        "-d"
    )
}

if ($global:IsWindows)
{
    perl "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
    $nproc = [Environment]::ProcessorCount
    nmake -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    nmake install
}
else
{
    & "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
    make -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    make install
}

Pop-Location