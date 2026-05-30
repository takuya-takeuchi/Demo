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
   $Architecture,

   [Parameter(
   Mandatory=$False,
   Position = 3
   )][string]
   $Option
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

$OptionArray =
@(
   "android",
   "ios",
   "ios-simulator",
   "uwp"
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

if ($Option -and $OptionArray.Contains($Option) -eq $False)
{
   $candidate = $OptionArray -join "/"
   Write-Host "Error: Specify Option [${candidate}]" -ForegroundColor Red
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
if ($Option)
{
    $os = $Option
}
else
{
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
switch ($os)
{
    "win"
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

        $targets = @{
            "x86"    = "VC-WIN32"
            "x86_64" = "VC-WIN64A"
            "arm64"  = "VC-WIN64-ARM"
        }

        $configureArgs += @(
            $targets[$Architecture]
        )
    }
    "osx"
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
    "linux"
    {
        $targets = @{
            "x86"    = "linux-x86"
            "x86_64" = "linux-x86_64"
            "arm64"  = "linux-aarch64"
        }
        
        $configureArgs += @(
            $targets[$Architecture]
        )
    }
    "android"
    {
        $androidHome = $env:ANDROID_HOME
        if (!(${androidHome}))
        {
            Write-Host "Error: ANDROID_HOME is missing" -ForegroundColor Red
            exit -1
        }
        if (!(Test-Path ${androidHome}))
        {
            Write-Host "Error: ANDROID_HOME: ${androidHome} is missing" -ForegroundColor Red
            exit -1
        }
        $androidNdkHome = $env:ANDROID_NDK_HOME
        if (!(${androidNdkHome}))
        {
            Write-Host "Error: ANDROID_NDK_HOME is missing" -ForegroundColor Red
            exit -1
        }
        if (!(Test-Path ${androidNdkHome}))
        {
            Write-Host "Error: ANDROID_NDK_HOME: ${androidNdkHome} is missing" -ForegroundColor Red
            exit -1
        }

        $env:ANDROID_NDK_ROOT = $env:ANDROID_NDK_HOME
        if ($global:IsWindows)
        {
            $env:TOOLCHAIN="$env:ANDROID_NDK_HOME/toolchains/llvm/prebuilt/windows-x86_64"
        }
        elseif ($global:IsMacOS)
        {
            $env:TOOLCHAIN="$env:ANDROID_NDK_HOME/toolchains/llvm/prebuilt/darwin-x86_64"
        }
        elseif ($global:IsLinux)
        {
            $env:TOOLCHAIN="$env:ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64"
        }
        $env:PATH="$env:TOOLCHAIN/bin:$env:PATH"

        $env:ANDROID_API=${ANDROID_NATIVE_API_LEVEL}
        $env:CC="aarch64-linux-android${ANDROID_NATIVE_API_LEVEL}-clang"
        $env:AR="llvm-ar"
        $env:RANLIB="llvm-ranlib"
        $env:STRIP="llvm-strip"

        $targets = @{
            "x86"    = "android-x86"
            "x86_64" = "android-x86_64"
            "arm64"  = "android-arm64"
        }
        
        $configureArgs += @(
            $targets[$Architecture]
        )
    }
    "ios"
    {
        
    }
    "ios-simulator"
    {
        
    }
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
    # It may occur exhasted memory when using multiple processes. So, use single process.
    # $nproc = [Environment]::ProcessorCount
    # nmake -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    nmake 2>&1 | Tee-Object -FilePath $buildLogFile
    nmake install
}
else
{
    & "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
    make -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    make install
}

Pop-Location