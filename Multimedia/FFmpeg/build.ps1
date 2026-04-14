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

Push-Location $sourceDir
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
    $msysRoot = "C:\msys64"
    $shell = Join-Path $msysRoot "msys2_shell.cmd"
    if (!(Test-Path($shell)))
    {
        Write-Host "${shell} is missing" -ForegroundColor Red
        exit
    }

    & $shell -defterm -no-start -ucrt64 -here -c "pacman --needed -Sy bash pacman pacman-mirrors msys2-runtime --noconfirm"
    & $shell -defterm -no-start -ucrt64 -here -c "pacman -Syuu --noconfirm"
    & $shell -defterm -no-start -ucrt64 -here -c "pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-yasm mingw-w64-x86_64-pkg-config git make diffutils --noconfirm"
}

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

$configLogFile = Join-Path $buildDir make-config.log
$buildLogFile = Join-Path $buildDir make-build.log

if ($global:IsWindows)
{
    $msysRoot = "C:\msys64"
    $shell = Join-Path $msysRoot "msys2_shell.cmd"
    if (!(Test-Path($shell)))
    {
        Write-Host "${shell} is missing" -ForegroundColor Red
        exit
    }

    & $shell -defterm -no-start -ucrt64 -here -c "${configure} @{configureArgs}" 2>&1 | Tee-Object -FilePath $configLogFile
    & $shell -defterm -no-start -ucrt64 -here -c  "make -j ${nproc}" 2>&1 | Tee-Object -FilePath $buildLogFile
    & $shell -defterm -no-start -ucrt64 -here -c  "make install" 2>&1 | Tee-Object -FilePath $buildLogFile
}
else
{
    & "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
    make -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    make install
}

Pop-Location