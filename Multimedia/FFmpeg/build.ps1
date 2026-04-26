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

$ConfigurationArray =
@(
   "Debug",
   "Release",
   "RelWithDebInfo",
   "MinSizeRel"
)

if ($ConfigurationArray.Contains($Configuration) -eq $False)
{
   $candidate = $ConfigurationArray -join "/"
   Write-Host "Specify build configuration [${candidate}]" -ForegroundColor Red
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

$buildTargets = @()
$buildTargets += New-Object PSObject -Property @{ Option = "--enable-optimizations";                           Flag = ($Configure -ne "Debug") }
$buildTargets += New-Object PSObject -Property @{ Option = "--disable-debug";                                  Flag = ($Configure -ne "Debug") }
$buildTargets += New-Object PSObject -Property @{ Option = "--enable-shared";                                  Flag = $sharedFlag }
$buildTargets += New-Object PSObject -Property @{ Option = "--disable-static";                                 Flag = $sharedFlag }
$buildTargets += New-Object PSObject -Property @{ Option = "--extra-ldflags=-static-libgcc -static-libstdc++"; Flag = !$config.ffmpeg.linkStaticRuntime }

$config.ffmpeg.options.standard | Where-Object { $_.flag } | ForEach-Object {
    $buildTargets += New-Object PSObject -Property @{ Option = $_.option; Flag = $_.flag }
}
$config.ffmpeg.options.documentation | Where-Object { $_.flag } | ForEach-Object {
    $buildTargets += New-Object PSObject -Property @{ Option = $_.option; Flag = $_.flag }
}
$config.ffmpeg.options.licensing | Where-Object { $_.flag } | ForEach-Object {
    $buildTargets += New-Object PSObject -Property @{ Option = $_.option; Flag = $_.flag }
}
$config.ffmpeg.options.component | Where-Object { $_.flag } | ForEach-Object {
    $buildTargets += New-Object PSObject -Property @{ Option = $_.option; Flag = $_.flag }
}
$config.ffmpeg.options.externalLibrarySupport | Where-Object { $_.flag } | ForEach-Object {
    $buildTargets += New-Object PSObject -Property @{ Option = $_.option; Flag = $_.flag }
}

$configureArgs = @()
foreach ($buildTarget in $buildTargets)
{
    $option = $buildTarget.Option
    $flag = $buildTarget.Flag
    if ($flag)
    {
        $configureArgs += @(
            $option
        )
    }
}

$pkgConfigPathList = @()
foreach ($item in $config.ffmpeg.options.externalLibrarySupport)
{
    $flag = $item.flag
    $option = $item.option
    if (!$flag)
    {
        continue
    }

    if ($null -ne $item.PSObject.Properties['version'])
    {
        $name = $option.Replace("--enable-", "")
        $scriptPath = Join-Path $current "scripts" | Join-Path -ChildPath "${name}.ps1"
        if (Test-Path($scriptPath))
        {
            $version = $item.version
            $buildExternalDir = Join-Path $current build | `
                                Join-Path -ChildPath $os | `
                                Join-Path -ChildPath $name | `
                                Join-Path -ChildPath $version
            $installExternalDir = Join-Path $current install | `
                                  Join-Path -ChildPath $os | `
                                  Join-Path -ChildPath $name | `
                                  Join-Path -ChildPath $version
            New-Item -Type Directory $installExternalDir -Force | Out-Null
            New-Item -Type Directory $buildExternalDir -Force | Out-Null
            Write-Host "[Info] Start build ${name}..." -ForegroundColor Green
            & $scriptPath -BuildDir $buildExternalDir -InstallDir $installExternalDir -Version $version
            Write-Host "[Info] Finish build ${name}" -ForegroundColor Green

            $pkgConfigPath = Join-Path ${installExternalDir} lib | Join-Path -ChildPath pkgconfig
            if (!(Test-Path($pkgConfigPath)))
            {
                Write-Host "[Warn] ${pkgConfigPath} is missing" -ForegroundColor Yellow
                continue
            }
            $pkgConfigPathList += ${pkgConfigPath}
        }
        else
        {
            Write-Host "[Info] Script for ${option} is missing. Skip build ${option}." -ForegroundColor Yellow
        }
    }
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

    & $shell -defterm -no-start -ucrt64 -here -c "pacman --needed -Sy bash pacman pacman-mirrors msys2-runtime --noconfirm"
    & $shell -defterm -no-start -ucrt64 -here -c "pacman -Syuu --noconfirm"
    & $shell -defterm -no-start -ucrt64 -here -c "pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-yasm mingw-w64-x86_64-pkg-config git make diffutils --noconfirm"

    function Convert-ToMsys2Path {
        param(
            [Parameter(Mandatory = $true, ValueFromPipeline = $true)]
            [string]$Path
        )

        process {
            if ([string]::IsNullOrWhiteSpace($Path)) {
                return ""
            }

            $p = $Path.Trim()
            $p = $p -replace '\\', '/'

            # replace drive letter
            if ($p -match '^([A-Za-z]):(/.*)?$') {
                $drive = $Matches[1].ToLower()
                $rest  = $Matches[2]

                if ([string]::IsNullOrEmpty($rest)) {
                    return "/$drive"
                }

                return "/$drive$rest"
            }

            # already msys2-path
            if ($p -match '^/[A-Za-z](/|$)') {
                return $p
            }

            # unc-path
            if ($p -match '^//') {
                return $p
            }

            # relative-path
            return $p
        }
    }

    $configureArgs += @(
        "--enable-cross-compile"
    )
    
    $installDir = Convert-ToMsys2Path $installDir
    $configureArgs += @(
        "--prefix=${installDir}"
    )

    # /bin/sh: line 1: strip: command not found
    $stripPath = Convert-ToMsys2Path "C:\msys64\mingw64\bin\strip.exe"
    $configureArgs += @(
        "--strip=${stripPath}"
    )

    $env:MSYSTEM = "MINGW64"
    $env:CHERE_INVOKING = "1"
    $bash = "C:\msys64\usr\bin\bash.exe"
    if (!(Test-Path($bash)))
    {
        Write-Host "${bash} is missing" -ForegroundColor Red
        exit
    }

    Write-Host "Build Options:" -ForegroundColor Green
    foreach ($arg in $configureArgs)
    {
        Write-Host "`t${arg}" -ForegroundColor Green
    }

    Write-Host "Start configure. It take a long time..." -ForegroundColor Blue
    $configure = Convert-ToMsys2Path $configure
    $configLogFile = Convert-ToMsys2Path $configLogFile
    $pkgConfigPath = ($pkgConfigPathList | Convert-ToMsys2Path) -Join ":"
    $pkgConfigPath = "${pkgConfigPath}"
    & $bash -lc "PATH=/mingw64/bin:`$PATH PKG_CONFIG_PATH=${pkgConfigPath} ${configure} ${configureArgs} 2>&1 | tee ${configLogFile}"
    
    Write-Host "Start build. It take a long time..." -ForegroundColor Blue
    $buildLogFile = Convert-ToMsys2Path $buildLogFile
    $nproc = [Environment]::ProcessorCount
    & $bash -lc "PATH=/mingw64/bin:`$PATH make -j ${nproc} 2>&1 | tee ${buildLogFile}"
    & $bash -lc "PATH=/mingw64/bin:`$PATH make install"
}
else
{
    $configureArgs += @(
        "--prefix=${installDir}"
    )

    Write-Host "Build Options:" -ForegroundColor Green
    foreach ($arg in $configureArgs)
    {
        Write-Host "`t${arg}" -ForegroundColor Green
    }

    $pkgConfigPath = $pkgConfigPathList -Join ":"
    $env:PKG_CONFIG_PATH = "${pkgConfigPath}:/usr/local/lib/pkgconfig"
    & "${configure}" @configureArgs 2>&1 | Tee-Object -FilePath $configLogFile
    make -j $nproc 2>&1 | Tee-Object -FilePath $buildLogFile
    make install
}

Pop-Location