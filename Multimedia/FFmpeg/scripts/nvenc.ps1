#***************************************
#Arguments
#%1: InbstallRootDir
#%2: Version
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $BuildDir,

   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $InstallDir,

   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $Version
)

$packageName = "nvenc"
$repositoryName = "nv-codec-headers"
$repository = "https://github.com/FFmpeg/${repositoryName}"

$current = $PSScriptRoot

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

# build nv-codec-headers
$sourceDir = Join-Path $BuildDir "${repositoryName}"
if (Test-Path $sourceDir)
{
    Push-Location $sourceDir
    git fetch --all --prune
    git checkout $Version
    Pop-Location
}
else
{
    git clone -b ${Version} $repository $sourceDir
}

Push-Location $sourceDir

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
    
    $InstallDir = Convert-ToMsys2Path $InstallDir

    # /bin/sh: line 1: strip: command not found
    $stripPath = Convert-ToMsys2Path "C:\msys64\mingw64\bin\strip.exe"

    $env:MSYSTEM = "MINGW64"
    $env:CHERE_INVOKING = "1"
    $bash = "C:\msys64\usr\bin\bash.exe"
    if (!(Test-Path($bash)))
    {
        Write-Host "${bash} is missing" -ForegroundColor Red
        exit
    }
    
    Write-Host "Start build. It take a long time..." -ForegroundColor Blue
    $buildLogFile = Convert-ToMsys2Path $buildLogFile
    & $bash -lc "PATH=/mingw64/bin:`$PATH make install PREFIX=$InstallDir | tee ${buildLogFile}"
}
else
{
    make install PREFIX=$InstallDir | Tee-Object -FilePath $buildLogFile
}

Pop-Location

Write-Host "[Info] [${packageName}] All packages are ready in: $InstallDir" -ForegroundColor Green