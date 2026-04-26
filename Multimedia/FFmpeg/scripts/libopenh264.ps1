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

$packageName = "libopenh264"
$repositoryName = "openh264"
$repository = "https://github.com/cisco/${repositoryName}"

$current = $PSScriptRoot
$versionNumber = $Version.Replace("v", "")

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $libname = "openh264"
    $postfix = "win64.dll"
    $deployName = "lib${libname}.dll"

    # deploy bzip2
    $zipRoot = Join-Path $installDir zip
    New-Item -Type Directory $zipRoot -Force | Out-Null
    $bzip2File = Join-Path $zipRoot "bzip2-1.0.5-bin.zip"
    Invoke-WebRequest -Uri "http://downloads.sourceforge.net/gnuwin32/bzip2-1.0.5-bin.zip" -UserAgent "wget"  -OutFile $bzip2File
    Expand-Archive -LiteralPath $bzip2File -DestinationPath $zipRoot -Force
    $bzipDir = Join-Path $zipRoot bin
    $env:PATH = "${env:PATH};${bzipDir}"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $libname = "libopenh264"
    $postfix = "mac-arm64.dylib"
    $deployName = "${libname}.${versionNumber}.dylib"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $libname = "libopenh264"
    $postfix = "linux64.8.so"
    $deployName = "${libname}.so.${versionNumber}"
}

$binaryFileName = "${libname}-${versionNumber}-${postfix}"
$archiveFileName = "${libname}-${versionNumber}-${postfix}.bz2"
$hashFileName = "${libname}-${versionNumber}-${postfix}.signed.md5.txt"

function Get-FileHashValue {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path $Path)) {
        return $null
    }

    return (Get-FileHash -Path $Path -Algorithm MD5).Hash.ToLowerInvariant()
}

function Download-IfNeeded {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Url,

        [Parameter(Mandatory = $true)]
        [string]$OutFile,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedHash
    )

    $ExpectedHash = $ExpectedHash.ToLowerInvariant()

    $BinaryFile = $OutFile.Replace(".bz2", "")
    if (Test-Path $BinaryFile) {
        $currentHash = Get-FileHashValue -Path $BinaryFile
        if ($currentHash -eq $ExpectedHash) {
            Write-Host "[Info] [${packageName}] Hash matched: $BinaryFile" -ForegroundColor Yellow
            return
        }

        Write-Host "[Info] [${packageName}] Hash mismatch for '$BinaryFile'. Expected=$ExpectedHash Actual=$currentHash" -ForegroundColor Yellow
        Remove-Item $BinaryFile -Force
    }

    Write-Host "[Info] [${packageName}] Downloading from $Url to $OutFile" -ForegroundColor Yellow
    Invoke-WebRequest -Uri $Url -OutFile $OutFile
    $Destination = Split-Path $OutFile -Parent
    Expand-ArchiveForce -ArchivePath $OutFile -Destination $Destination

    $downloadedHash = Get-FileHashValue -Path $BinaryFile
    if ($downloadedHash -ne $ExpectedHash) {
        Remove-Item $OutFile -Force -ErrorAction SilentlyContinue
        throw "Hash mismatch for '${OutFile}'. Expected=${ExpectedHash} Actual=${downloadedHash}"
    }

    Write-Host "[Info] [${packageName}] Verified: $OutFile" -ForegroundColor Yellow
}

function Expand-ArchiveForce {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ArchivePath,

        [Parameter(Mandatory = $true)]
        [string]$Destination
    )

    if (-not (Test-Path $Destination)) {
        New-Item -ItemType Directory -Path $Destination | Out-Null
    }

    Write-Host "[Info] [${packageName}] Expand: $ArchivePath -> $Destination" -ForegroundColor Yellow
    Push-Location $Destination
    bzip2 -k -d $ArchivePath
    Pop-Location
}

# Download prebuilt binary from cisco
$binary = "http://ciscobinary.openh264.org/${archiveFileName}"
$hash = "http://ciscobinary.openh264.org/${hashFileName}"

$response = Invoke-WebRequest -Uri $hash
$bytes = $response.RawContentStream.ToArray()
$content = [System.Text.Encoding]::UTF8.GetString($bytes)
$content = ($content -split "`r?`n")[0]
if ($global:IsWindows)
{
    $hash = $content.Substring(0, 32);
}
else
{
    $hash = $content.Substring($content.Length - 32, 32);
}
$binaryPath = Join-Path $InstallDir $binaryFileName
$archivePath = Join-Path $InstallDir $archiveFileName

if ((Test-Path $archivePath) -and !(Test-Path $binaryPath))
{
    Expand-ArchiveForce -ArchivePath $archivePath -Destination $InstallDir
}

if (!(Test-Path $binaryPath))
{
    Download-IfNeeded -Url $binary -OutFile $archivePath -ExpectedHash $hash  
}

$downloadedHash = Get-FileHashValue -Path $binaryPath
if ($downloadedHash -ne $hash)
{
    Remove-Item $binaryPath -Force -ErrorAction SilentlyContinue
    throw "[Error] [${packageName}] Hash mismatch for '${binaryPath}'. Expected=${hash} Actual=${downloadedHash}"
}

# build openh264
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

    $env:MSYSTEM = "MINGW64"
    $env:CHERE_INVOKING = "1"
    $bash = "C:\msys64\usr\bin\bash.exe"
    if (!(Test-Path($bash)))
    {
        Write-Host "${bash} is missing" -ForegroundColor Red
        exit
    }
    
    Write-Host "Start build. It take a long time..." -ForegroundColor Blue
    $installMsysDir = $installDir.Replace("`\", "/").Replace(":", "")
    $buildLogFile = $buildLogFile.Replace("`\", "/").Replace(":", "")
    Push-Location $sourceDir
    & $bash -lc "PATH=/mingw64/bin:`$PATH make PREFIX=/${installMsysDir} install-shared 2>&1 | tee /${buildLogFile}"
    Pop-Location

    $deployPath = Join-Path $InstallDir bin | Join-Path -ChildPath $deployName
    Copy-Item -Path ${binaryPath} -Destination ${deployPath} -Force
}
else
{
    Push-Location $sourceDir
    make PREFIX="${InstallDir}" install-shared 2>&1 | Tee-Object -FilePath $buildLogFile
    Pop-Location

    $deployPath = Join-Path $InstallDir lib | Join-Path -ChildPath $deployName
    Copy-Item -Path ${binaryPath} -Destination ${deployPath} -Force
}

Write-Host "[Info] [${packageName}] All packages are ready in: $InstallDir" -ForegroundColor Green
