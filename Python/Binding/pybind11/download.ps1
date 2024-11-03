#***************************************
#Arguments
#%1: Version (e.g. 2.2.0)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Version
)

$target = "pybind11"

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

$filename = "v${Version}.zip"
$filepath = Join-Path $current $filename
$url = "https://github.com/pybind/pybind11"
if (!(Test-Path("${filepath}")))
{
    $url = "${url}/archive/refs/tags/v${Version}.zip"
    Write-Host "Download ${target} source code from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"
}

if (!(Test-Path($filepath)))
{
    Write-Host "Failed to download from ${url}" -ForegroundColor Red
    exit
}

$baseName = "${target}-${Version}"
$targetDir = Join-Path $current ${target}
$sourceDir = Join-Path $targetDir $Version
New-Item -Type Directory $targetDir -Force | Out-Null
if (!(Test-Path("${sourceDir}")))
{
    Expand-Archive -Path "${filename}" -DestinationPath $targetDir
    # Avoid access denied
    Start-Sleep -Seconds 5

    $sourceDir = Join-Path $targetDir $baseName
    $targetDir = Join-Path $targetDir $Version
    Move-Item $sourceDir $targetDir
}
