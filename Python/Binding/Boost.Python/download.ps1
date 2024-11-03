#***************************************
#Arguments
#%1: Version (e.g. 1.84.0)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Version
)

$target = "boost"

$current = $PSScriptRoot

$filenameVersion = ${Version}.Replace(".", "_")

# get os name
if ($global:IsWindows)
{
    $os = "win"
    $filename = "boost_${filenameVersion}.zip"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    $filename = "boost_${filenameVersion}.tar.gz"
}
elseif ($global:IsLinux)
{
    $os = "linux"
    $filename = "boost_${filenameVersion}.tar.gz"
}

$filepath = Join-Path $current $filename
if (!(Test-Path("${filepath}")))
{
    $url = "https://boostorg.jfrog.io/artifactory/main/release/${Version}/source/${filename}"
    Write-Host "Download boost source code from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"
}

$baseName = "boost_${filenameVersion}"
$targetDir = Join-Path $current boost
$sourceDir = Join-Path $targetDir $filenameVersion
New-Item -Type Directory $targetDir -Force | Out-Null
if (!(Test-Path("${sourceDir}")))
{    
    if ($global:IsWindows)
    {
        Expand-Archive -Path "${filename}" -DestinationPath $targetDir
        
        $sourceDir = Join-Path $targetDir $baseName
        $targetDir = Join-Path $targetDir $filenameVersion
        Move-Item $sourceDir $targetDir
    }
    else
    {
        tar -xzf "${filename}" -C "${targetDir}"
        
        $sourceDir = Join-Path $targetDir $baseName
        $targetDir = Join-Path $targetDir $filenameVersion
        Move-Item $sourceDir $targetDir
    }
}
