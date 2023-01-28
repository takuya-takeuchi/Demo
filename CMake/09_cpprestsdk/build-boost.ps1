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

$target = "boost"
$boostVersion = "1.81.0"

$current = $PSScriptRoot

$filenameVersion = ${boostVersion}.Replace(".", "_")

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
    $url = "https://boostorg.jfrog.io/artifactory/main/release/${boostVersion}/source/${filename}"
    Write-Host "Download boost source code from ${url}" -ForegroundColor Blue
    Invoke-WebRequest "${url}" -OutFile "${filename}"
}

$targetDir = Join-Path $current boost
$sourceDir = Join-Path $targetDir "boost_${filenameVersion}"
New-Item -Type Directory $targetDir -Force | Out-Null
if (!(Test-Path("${sourceDir}")))
{    
    if ($global:IsWindows)
    {
        Expand-Archive -Path "${filename}" -DestinationPath $targetDir
    }
    else
    {
        tar -xzf "${filename}" -C "${targetDir}"
    }
}

$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $boostVersion
              
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir
if ($global:IsWindows)
{
    $bootstrap = Join-Path $sourceDir bootstrap.bat
    $b2 = Join-Path $sourceDir b2.exe
    & "${bootstrap}"
    & "${b2}" link=static `
              address-model=64 `
              install `
              -j2 `
              --prefix=${installDir} `
              --with-system `
              --with-date_time `
              --with-regex
}
else
{
    $bootstrap = Join-Path $sourceDir bootstrap.sh
    $b2 = Join-Path $sourceDir b2
    & bash "${bootstrap}"
    & "${b2}" link=static `
              address-model=64 `
              install `
              -j2 `
              --prefix=${installDir} `
              --with-system `
              --with-date_time `
              --with-regex
}
Pop-Location
