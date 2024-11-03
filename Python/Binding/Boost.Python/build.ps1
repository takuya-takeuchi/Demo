#***************************************
#Arguments
#%1: Version (e.g. 1.84.0)
#%2: PythonPath (e.g. C:\Python\3.11\x64\python.exe)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Version,
   
   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $PythonPath
)

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

$target = "boost"

if (!(Test-Path($PythonPath)))
{
    Write-Host "[Error] ${PythonPath} is missing" -ForegroundColor Red
    exit
}

$versionOutput = & "${PythonPath}" --version 2>&1
if ($versionOutput -match "\d+\.\d+\.\d+")
{
    $pythonVersion = $matches[0]
}
else
{
    Write-Host "[Error] `${PythonPath} --version` output unexpected value: ${versionOutput}" -ForegroundColor Red
    exit
}

$pythonRoot = Split-Path $PythonPath -Parent
$pythonOptions = " python=${PythonPath} python-version=${pythonVersion} python-root=${pythonRoot}"
$buildOptions = " --with-system --with-filesystem --with-python"

# build
$filenameVersion = ${Version}.Replace(".", "_")
$sourceDir = Join-Path $current $target | `
             Join-Path -ChildPath $filenameVersion
if (!(Test-Path($sourceDir)))
{
    Write-Host "[Error] ${sourceDir} is missing" -ForegroundColor Red
    exit
}

$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $Version | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $Version | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $sourceDir
if ($global:IsWindows)
{
    $bootstrap = Join-Path $sourceDir bootstrap.bat
    $b2 = Join-Path $sourceDir b2.exe
    & "${bootstrap}"
}
else
{
    $bootstrap = Join-Path $sourceDir bootstrap.sh
    $b2 = Join-Path $sourceDir b2
    & bash "${bootstrap}"
}

$argument =  "link=static "
$argument += "address-model=64 "
$argument += "install "
$argument += "-j2 "
$argument += "--prefix=${installDir} "
$argument += "${pythonOptions}"
$argument += "${buildOptions}"

Invoke-Expression -Command "${b2} ${argument}"
Pop-Location