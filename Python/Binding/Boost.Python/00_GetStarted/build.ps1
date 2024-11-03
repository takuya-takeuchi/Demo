#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: Version (e.g. 1.84.0)
#%3: PythonPath (e.g. C:\Python\3.11\x64\python.exe)
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
   $Version,
   
   [Parameter(
   Mandatory=$True,
   Position = 3
   )][string]
   $PythonPath
)

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent

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
$sharedFlag = "OFF"

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

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$installBinaryDir = Join-Path $installDir bin

$targetInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $Version | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath lib | `
                    Join-Path -ChildPath cmake

if (!(Test-Path($targetInstallDir)))
{
    Write-Host "[Error] '${targetInstallDir}' is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          -D Python_FIND_VERSION="${pythonVersion}" `
          -D Python_EXECUTABLE="${PythonPath}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location