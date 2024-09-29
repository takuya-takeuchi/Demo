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

if (!($global:IsWindows))
{
    Write-Host "Error: This plaform is not support" -ForegroundColor Red
    exit -1
}

$current = $PSScriptRoot

# get os name
$os = "win"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

# find Windows SDK
$programFiles = ${env:ProgramFiles(x86)}
Get-ChildItem Env:
Write-Host "programFiles: ${programFiles}" -ForegroundColor Green
$windowsKitRoot = Join-Path "${programFiles}" "Windows Kits" | `
                  Join-Path -ChildPath "10"
$windowsKitIncludeRoot = Join-Path $windowsKitRoot Include
$directories = Get-ChildItem -Directory $windowsKitIncludeRoot
foreach ($directory in $directories)
{
    Write-Host "${directory}" -ForegroundColor Green
}
# "C:\Program Files (x86)\Windows Kits\10\Include\10.0.22621.0\shared\dxgi.h"

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_PREFIX_PATH="${targetDir}" `
      $sourceDir
cmake --build . --config ${Configuration} --target install
Pop-Location