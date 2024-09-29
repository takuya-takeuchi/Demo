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
$windowsKitRoot = Join-Path "${env:ProgramFiles(x86)}" "Windows Kits" | `
                  Join-Path -ChildPath "10"
$windowsKitIncludeRoot = Join-Path $windowsKitRoot Include
if (!(Test-Path("${windowsKitIncludeRoot}")))
{
    Write-Host "Error: '${windowsKitIncludeRoot}' is missing" -ForegroundColor Red
    exit -1
}

$directories = Get-ChildItem -Directory $windowsKitIncludeRoot| Sort-Object Name -Descending
if ($directories.Count -eq 0)
{
    Write-Host "Error: '${windowsKitIncludeRoot}' has no sub directories" -ForegroundColor Red
    exit -1
}

$windowsKitIncludeRoot = $directories | Select-Object -First 1
Write-Host "${windowsKitIncludeRoot}" -ForegroundColor Green

$version = Split-Path -Leaf ${windowsKitIncludeRoot}
$windowsKitLibRoot = Split-Path -Parent ${windowsKitIncludeRoot}
$windowsKitLibRoot = Split-Path -Parent ${windowsKitLibRoot}
$windowsKitLibRoot = Join-Path $windowsKitLibRoot Lib | `
                     Join-Path -ChildPath $version | `
                     Join-Path -ChildPath um | `
                     Join-Path -ChildPath x64
if (!(Test-Path("${windowsKitLibRoot}")))
{
    Write-Host "Error: '${windowsKitLibRoot}' is missing" -ForegroundColor Red
    exit -1
}

Push-Location $buildDir
cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
      -D CMAKE_PREFIX_PATH="${targetDir}" `
      -D WINDOWSKIT_INCLUDE_DIR="${windowsKitIncludeRoot}" `
      -D WINDOWSKIT_LIBRARY_DIR="${windowsKitLibRoot}" `
      $sourceDir
cmake --build . --config ${Configuration} --target install
Pop-Location