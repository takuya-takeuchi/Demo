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

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir | Out-Null
if ($global:IsWindows)
{
    cmake -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location | Out-Null

# run
if ($global:IsWindows)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo.exe
}
elseif ($global:IsMacOS)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo
}
elseif ($global:IsLinux)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo
    $env:LD_LIBRARY_PATH="/usr/local/lib"
}

Push-Location $programDir | Out-Null
& "${program}"
Pop-Location