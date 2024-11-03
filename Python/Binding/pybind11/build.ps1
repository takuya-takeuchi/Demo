#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: Version (e.g. 2.2.0)
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
   $Version
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

$target = "pybind11"

# build
$sourceDir = Join-Path $current $target | `
             Join-Path -ChildPath $Version
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $Version | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $Version | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location