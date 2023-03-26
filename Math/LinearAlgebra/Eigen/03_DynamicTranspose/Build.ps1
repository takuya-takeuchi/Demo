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

$target = "eigen"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $os

$rootDir = Split-Path $current -Parent
$eigenInstallDir = Join-Path $rootDir install | `
                   Join-Path -ChildPath $os | `
                   Join-Path -ChildPath $target | `
                   Join-Path -ChildPath share | `
                   Join-Path -ChildPath eigen3 | `
                   Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${eigenInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${eigenInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${eigenInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
if ($global:IsWindows)
{
    $exe = Join-Path "${installDir}" bin | `
           Join-Path -ChildPath "Test.exe"
    & "${exe}"
}
elseif ($global:IsLinux)
{
    $exe = Join-Path "${installDir}" bin | `
           Join-Path -ChildPath "Test"
    & "${exe}"
}
elseif ($global:IsMacOS)
{
    $exe = Join-Path "${installDir}" bin | `
           Join-Path -ChildPath "Test"
    & "${exe}"
}

Set-Location $current