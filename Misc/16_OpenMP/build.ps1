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
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -DCMAKE_C_COMPILER=/usr/bin/gcc `
          -DCMAKE_CXX_COMPILER=/usr/bin/g++ `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location