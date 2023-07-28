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

$rootDir = Split-Path $current -Parent
$torchInstallDir = Join-Path $rootDir install | `
                   Join-Path -ChildPath $os | `
                   Join-Path -ChildPath share | `
                   Join-Path -ChildPath cmake | `
                   Join-Path -ChildPath Torch
$protobufLibInstallDir = Join-Path $rootDir install | `
                         Join-Path -ChildPath $os | `
                         Join-Path -ChildPath lib

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $env:Protobuf_LIBRARIES="${protobufLibInstallDir}/libprotobuf.lib"
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${torchInstallDir};${protobufInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${torchInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

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
}
& ${program}