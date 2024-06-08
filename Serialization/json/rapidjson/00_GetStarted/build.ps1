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

$target = "rapidjson"
$tagetCMakeName = "RapidJSON"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$installBinaryDir = Join-Path $installDir bin

$rootDir = Split-Path $current -Parent

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    # windows version does not generate RapidJSON-targets.cmake into under lib/cmake
    $targetInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target | `
                        Join-Path -ChildPath $Configuration | `
                        Join-Path -ChildPath cmake

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    $targetInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target | `
                        Join-Path -ChildPath $Configuration | `
                        Join-Path -ChildPath lib | `
                        Join-Path -ChildPath cmake

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location