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

$target = "libyuv"
$shared = "static"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

$rootDir = Split-Path $current -Parent

$libyuvInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath "${target}"

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory (Join-Path $installDir bin) -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D libyuv_LIBRARIES="${libyuvInstallDir}\lib\yuv.lib" `
          -D libyuv_INCLUDE_DIRS="${libyuvInstallDir}\include" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D libyuv_LIBRARIES="${libyuvInstallDir}/lib/libyuv.a" `
          -D libyuv_INCLUDE_DIRS="${libyuvInstallDir}/include" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D libyuv_LIBRARIES="${libyuvInstallDir}/lib/libyuv.a" `
          -D libyuv_INCLUDE_DIRS="${libyuvInstallDir}/include" `
          -D CMAKE_POLICY_VERSION_MINIMUM=3.5 `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location