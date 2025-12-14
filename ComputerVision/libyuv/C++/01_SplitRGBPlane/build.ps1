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
$openBenchmarkDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath benchmark | `
                    Join-Path -ChildPath $shared

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory (Join-Path $installDir bin) -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openBenchmarkDir}" `
          -D libyuv_LIBRARIES="${libyuvInstallDir}\lib\yuv.lib" `
          -D libyuv_INCLUDE_DIRS="${libyuvInstallDir}\include" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openBenchmarkDir}" `
          -D libyuv_LIBRARIES="${libyuvInstallDir}/lib/libyuv.a" `
          -D libyuv_INCLUDE_DIRS="${libyuvInstallDir}/include" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $rootDir = Split-Path $current -Parent
    $openCVInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath "${target}-enable-jpegxl" | `
                        Join-Path -ChildPath $shared | `
                        Join-Path -ChildPath lib | `
                        Join-Path -ChildPath cmake | `
                        Join-Path -ChildPath $target

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir};${openBenchmarkDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location