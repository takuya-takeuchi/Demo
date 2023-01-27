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

if ($global:IsWindows)
{
    # Cmake show following error message when building on windows
    #
    #     CMake Warning at build/win/opencv/win-install/x64/vc17/staticlib/OpenCVConfig.cmake:116 (message):
    #       OpenCV: Include directory doesn't exist:
    #       'C:/Demo/Misc/15_VSCodeDebugWithCMake/build/win/opencv/win-install/include'.
    #       OpenCV installation may be broken.  Skip...
    #
    # I'm not sure why OpenCVConfig.cmake file can not specify proper directory.
    # Therefore, specify OpenCV_INCLUDE_DIRS and OpenCV_LIBS variable directory via OpenCV_DIR variable
    $opencvDir = Join-Path $installDir opencv
}
elseif ($global:IsMacOS)
{
    $opencvDir = Join-Path $installDir opencv | `
                 Join-Path -ChildPath lib | `
                 Join-Path -ChildPath cmake | `
                 Join-Path -ChildPath opencv4
}
elseif ($global:IsLinux)
{
    $opencvDir = Join-Path $installDir opencv | `
                 Join-Path -ChildPath lib | `
                 Join-Path -ChildPath cmake | `
                 Join-Path -ChildPath opencv4
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
$env:OpenCV_DIR=${opencvDir}
cmake -D CMAKE_INSTALL_PREFIX=$installDir `
      -D CMAKE_BUILD_TYPE=${Configuration} `
      $sourceDir
cmake --build . --config ${Configuration}
Pop-Location
