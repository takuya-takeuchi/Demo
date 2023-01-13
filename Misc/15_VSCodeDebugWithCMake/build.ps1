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
    $opencvDir = Join-Path $installDir opencv | `
                 Join-Path -ChildPath x64
    $files = Get-ChildItem -Recurse OpenCVConfig.cmake
    foreach ($file in $files)
    {
        $opencvDir = Split-Path $file -Parent
        break
    }    
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
      -D OpenCV=${opencvDir} `
      $sourceDir
cmake --build . --config ${Configuration}
Pop-Location
