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
$rootDir = Split-Path $current -Parent
$copnfigPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($copnfigPath)))
{
    Write-Host "${copnfigPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $copnfigPath | ConvertFrom-Json

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

$target = "onnxruntime-cuda"
$version = $config.onnxruntime.version
$openCVVersion = $config.opencv.version

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$installBinaryDir = Join-Path $installDir bin
$targetInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $version | `
                    Join-Path -ChildPath $Configuration
if (!(Test-Path(${targetInstallDir})))
{
    Write-Host "${targetInstallDir} is missing" -ForegroundColor Red
    return
}

$openCVInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath opencv | `
                    Join-Path -ChildPath $openCVVersion | `
                    Join-Path -ChildPath static | `
                    Join-Path -ChildPath $Configuration
$cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
if (!($cmakeModuleFile))
{
    Write-Host "OpenCVModules.cmake is missing"
    exit
}
$openCVInstallDir = Split-Path $cmakeModuleFile -Parent

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D ONNXRUNTIME_ROOT="${targetInstallDir}" `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D ONNXRUNTIME_ROOT="${targetInstallDir}" `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location