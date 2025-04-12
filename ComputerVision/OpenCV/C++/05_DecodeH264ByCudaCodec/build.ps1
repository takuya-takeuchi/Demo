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
elseif ($global:IsLinux)
{
    $os = "linux"
}
else
{
    Write-Host "[Error] This platform does not support CUDA" -ForegroundColor Red
    exit
}

# check CUDA
if (!($env:CUDA_PATH))
{
    Write-Host "CUDA_PATH environmental variable is missing" -ForegroundColor Red
    return
}
if (!(Test-Path($env:CUDA_PATH)))
{
    Write-Host "${env:CUDA_PATH} is missing" -ForegroundColor Red
    return
}

$target = "opencv4"
$shared = "static"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$installBinaryDir = Join-Path $installDir bin

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $rootDir = Split-Path $current -Parent
    $openCVInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath "${target}-with-cudacodec"| `
                        Join-Path -ChildPath $shared
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }

    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $rootDir = Split-Path $current -Parent
    $openCVInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath "${target}-with-cudacodec" | `
                        Join-Path -ChildPath $shared | `
                        Join-Path -ChildPath lib | `
                        Join-Path -ChildPath cmake | `
                        Join-Path -ChildPath $target

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location