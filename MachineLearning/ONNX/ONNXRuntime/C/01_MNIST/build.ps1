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

$target = "onnxruntime"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program | `
            Join-Path -ChildPath $Configuration
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
                        Join-Path -ChildPath opencv | `
                        Join-Path -ChildPath static | `
                        Join-Path -ChildPath $Configuration
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }

    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    $rootDir = Split-Path $current -Parent
    $targetInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target-cuda | `
                        Join-Path -ChildPath $Configuration
    if (!(Test-Path(${targetInstallDir})))
    {
        Write-Host "${targetInstallDir} is missing" -ForegroundColor Red
        return
    }

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D ONNXRUNTIME_ROOT="${targetInstallDir}" `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $rootDir = Split-Path $current -Parent
    $openCVInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath opencv | `
                        Join-Path -ChildPath static | `
                        Join-Path -ChildPath $Configuration
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }

    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    $rootDir = Split-Path $current -Parent
    $targetInstallDir = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath $target-cuda | `
                        Join-Path -ChildPath $Configuration
    if (!(Test-Path(${targetInstallDir})))
    {
        Write-Host "${targetInstallDir} is missing" -ForegroundColor Red
        return
    }

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D ONNXRUNTIME_ROOT="${targetInstallDir}" `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location