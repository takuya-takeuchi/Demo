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

$target = "opencv4"
$shared = "static"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installDir/bin/fonts -Force | Out-Null

$rootDir = Split-Path $current -Parent
$openCVInstallDir = Join-Path $rootDir install-enable-freetype | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $shared
$freetype_installDir = Join-Path $rootDir install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath freetype | `
                       Join-Path -ChildPath $shared
$harfbuzz_installDir = Join-Path $rootDir install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath harfbuzz | `
                       Join-Path -ChildPath $shared

Push-Location $buildDir
if ($global:IsWindows)
{
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }
    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir};${freetype_installDir};${harfbuzz_installDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }
    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir};${freetype_installDir};${harfbuzz_installDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $cmakeModuleFile = Get-ChildItem $openCVInstallDir -Recurse -include OpenCVModules.cmake | Select-Object -First 1
    if (!($cmakeModuleFile))
    {
        Write-Host "OpenCVModules.cmake is missing"
        exit
    }
    $openCVInstallDir = Split-Path $cmakeModuleFile -Parent

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${openCVInstallDir};${freetype_installDir};${harfbuzz_installDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location