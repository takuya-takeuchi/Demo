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

$target = "gstreamer"
$version = $config.gstreamer.version

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
    Write-Host "[Error] ${targetInstallDir} is missing" -ForegroundColor Red
    return
}
$cmakeModuleFile = Get-ChildItem $targetInstallDir -Recurse -include FindGStreamer.cmake | Select-Object -First 1
if (!($cmakeModuleFile))
{
    Write-Host "[Error] FindGStreamer.cmake is missing" -ForegroundColor Red
    exit
}
$gstreamerInstallDir = (Split-Path $cmakeModuleFile -Parent).Replace("`\", "/")

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null
New-Item -Type Directory $installBinaryDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    $pkgConfigExe = Join-Path $rootDir install | `
                        Join-Path -ChildPath $os | `
                        Join-Path -ChildPath pkg-config | `
                        Join-Path -ChildPath bin | `
                        Join-Path -ChildPath pkg-config.exe
    if (!(Test-Path(${pkgConfigExe})))
    {
        Write-Host "[Error] ${pkgConfigExe} is missing. Please run ../download-pkg-config.ps1" -ForegroundColor Red
        return
    }

    # Multimedia\GStreamer\install\win\gstreamer\1.26.11\Release\bin\gstreamer-1.0-0.dll
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_MODULE_PATH="${gstreamerInstallDir}" `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_MODULE_PATH="${gstreamerInstallDir}" `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location