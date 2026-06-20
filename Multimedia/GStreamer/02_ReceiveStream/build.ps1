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
$configPath = Join-Path $rootDir "build-config.json"
if (!(Test-Path($configPath)))
{
    Write-Host "${configPath} is missing" -ForegroundColor Red
    exit
}

$config = Get-Content -Path $configPath | ConvertFrom-Json

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
$targetInstallDir = $targetInstallDir.Replace("`\", "/")

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

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          -D PKG_CONFIG_EXECUTABLE="${pkgConfigExe}" `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetInstallDir}" `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $config = "${targetInstallDir}/lib/x86_64-linux-gnu/pkgconfig"
    if (!(Test-Path(${config})))
    {
        Write-Host "[Error] ${config} is missing" -ForegroundColor Red
        return
    }
    
    cmake -E env PKG_CONFIG_PATH="${config}" `
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D PKG_CONFIG_USE_CMAKE_PREFIX_PATH=FALSE `
          -D GSTREAMER_ROOT="${targetInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location