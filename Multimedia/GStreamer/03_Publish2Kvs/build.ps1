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

function Join-PathArray {
    [CmdletBinding()]
    param (
        [Parameter(Mandatory = $true, ValueFromPipeline = $true)]
        [string[]]$PathElements
    )

    process {
        if ($PathElements.Count -eq 0) { return }
        $result = $PathElements[0]
        for ($i = 1; $i -lt $PathElements.Count; $i++) { $result = Join-Path -Path $result -ChildPath $PathElements[$i] }
        return $result
    }
}

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

$target = "gstreamer-kvs"
$version = $config.gstreamer.version

# build
$sourceDir = $current
$buildDir = Join-PathArray -PathElements @($current, "build", $os, "program", $Configuration)
$installDir = Join-PathArray -PathElements @($current, "install", $os)
$installBinaryDir = Join-PathArray -PathElements @($installDir, "bin")
$targetInstallDir = Join-PathArray -PathElements @($rootDir, "install", $os, $target, $version, $Configuration)
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
    $pkgConfigExe = Join-PathArray -PathElements @($rootDir, "install", $os, "pkg-config", "bin", "pkg-config.exe")
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
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location