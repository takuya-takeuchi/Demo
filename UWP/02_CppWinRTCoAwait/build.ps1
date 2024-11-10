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
else
{
    Write-Host "[Error] This platform is not supported" -ForegroundColor Red
    exit
}

$sdkVersions = @(
    # "10.0.10150.0"
    # "10.0.10240.0"
    # "10.0.10586.0"
    # "10.0.14393.0"
    # "10.0.15063.0"
    # "10.0.16299.0"
    # "10.0.17134.0"
    # "10.0.17763.0"
    # "10.0.18362.0"
    # "10.0.19041.0"
    "10.0.22000.0"
    # "10.0.22621.0"
)

foreach ($sdkVersion in $sdkVersions)
{
    # build
    $sourceDir = $current
    $buildDir = Join-Path $current build | `
                Join-Path -ChildPath $os | `
                Join-Path -ChildPath $sdkVersion | `
                Join-Path -ChildPath program
    $installDir = Join-Path $current install | `
                  Join-Path -ChildPath $os | `
                  Join-Path -ChildPath $sdkVersion
    
    New-Item -Type Directory $buildDir -Force | Out-Null
    New-Item -Type Directory $installDir -Force | Out-Null
    
    Push-Location $buildDir
    cmake -G "Visual Studio 17 2022" -A x64 -T host=x64 `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${targetDir}" `
          -D CMAKE_SYSTEM_NAME="WindowsStore" `
          -D CMAKE_SYSTEM_VERSION="${sdkVersion}" `
          $sourceDir
    cmake --build . --config ${Configuration} --target install
    Pop-Location
}