#***************************************
#Arguments
#%1: Build Configuration (Release/Debug)
#%2: OpenSSL version (e.g. 0.18.0)
#***************************************
Param
(
   [Parameter(
   Mandatory=$True,
   Position = 1
   )][string]
   $Configuration,
   
   [Parameter(
   Mandatory=$True,
   Position = 2
   )][string]
   $Open3DVersion
)

$current = $PSScriptRoot
$rootDir = Split-Path $current -Parent

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

$target = "Open3D"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath program
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os
$open3DInstallDir = Join-Path $rootDir install | `
                    Join-Path -ChildPath $target | `
                    Join-Path -ChildPath $os | `
                    Join-Path -ChildPath ${Open3DVersion}

if (!(Test-Path($open3DInstallDir)))
{
    Write-Host "'${open3DInstallDir}' is missing" -ForegroundColor Red
    exit
}

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    # $env:OPEN3D_ROOT_DIR = "${open3DInstallDir}"
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${open3DInstallDir}" `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${open3DInstallDir}" `
          -D CMAKE_OSX_ARCHITECTURES="arm64" `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_PREFIX_PATH="${open3DInstallDir}" `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location