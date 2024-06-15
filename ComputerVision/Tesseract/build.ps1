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

$target = "tesseract"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $Configuration
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $Configuration
$leptonicaInstallDir = Join-Path $current install | `
                       Join-Path -ChildPath $os | `
                       Join-Path -ChildPath leptonica | `
                       Join-Path -ChildPath $Configuration | `
                       Join-Path -ChildPath lib | `
                       Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{
    if (!($env:VCPKG_ROOT_DIR))
    {
        Write-Host "VCPKG_ROOT_DIR environmental variable is missing" -ForegroundColor Red
        return
    }

    Write-Host "Build of Tesseract for Windows does not take care of build configuration (Release or Debug)" -ForegroundColor Yellow

    $vcpkg = Join-Path $env:VCPKG_ROOT_DIR vcpkg.exe
    & $vcpkg install tesseract --triplet x64-windows-static
}
elseif ($global:IsMacOS)
{
    $env:Leptonica_DIR="${leptonicaInstallDir}"
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_DISABLE_FIND_PACKAGE_PkgConfig=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    $env:Leptonica_DIR="${leptonicaInstallDir}"
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location