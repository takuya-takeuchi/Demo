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

$target = "libicon"

# build
$sourceDir = $current
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

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

    $toolchain = "${env:VCPKG_ROOT_DIR}\scripts\buildsystems\vcpkg.cmake"
    if (!(Test-Path(${toolchain})))
    {
        Write-Host "${toolchain} is missing" -ForegroundColor Red
        return
    }

    $library_type = "x64-windows"
    $vcpkg_base_directory = "${env:VCPKG_ROOT_DIR}\installed\${library_type}"
    $Env:ICONV_ROOT="${vcpkg_base_directory}"
    $Env:ICONV_INCLUDE_DIR="${vcpkg_base_directory}\include\"
    $Env:ICONV_LIBRARY_DIR="${vcpkg_base_directory}\lib\"
    $Env:CHARSET_BINARY="${vcpkg_base_directory}\bin\charset-1.dll"
    $Env:ICONV_BINARY="${vcpkg_base_directory}\bin\iconv-2.dll"

    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location

# run
if ($global:IsWindows)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo.exe
}
elseif ($global:IsMacOS)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo
}
elseif ($global:IsLinux)
{
    $programDir = Join-Path $installDir bin
    $program = Join-Path $programDir Demo
    $env:LD_LIBRARY_PATH="/usr/local/lib"
}

Push-Location $programDir
& "${program}"
Pop-Location