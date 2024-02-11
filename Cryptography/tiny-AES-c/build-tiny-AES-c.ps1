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

$target = "tiny-AES-c"
$url = "https://github.com/kokke/tiny-AES-c"
$branch = "f06ac37fc31dfdaca2e0d9bec83f90d5663c319b"
$buildOptions = " --with-system --with-filesystem --with-program_options"

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

$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

if (!(Test-Path($target)))
{
    git clone $url
}

Push-Location $target
git checkout $branch

Push-Location $buildDir

if ($global:IsWindows)
{
    cmake -G "Visual Studio 17 2022" `
          -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
    $library = Join-Path $buildDir tiny-aes.lib
    $headerH = Join-Path $sourceDir aes.h
    $headerHpp = Join-Path $sourceDir aes.hpp
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
    $library = Join-Path $buildDir libtiny-aes.a
    $headerH = Join-Path $sourceDir aes.h
    $headerHpp = Join-Path $sourceDir aes.hpp
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D CMAKE_BUILD_TYPE=$Configuration `
          $sourceDir
    $library = Join-Path $buildDir libtiny-aes.a
    $headerH = Join-Path $sourceDir aes.h
    $headerHpp = Join-Path $sourceDir aes.hpp
}

cmake --build "${buildDir}" --config $Configuration

Copy-Item $library $installDir -Force | Out-Null
Copy-Item $headerH $installDir -Force | Out-Null
Copy-Item $headerHpp $installDir -Force | Out-Null

Pop-Location
Pop-Location