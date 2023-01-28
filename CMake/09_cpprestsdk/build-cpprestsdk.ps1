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

# update submodule
git submodule update --init --recursive .

# get os name
if ($global:IsWindows)
{
    $os = "win"
}
elseif ($global:IsMacOS)
{
    $os = "osx"
    Push-Location
    Pop-Location
}
elseif ($global:IsLinux)
{
    $os = "linux"
}

$target = "cpprestsdk"

$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target

$boostDir = Join-Path $current install | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath boost | `
            Join-Path -ChildPath 1.81.0 | `
            Join-Path -ChildPath lib | `
            Join-Path -ChildPath cmake

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

# path Fix likely typo in SafeInt3.hpp, that results in error with clang 15 (https://github.com/microsoft/cpprestsdk/pull/1711)
if ($global:IsMacOS)
{
    Push-Location $sourceDir
    git checkout e1b6a8e61d6b3ab98734b0532bad1da46458212a
    Pop-Location
}

Push-Location $buildDir
if ($global:IsWindows)
{
    cmake -D CMAKE_INSTALL_PREFIX=$installDir `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH=${boostDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D Boost_USE_STATIC_LIBS=ON `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=$installDir `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH=${boostDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D Boost_USE_STATIC_LIBS=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=$installDir `
          -D CMAKE_BUILD_TYPE=$Configuration `
          -D CMAKE_PREFIX_PATH=${boostDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D Boost_USE_STATIC_LIBS=ON `
          $sourceDir
}
cmake --build . --config $Configuration --target install
Pop-Location