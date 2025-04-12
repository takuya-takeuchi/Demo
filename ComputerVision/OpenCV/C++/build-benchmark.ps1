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

$target = "benchmark"
$shared = "static"

# build
$sourceDir = Join-Path $current $target
$buildDir = Join-Path $current build | `
            Join-Path -ChildPath $os | `
            Join-Path -ChildPath $target | `
            Join-Path -ChildPath $shared
$installDir = Join-Path $current install | `
              Join-Path -ChildPath $os | `
              Join-Path -ChildPath $target | `
              Join-Path -ChildPath $shared

New-Item -Type Directory $buildDir -Force | Out-Null
New-Item -Type Directory $installDir -Force | Out-Null

Push-Location $buildDir
if ($global:IsWindows)
{    
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D BENCHMARK_DOWNLOAD_DEPENDENCIES=ON `
          $sourceDir
}
elseif ($global:IsMacOS)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D BENCHMARK_DOWNLOAD_DEPENDENCIES=ON `
          $sourceDir
}
elseif ($global:IsLinux)
{
    cmake -D CMAKE_INSTALL_PREFIX=${installDir} `
          -D BUILD_SHARED_LIBS=OFF `
          -D BENCHMARK_DOWNLOAD_DEPENDENCIES=ON `
          $sourceDir
}
cmake --build . --config ${Configuration} --target install
Pop-Location